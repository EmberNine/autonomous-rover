# ROS 2 bridge (Pi ↔ Nucleo)

`ros_ws/src/rover_bridge` connects the Nucleo's binary link (protocol v1,
[tools/rover_link.py](../tools/rover_link.py)) to ROS 2 Jazzy. It runs on
the Orange Pi in a Docker container. Laptop ↔ Pi networking: WSL mirrored
mode with Fast DDS static peers, domain 42.

**Group 3: driving**, speed and steering (since 2026-10-22). Until
`/rover/arm`, the bridge sends nothing but HELLO, so the motor stays latched.
**Group 4: keyboard teleop** ([Teleop](#teleop-group-4)).
**Group 5: the Pi's low-battery shutdown** ([Low-battery shutdown](#low-battery-shutdown-group-5)).
**Group 6: the EKF and the pose in RViz** ([EKF and pose](#ekf-and-pose-group-6)).

## Driving (`safety.py`)

| Interface | Type | What it does |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (best effort, depth 1) | `linear.x` in m/s ÷ `meters_per_encoder_tick` (from `config/vehicle.cfg`) → counts/s, rounded. \|cps\| < 1000 is sent as 0 (counted, WARN); it's never rounded up, because the firmware coasts below 1000. \|cps\| > 5000 is clamped (counted). `angular.z` steers ([Steering](#steering), below). Every message is also the **operator heartbeat**. |
| `/rover/arm` | `std_srvs/Trigger` | Arms, after reading the board's latch reason (rules below). |
| `/rover/arm_force` | `std_srvs/Trigger` | The same, but also clears STALL_LOCKED, STALL_SATURATION and ESTIMATE_INVALID. It's a separate service, not a parameter, so forcing is one explicit call and can't be left switched on. |
| `/rover/estop` | `std_srvs/Trigger` | ESTOP ×3 and disarm. Driving needs `/rover/arm` again. |

**What the TX thread sends** (the port's only writer, every 20 ms):
- **DISARMED:** nothing.
- **ARMING:** one DRIVE 0 with the arm bit, then DRIVE 0 until a STATUS shows the motor no longer latched. If that doesn't happen within 0.5 s: ESTOP ×3.
- **ARMED:** a DRIVE with the commanded speed and steering (`steer_valid` on every DRIVE).
- Speeds are quantized by the firmware to 50-cps units: 0.1 m/s → DRIVE 2542 cps → the controller runs on 51 units (2550). STATUS `setpoint_cps` **echoes the DRIVE as received (2542)**, not the quantized value; `/diagnostics` shows both (below).

**Heartbeat** (the Pi's receive time of `/cmd_vel`):
- **After 300 ms without one:** DRIVE 0 with the steering centred (0 %), still armed; the next `/cmd_vel` drives again.
- **After 1 s without one:** ESTOP ×3 (latch reason 6) and disarm.
- **Logged, every time:** each stop is a WARN (`no /cmd_vel for 300 ms: DRIVE 0 (still armed)`), and the `/cmd_vel` that ends it an INFO (`/cmd_vel back after 550 ms: driving again`). A gap of 150 ms or more that didn't stop the wheel is an INFO too (`/cmd_vel gap 251 ms (no stop)`): the line's time is when `/cmd_vel` came back, on the Pi. At most one line per second for each of the three; what was held back is said within about a second (`+1 more "heartbeat stop" in the 1.0 s after the last such line, not logged (counted)`). (Until 2026-10-19 the node logged an event only when its text changed, so a drive's second stop went unlogged; and a held-back stop was mentioned only at the next stop, once 220 s later.)
- **Counted:** heartbeat stops (total, and since the last arm), the longest `/cmd_vel` gap since the arm (an ESTOP's included), the last stop's whole gap, and the gaps of 150 ms or more since the arm (stops included). In `/diagnostics` (operator) and the 10 s line.

This covers a Wi-Fi drop, which the firmware's watchdog can't see, because DRIVEs keep coming from the Pi.

**`/rover/arm` refuses** (and says why) when any of these holds:
- no HELLO_ACK yet;
- a test build (`build_flags ≠ 0`);
- no STATUS in the last 0.5 s;
- the motor is in FAULT;
- the battery isn't OK: LOW, CUTOFF, no pack, ADC fault, or removed. **On wall power, arming is refused**, so driving needs the LiPo;
- no `/cmd_vel` in the last 300 ms: start the operator first;
- the latch reason is STALL_*, or ESTIMATE_INVALID, without force.

It clears a latch from BOOT, HELLO, ESTOP or WATCHDOG.

**ESTOP ×3 and disarm** on `/rover/estop`, on the heartbeat's 1 s, on an arm that times out, and **on every way out of the node**: SIGINT, SIGTERM, SIGHUP, or an exception. The shutdown ESTOPs go out through the TX thread before anything is torn down. After them, nothing is sent again.

Also disarmed (no ESTOP needed):
- the board latching by itself while armed (a STATUS showing LATCHED or FAULT);
- a board reset.

`SIGKILL` can't send anything: the firmware's 500 ms watchdog latches the motor (latch reason 2).

### Steering

`/cmd_vel` → steering %, in `safety.steer_percent` (the bicycle model):

    δ = atan(wheelbase_meters · angular.z / linear.x), clamped to ±max_steering_angle_radians
    steer % = round(δ / max_steering_angle_radians · 100)

Both values come from `config/vehicle.cfg` (0.133 m; 0.204 rad, measured
2026-10-22 at `s ±100`). There's no ROS parameter for them. Example: 0.1 m/s
and 0.1 rad/s give δ = 0.132 rad, which is 65 %.

- **Sign:** ROS `angular.z` > 0 is counter-clockwise, a left turn (REP 103). The firmware's `s` > 0 is also left. The sign passes straight through.
- **Reverse** (`linear.x` < 0): the formula flips the sign by itself. Counter-clockwise while backing up means steering right, which is the bicycle model's answer.
- **Full lock:** a δ beyond ±max is clamped to ±100 % and counted (`steering clamped`, WARN).
- **v ≈ 0:** when the speed goes out as 0 (a 0 command, or the coast region below 1000 cps), the **last steering is kept**. A car can't turn in place, and holding stops the wheels flicking straight at every pause. A turn asked for then is counted (`steering held`), with no WARN.
- **Not a number:** a NaN or infinite `linear.x` or `angular.z` centres the steering (`steering invalid`, WARN).
- **The held value resets to centre** when arming starts, on every disarm or ESTOP, and on a heartbeat stop. So each arm starts straight.
- **`steer_valid`** is set on every DRIVE, so the servo holds what it's sent instead of timing out.

**Every stop path centres the steering**, though not all in the same way:

| Path | Steering |
|---|---|
| ARMING | DRIVE 0 at 0 %, explicitly |
| Heartbeat 300 ms | DRIVE 0 at 0 %, explicitly |
| Heartbeat 1 s, `/rover/estop`, an arm timeout, shutdown | ESTOP ×3, then no DRIVE: the **firmware's 500 ms steering timeout** centres it |
| The board latching by itself, a board reset | no DRIVE: the same timeout |

An ESTOP latches only the motor (`link_rx.c`); the steering stays where it
was until that timeout. So for up to 500 ms a coasting truck can keep its
wheels turned. Sending DRIVEs after an ESTOP to centre it sooner was decided
against (2026-10-22): it would break "after shutdown, nothing but ESTOPs" and
"DISARMED sends nothing". Centring the steering in the firmware's ESTOP
handler is an open item (below).

**Assumption: percent → angle is linear.** Only full lock (±100 %) was
measured. The firmware's pulse is linear in percent (1200 / 1500 / 1800 µs),
but the linkage may not be, so a part-lock command may turn more or less
than the model says.

**Open items:**
- Centre the steering on ESTOP in the firmware, so the 500 ms above goes away.
- Measure a part-lock point (a turning circle at `s ±50`) to check the linear assumption.

## Teleop (group 4)

Drive the truck from the laptop's keyboard. The teleop node
(`rover_bridge/teleop_node.py`, rules in `teleop.py`) runs **on the Pi, inside
the bridge's container**, and publishes `/cmd_vel` there. The laptop runs
`tools/teleop_keys.py`, which sends only keystrokes and a keepalive byte every
100 ms over SSH, and shows the node's status line. So `/cmd_vel` never crosses
Wi-Fi.

    py tools\teleop_keys.py armando@192.168.1.18        # Windows: PowerShell or cmd, not Git Bash

It runs `ssh -T -e none -o IPQoS=ef -o ServerAliveInterval=2 -o ServerAliveCountMax=2 armando@192.168.1.18 "docker exec -i rover-bridge /ros_entrypoint.sh python3 -u -m rover_bridge.teleop_node"`.
The bridge must already be running (`docker compose ... up -d`).

| Key | Effect |
|---|---|
| W / S | Speed step up / down: 0 → 0.05 m/s (1271 cps, past the firmware's coast band below 0.039 m/s), then 0.025 steps. Forward up to **0.15 m/s** (3813 cps), reverse down to **−0.10 m/s**. |
| A / D | Steering 10 % of full lock left / right, up to ±100 % (±0.204 rad). **A is always wheels left**, reversing too. |
| C | Steering centre |
| Space | Speed 0 and centre |
| R | `/rover/arm` |
| X, Esc | Speed 0, centre, `/rover/estop`. On Linux, arrows and F-keys arrive as Esc sequences, so they are Esc too. |
| Q, Ctrl+C | Quit: one `/cmd_vel` of 0, `/rover/estop`, then nothing is published; the node exits |

**What it publishes**, at 20 Hz on a STEADY-clock timer, with the bridge's QoS:
`linear.x = v`, `angular.z = v·tan(δ)/L`, where δ = steer % / 100 × 0.204 and
L = 0.133 m (`config/vehicle.cfg`). The bridge computes atan(L·ω/v) and gets
back exactly δ. So the percent on the status line is the percent the servo
gets, at every speed; it isn't saturated at low speed. Going forward, A gives
`angular.z` > 0. Reversing, it gives `angular.z` < 0, which the bridge turns
back into wheels left. **At v = 0, `angular.z` is 0 and the bridge holds its last
steering** ([Steering](#steering)). A steering change while stopped (C and Space
included) reaches the wheels once the truck moves; the status line says
`applies when moving`.

**The deadman.** The teleop runs next to the bridge. If the laptop drops, `/cmd_vel`
doesn't stop, and the bridge's heartbeat never fires. The keyboard link is
what has to stop the truck. sshd notices a dead laptop late: with the default
`ClientAliveInterval 0`, only when its writes fail, which can take minutes. And a
`docker exec`'d process may outlive its client. So every byte from the laptop,
key or keepalive, feeds a deadman in the node:

| No byte for | Then |
|---|---|
| **0.5 s** | Speed 0 and centre, still publishing (still armed). **Latched:** keepalives coming back don't drive again; W does. |
| **2 s** | `/rover/estop`, nothing more published, the node exits |
| End of input (ssh or docker gone), SIGHUP / SIGTERM / SIGINT | The same at once |

The 10 min gap measurement of 2026-10-21 had a longest steady-state gap of
592 ms, 4 gaps of 500–999 ms and 26 of 300–499 ms. So **the 0.5 s stage will
trip now and then** (about 4 times in 10 min at that rate). That's safe: the
truck stops, stays armed, and W drives it again. Counted as `link stops`.

**Status line** (5 Hz): v (m/s, cps), steering (%, rad), ω, then
`link ok, last byte 34 ms, max 103 ms/10 s` (the age of the last byte and the
longest gap in the last 10 s, measured on the Pi). After that, the bridge's
state and what it sends (from `/diagnostics`), and the last service reply.
**Watch the last-byte age on the bench:** with keepalives every 100 ms it
should stay under ~150 ms, and the max shows every hiccup.

**Events** (link stop, link back, arm / estop replies, the final line) are
printed as `[Pi epoch s] text`, the clock of the bridge's log lines. They also go
to `~/rover-logs/teleop_<date>_<time>.log` (the container's `/logs`) on the Pi.

**SSH options, checked against the OpenSSH source** (the laptop has
OpenSSH_for_Windows 9.5p2; Debian 13 has 10.0):
- **No Nagle either way.** `ssh_packet_set_interactive` calls `set_nodelay`
  unconditionally (packet.c, 9.5 and 10.0), on both client and server. Only the
  DSCP mark depends on whether a tty was requested.
- **`-T` loses the interactive mark.** That is what `IPQoS=ef` is for. Without a
  tty the session is marked CS1 ("lower effort", the default bulk class in 9.5
  and 10.0; OpenSSH 10.2 changed the defaults to EF / CS0). Wi-Fi WMM maps DSCP
  by its top three bits: CS1 and the tty default AF21 both land in the
  **background** queue, so `-tt` wouldn't help. `IPQoS=ef` (one value, so it
  applies whatever the tty) puts the keys and keepalives in the video queue.
  The Pi → laptop direction (the status line) keeps sshd's CS1. Optional check
  on the Pi: `sudo tcpdump -i any -c 5 -v 'tcp port 22 and src host <laptop IP>'`
  should show `tos 0xb8`.
- **`-T`, not `-tt`:** stdin must be a clean byte pipe. A tty on the Pi would echo
  the keys and line-buffer them.
- **`-e none`:** no `~` escapes in the key stream. ServerAlive: ssh itself gives up
  after ~4–6 s. The node has stopped (0.5 s) and ESTOPped (2 s) well before.

**Password login works with stdin piped.** For password auth, Windows OpenSSH reads
the console directly (`readpassphrase`: `_getwch`), whatever stdin is, and
Linux OpenSSH reads `/dev/tty`. `teleop_keys.py` doesn't touch the keyboard
until the node prints `teleop: ready`, so the password (and a host-key question)
has the console to itself. If the prompt doesn't appear on the bench, set up a
key instead (PowerShell):

    ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_ed25519
    type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh armando@192.168.1.18 "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
    ssh armando@192.168.1.18 true      # must not ask for the password any more

**Updating the Pi:** no image rebuild. The Dockerfile has only ROS and pyserial,
and `compose.yaml` mounts the repo read-only at `/repo`:

    ssh armando@192.168.1.18
    cd ~/autonomous-rover && git pull --ff-only && git log --oneline -1
    docker compose -f ros_ws/docker/compose.yaml restart bridge     # ESTOP x3 on the way down, as always
    docker compose -f ros_ws/docker/compose.yaml logs --tail 20 bridge

The new bridge's `opened ...` line ends with `steering: wheelbase 0.133 m, full lock 0.204 rad`.
The teleop needs no restart: each `docker exec` is a new process on the mounted code.

### First run on the lift

LiPo in, truck on the lift, wheels free. Update the Pi (above), then:
1. `py tools\teleop_keys.py armando@192.168.1.18`: password, then `teleop: ready` and the status line: v 0, `link ok`, last byte under ~150 ms. On the Pi (second SSH session), `docker exec rover-bridge /ros_entrypoint.sh ros2 topic hz /cmd_vel` should read ~20 Hz.
2. **R:** `/rover/arm: ok`, and the status line shows `bridge ARMED`.
3. **W:** 0.05 m/s. The wheels turn **forward**; the bridge sends 1271 cps.
4. **The first ROS steering sign check:** **A A A** (30 %). The front wheels turn **left, seen from behind**; the bridge sends 30 %. **D** ×6 (−30 %): right. If A turns them right, stop (X): the sign is wrong somewhere between teleop, bridge and firmware.
5. **S S** (−0.05 m/s, reversing) with 30 % left. The wheels **still point left**, and the bridge sends 30 %.
6. **A** up to 100 %. The servo is quiet at full lock (`s_us` 1800). Then **C**.
7. **W** up to the top: 0.15 m/s, 3813 cps, and more W changes nothing. **S** down to −0.10 m/s, and no further.
8. **Space:** the wheel stops. The steering stays where it was (the hold), and `applies when moving` is shown if not centred.
9. **X:** `/rover/estop: ok`, LATCHED (ESTOP). The steering centres within 0.5 s (the firmware's timeout).
10. **R**, then **Q:** `/cmd_vel` 0, `ESTOP x3: requested (/rover/estop)` in the bridge log, the node exits, and `teleop_keys.py` says ssh exited.

### Wi-Fi drop test (on the lift, armed, 0.05 m/s, 30 %)

| Drop | Must happen | Proof (Pi clock: the teleop log and the bridge log) |
|---|---|---|
| a. Kill `ssh.exe` on the laptop (Task Manager, or close the window) | Within 0.5 s: speed 0, centre, the wheel stops. ESTOP at the latest 2 s after the last byte. | teleop: `no keyboard byte for 5xx ms: speed 0, centre`, then `keyboard link closed (end of input)` or `no keyboard byte for 20xx ms`, `/rover/estop: ok`. Bridge: `ESTOP x3: requested (/rover/estop)`, LATCHED (ESTOP). **No** `heartbeat` lines: the teleop acted, not the bridge. |
| b. Laptop Wi-Fi off | The same as a, without sshd ever noticing: the 0.5 s stop, then the 2 s ESTOP | The same lines, ~0.5 s and ~2 s after the teleop's last byte. Afterwards `docker exec rover-bridge pgrep -af teleop_node` is empty: no teleop left running. |
| c. `docker exec rover-bridge pkill -KILL -f teleop_node` (second session) | The bridge's own path: DRIVE 0 at 300 ms, ESTOP at 1 s | Bridge: `no /cmd_vel for 300 ms: DRIVE 0 (still armed)`, then `ESTOP x3: no /cmd_vel for 1 s (heartbeat)` |
| After each | No driving until R. The steering centres within 0.5 s of the ESTOP. | `s_us` 1500. Keys without R don't move the truck. |

## Low-battery shutdown (group 5)

The Pi runs from the pack (buck #1, header pins 4/6, since 2026-10-16). After
the firmware's cutoff, the motor is locked out but the Pi keeps draining the
pack. A host systemd service, `rover-battery-watch` (`tools/battery_watch.py`),
shuts the Pi down when the pack is flat at rest.

**A shutdown slows the drain; it doesn't stop it.** A halted Pi still draws
power, and buck #2 (the servo), the divider and the Nucleo (on the Pi's USB)
may keep drawing from the pack too. **Unplug the pack after a shutdown, and
never leave it plugged in unattended.**

**How it gets the battery:** ROS runs only in the container, so the bridge
writes `/run/rover/battery.json` once a second (`power_state.py`). That's a
tmpfs, empty at every boot, mounted into the container at the same path. The
file holds `boot_id`, `status_monotonic` (when the STATUS was received),
`battery_state`, `vbat_mv`, `motor_state` and the wall time. It's written to a
temporary file and renamed, so the watcher never reads half of one. Freshness
is checked on CLOCK_MONOTONIC, which the container and the host share, so an
NTP step can't make old data look fresh. The watcher polls the file at 1 Hz.

**The rule:**

| Sample (fresh: this boot, a newer STATUS, at most 5 s old) | Effect on the 30 s hold |
|---|---|
| Pack measured (OK, LOW, CUTOFF), motor not RUNNING, vbat < **6800 mV** (3.4 V/cell) | **counts** |
| The same, vbat ≥ 6800 mV | resets |
| No pack: NOT_PRESENT (**wall power**), REMOVED_WHILE_DRIVING | resets; never a shutdown |
| Motor RUNNING (sag under load isn't the resting voltage) | pauses |
| ADC_FAULT (the voltage can't be trusted) | pauses |
| Stale or missing data: the bridge not running (e.g. a motor_console session), the board unplugged, an earlier boot's file, garbled JSON | pauses: **never a shutdown on missing data** |

- **30 s of counting samples shut the Pi down,** once (latched). One sample adds at most 2 s.
- **Why 6.8 V, not the firmware's 6.6 V:** it leaves margin for what a halted Pi still drains.
- **Why at rest, not on the firmware's CUTOFF latch:** a cutoff can come from sag under load, and that pack rebounds at rest. A pack that is really flat rests below the threshold and keeps falling.
- **While driving it never acts,** so it needs no ESTOP of its own. A resting pack below 6.6 V makes the firmware cut the motor off within about 3 s, and then the hold runs. At the shutdown, docker stops the bridge, which sends `ESTOP x3` on its way out, as always.

**Warnings:**
- The watcher's state goes to `/run/rover/watch.json`. The bridge puts it in the `/diagnostics` battery status (`Pi shutdown watch`). While the watcher is counting down, that status is at least WARN and its message ends `; Pi SHUTDOWN in 18 s (shutdown)`. The **teleop status line** shows the same `bat …` text.
- The watcher's log, in the journal (`journalctl -u rover-battery-watch`) and in `/var/log/rover-battery-watch.log`, has every state change, the countdown every 10 s, and before the shutdown, a fsynced `SHUTDOWN: 6.70 V at rest for 30 s: shutting down (threshold 6800 mV, hold 30 s, action shutdown)`. Then `/usr/sbin/shutdown -h now`.

**What gets installed on the Pi** (from `ros_ws/pi/`; root unit, root-owned copy, no sudoers):

| File | What |
|---|---|
| `/usr/local/sbin/rover-battery-watch` | A copy of `tools/battery_watch.py` (stdlib, `#!/usr/bin/python3`). A copy, not the repo file, so nothing running as armando can change code that runs as root. |
| `/etc/systemd/system/rover-battery-watch.service` | `Restart=always`, `NoNewPrivileges`, `ProtectSystem=full`, `ProtectHome`, `PrivateTmp`, `EnvironmentFile=-/etc/default/rover-battery-watch` |
| `/etc/tmpfiles.d/rover.conf` | `d /run/rover 0755 armando armando -` |
| `/etc/default/rover-battery-watch` | Only for the bench (below): `THRESHOLD_MV`, `HOLD_S`, `ACTION=shutdown\|log` |

`ros_ws/docker/compose.yaml` mounts `/run/rover`, which needs a container recreate (`up -d`), not just a restart.

**Install** (on the Pi, from `~/autonomous-rover` after a `git pull`):

    python3 --version                    # the watcher needs the host's python3
    sudo install -m 0755 -o root -g root tools/battery_watch.py /usr/local/sbin/rover-battery-watch
    sudo install -m 0644 -o root -g root ros_ws/pi/rover-battery-watch.service /etc/systemd/system/
    sudo install -m 0644 -o root -g root ros_ws/pi/rover.tmpfiles.conf /etc/tmpfiles.d/rover.conf
    sudo systemd-tmpfiles --create /etc/tmpfiles.d/rover.conf
    docker compose -f ros_ws/docker/compose.yaml up -d
    sudo systemctl daemon-reload && sudo systemctl enable --now rover-battery-watch
    systemctl status rover-battery-watch --no-pager; ls -l /run/rover; tail -3 /var/log/rover-battery-watch.log

The bridge log then says `battery state for the shutdown watcher: /run/rover/battery.json`.
After a change to `tools/battery_watch.py`, repeat the first `install` line and `sudo systemctl restart rover-battery-watch`.

**Uninstall:**

    sudo systemctl disable --now rover-battery-watch
    sudo rm -f /etc/systemd/system/rover-battery-watch.service /usr/local/sbin/rover-battery-watch /etc/tmpfiles.d/rover.conf /etc/default/rover-battery-watch
    sudo systemctl daemon-reload
    sudo rm -rf /run/rover
    # then remove the /run/rover line from ros_ws/docker/compose.yaml and: docker compose -f ros_ws/docker/compose.yaml up -d

### Bench test (no draining to cutoff)

LiPo in, motor stopped, the bridge and the watcher running. Read the pack's
resting voltage on the teleop status line (`bat 7.85 V`) or in `/diagnostics`.

1. **Dry run:** `sudo cp ros_ws/pi/rover-battery-watch.bench /etc/default/rover-battery-watch`, set `THRESHOLD_MV` in it just above the resting voltage (e.g. 7900 for 7.85 V; `HOLD_S=10`, `ACTION=log`), then `sudo systemctl restart rover-battery-watch`. Expect, in `journalctl -u rover-battery-watch -f`: `low: 7.85 V at rest, below 7.90 V: Pi shutdown in 10 s`. The teleop status line shows `bat 7.85 V; Pi SHUTDOWN in … s (log)`. Then `SHUTDOWN: …` and `would shut down now (ACTION=log): nothing done`. The Pi stays up.
2. **Real:** set `ACTION=shutdown` in the same file, then `sudo systemctl restart rover-battery-watch`. The Pi powers off about 10 s later. **Unplug the pack.** Power the Pi back up, then check: `docker compose -f ros_ws/docker/compose.yaml logs bridge | grep 'shutdown: ESTOP'` shows `shutdown: ESTOP x3 sent` from the shutdown, and `grep SHUTDOWN /var/log/rover-battery-watch.log` still has the line. Then **remove the overrides**: `sudo rm /etc/default/rover-battery-watch && sudo systemctl restart rover-battery-watch`. The log should say `started: threshold 6800 mV at rest, hold 30 s, action shutdown`.

Optional, with the same overrides (`ACTION=log`):
- **Wall power** (no pack), `THRESHOLD_MV=9000`: `no pack (wall power)` and nothing more for 2 min.
- **Drive on the lift during the hold:** `running: … under load (not counted)`. Stop: the countdown goes on from where it was.
- **`docker compose stop` during the hold:** `stale`, and the count pauses. `start`: it goes on.
- **With a meter:** the pack current before and after a real shutdown, to measure what a halted Pi (and the rest) still draws.

**Open items:**
- **motor_console writes the state file too.** While a motor_console session holds the port, the bridge isn't running, and the watcher has no data (it never shuts down then). motor_console already parses `vbat`/`bst`; on the Pi it could write the same `/run/rover/battery.json`.
- A GPIO "shut down" line from the Nucleo would work without any of this software (firmware README, Open items).

## EKF and pose (group 6)

`ros_ws/src/rover_ekf` (C++) runs the core's EKF as a ROS 2 node. All the
logic is in the ROS-free `rover::OdometryFusion`
(`include/rover/odometry_fusion.hpp`, tested with the core in CI). The node
only converts messages.

| | Topic | What |
|---|---|---|
| in | `/rover/imu/raw` | `angular_velocity.z`, the yaw rate (IMU X forward, Y left, Z up). Each message is one filter cycle: predict, gyro update, then the encoder and steering updates. |
| in | `/rover/joint_states` | `rear_left_wheel_joint` velocity, converted to m/s with the cfg's `meters_per_encoder_tick`. Used as the rear-axle centre's speed: the rear axle is locked (`rear_axle_locked = true` in the cfg), so both wheels turn together and the encoder wheel reads the centre's speed. With `false` (an open differential) it would be corrected to v = v_left + ω·rear_track/2. No velocity means the estimate isn't valid, so no update. |
| in | `/rover/steering` | New in the bridge, 10 Hz from STATUS: joint `steering`, the angle the **servo has** (ACTIVE: % / 100 × 0.204 rad; TIMED_OUT and OFF: 0; FAULT: not published). |
| out | `/odom`, TF `odom` → `base_link` | 50 Hz. `base_link` is the rear-axle centre, the filter's point. Pose and twist covariance come from the filter. |
| srv | `/rover/ekf/reset` | `std_srvs/Trigger`: the pose back to (0, 0, 0). Speed, yaw rate and the bias are kept. |

- **Start:** nothing is published until 2 s of gyro samples at rest (encoder valid, |v| < 5 mm/s) have seeded the gyro bias. **Keep the truck still after the EKF starts.** The log says `started: gyro bias … rad/s`.
- **Stale inputs:** no IMU for 200 ms means no cycles and no `/odom` (WARN). A gyro gap over 50 ms isn't predicted across: the pose holds and the gap is counted. An encoder sample older than 30 ms isn't used, and each one is used once. Steering older than 0.3 s isn't used, nor while the encoder reads at rest (|v| < 5 mm/s), so a hand turn keeps its heading ([kalman-filter.md](kalman-filter.md)). Every 10 s the log has the pose, speed, bias and the counts.
- **Noise:**
  - **R_gyro = 1e-6 (rad/s)²** and **q_gyro_bias = 1e-9 (rad/s)²/s.** The static 1 h log gives about 9e-8 at this 100 Hz feed and 1e-11 ([kalman-filter.md](kalman-filter.md)). Both are raised here for vibration and warm-up.
  - **The steering model's noise comes from the 2026-10-22 full-lock spread:** six runs of 0.212, 0.197, 0.202, 0.216, 0.185, 0.221 rad, a sample standard deviation σ_δ = 0.0134 rad. With k = tan δ / L, the relative error at full lock is σ_δ / (sin δ cos δ) = **0.0675** (`steering_sigma_relative`). Near straight ahead, the yaw-rate error is \|v\|·σ_δ/L; at the teleop's 0.15 m/s that is **0.0151 rad/s** (`steering_sigma_min`). This assumes the full-lock spread holds at every angle.
  - The encoder's noise is still the core's placeholder (4e-4 (m/s)²).
- **Where:** on the Pi, as the `ekf` service in `compose.yaml`, with the same image, host network and IPC. The 100 Hz inputs stay off Wi-Fi (gaps up to ~600 ms there); RViz only needs `/odom` and TF. The repo is mounted read-only, so the service builds the package with colcon into the `ekf-ws` volume when it starts (`ros_ws/docker/ekf_start.sh`): minutes the first time, seconds after. The image gained the C++ build tools, so it needs **one** `up -d --build`.

### Bench (group 6)

On the Pi, once, after a `git pull`:

    cd ~/autonomous-rover && git pull --ff-only
    docker compose -f ros_ws/docker/compose.yaml up -d --build
    docker compose -f ros_ws/docker/compose.yaml logs -f ekf      # wait for "started: gyro bias"

On the laptop (WSL), RViz:

    source /opt/ros/jazzy/setup.bash
    colcon --log-base ~/desc_ws/log build --symlink-install --base-paths /mnt/c/DEV/autonomous-rover/ros_ws/src/rover_description --build-base ~/desc_ws/build --install-base ~/desc_ws/install
    source ~/desc_ws/install/setup.bash; source ~/rover_net.sh; ros2 daemon start
    ros2 launch rover_description pose.launch.py

Truck on the floor, LiPo in, the teleop running (`py tools\teleop_keys.py armando@192.168.1.18`). The pose (Pi, second SSH session):
`docker exec rover-ekf /ros_entrypoint.sh ros2 topic echo --once /odom --field pose.pose`
(heading = 2·atan2(orientation.z, orientation.w)). Before each step:
`docker exec rover-ekf /ros_entrypoint.sh ros2 service call /rover/ekf/reset std_srvs/srv/Trigger`.

1. **Yaw sign:** reset, then turn the truck by hand **90° CCW** along a taped right angle. Expect heading **+1.57 ± 0.05 rad**; in RViz, the arrow turns left.
2. **Straight (also the encoder scale check):** mark the rear-axle centre on a 2 m tape line, then reset. Teleop: **R**, **W W W** (0.10 m/s), drive about 2 m, **Space**. Tape the start mark to the rear-axle centre, then compare it with √(x² + y²) from `/odom`. **Pass: within 6 cm (3 %).** If the distance is off by more than 3 %, the new `meters_per_encoder_tick` = 3.9336e-5 × tape / EKF. Put it in `config/vehicle.cfg`, commit, `git pull` on the Pi, `docker compose -f ros_ws/docker/compose.yaml restart bridge ekf`, and rerun once.
3. **Turn:** mark the rear-axle centre, then reset. Teleop: **R**, **W** (0.05 m/s), then **A** ×10 (full lock left), drive until the truck faces back (a half circle, R ≈ 0.64 m), **Space**. Tape the end mark relative to the start (expect about x 0, y +1.29 m). **Pass: heading π ± 0.1 rad, position within 10 cm.** If the straight passes and the turn misses, record the numbers: a tuning note for Week 6, not a blocker.

## Topics

| Topic | Type | Content |
|---|---|---|
| `/rover/imu/raw` | `sensor_msgs/Imu` | 100 Hz. Each 10 ms period's mean gyro (rad/s) and accel (m/s²), in the IMU chip's axes. `frame_id imu_link`. No orientation: `orientation_covariance[0] = -1`. |
| `/rover/joint_states` | `sensor_msgs/JointState` | 100 Hz. `rear_left_wheel_joint` (the encoder wheel). Position in rad is the cumulative odometer, so it survives lost frames. Velocity in rad/s is sent only while the speed estimate is valid; otherwise it's left empty. |
| `/rover/battery` | `sensor_msgs/BatteryState` | 1 Hz, reliable + transient local, so a late subscriber gets the last one. **No pack** (wall power): `present: false`, `voltage: NaN`, status and health UNKNOWN; never 0 V "dead". LOW (below 7.0 V) is health GOOD here, with a WARN in `/diagnostics`. CUTOFF is DEAD; ADC fault is UNSPEC_FAILURE. Charge, current and percentage aren't measured (NaN). |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz, five statuses (`hardware_id nucleo`), described below. |

- **QoS:** IMU and joints use sensor-data QoS (best effort, keep last 5).
- **Stamps** (IMU, joints, battery): when the board *made* the frame, on the Pi's clock, from the board-clock fit below. Never later than the frame's own receive time, and never going backwards.
- **Not yet measured:** where the IMU sits on the chassis, and the calibration.

## Board clock fit (`clock_fit.py`)

The Nucleo's clock runs a few thousand ppm fast. Each frame reaches the Pi
after a delay that can only add (USB, the ST-LINK bridge, the reader). So:
- **The model:** for each STATE, y = receive time − board time. In every 1 s of board time, the earliest arrival is kept (only complete seconds with at least 20 frames count). A least-squares line through those minima covers the last 20 s (`clock_window`).
- **Late seconds are rejected:** a second whose minimum is more than 2 ms above the line is dropped, and the line is fitted again. During a reader stall, every frame of those seconds is late.
- **Board resets:** a reset (t_ms going backwards, or a boot line) restarts the fit. A u32 wrap is unwrapped.
- **Start-up:** for the first ~3 s, before the line exists, the stamp uses the earliest offset alone.

Reported in `/diagnostics` (clock):
- drift in ppm (+ = the board runs fast);
- offset (Pi − board);
- residual (RMS of the minima around the line);
- latency (a typical frame's delay above the line);
- the window in seconds;
- rejected seconds, held-back stamps, restarts.

Synthetic tests (2000–4000 ppm, jitter, 20–100 ms bursts, resets, a wrap, a drift change, a 2 s stall) give drift within ±100 ppm and stamps within 2 ms.

**Host clock steps.** The fit also gets each read's monotonic time. A change
of more than 50 ms in wall − monotonic is a step of the system clock (NTP, or
WSL's clock being corrected): the fit starts over, and the stamps follow the
system clock, which is the one case they may go back. NTP's slewing is far
below 50 ms and isn't a step. Steps are counted in `/diagnostics`. Seen
2026-10-17: **WSL's clock stepped back 0.85–1.0 s about every 32 s** (it runs
roughly 3 % fast). Anything on WSL that compares Pi stamps with its own clock
(RViz, TF) will see that until WSL's time is fixed.

**WSL's clock steps, measured** (2026-10-19, `tools/clock_step_probe.py`, 180 s,
no ROS; WSL 2.7.14.0, kernel 6.18.33.2, clocksource `tsc`):
- 6 steps, **all backward, 674–861 ms (median 763), every 32.2–32.4 s**: the
  clock gains ~2.6 % between corrections.
- The stepper is systemd-timesyncd: its poll interval is 32 s (its minimum),
  and `timedatectl timesync-status` showed offset −838.713 ms, the size of the
  last step the probe saw. An offset over 0.4 s is stepped, not slewed, and a
  2.6 % error is far beyond what its frequency correction can absorb, so it
  steps on every poll.
- **It stops the truck.** `ros2 topic pub -r` times its messages on the ROS
  clock, which is the wall clock (ros2topic `verb/pub.py`: `create_timer`
  without `clock=`; rclpy's `Node._clock` is a `ROSClock`). After a step back
  of S, its next message comes S late: a /cmd_vel gap of S + up to 50 ms,
  past the 300 ms stop (the wheel pauses, as on the 2026-10-19 bench). A step
  over ~0.95 s would pass the 1 s heartbeat ESTOP.
- **So, on WSL, don't drive with `ros2 topic pub`.** Use
  `tools/clock_step_probe.py --publish` (a STEADY-clock timer), or run the
  publisher on the Pi. Group 4's teleop node times on a steady clock.
- **It got worse during the day:** 948-1454 ms steps at ~16:00 (the second
  bench), so every step passed the 1 s ESTOP; 1053-1172 ms at 16:29.
- **WSL's monotonic clock runs fast too** (same clocksource): from the first
  to the last step, monotonic advanced 3.3-3.7 % more than the corrected wall
  clock. So the probe's own step and gap sizes read that much long; the Pi's
  numbers are unaffected. The probe prints this rate ("monotonic vs the
  corrected wall clock").
- **The probe's `--publish` also listens, to its own messages.** Its gap line
  is local delivery on this machine (timer, rclpy, DDS here), not the network
  to the Pi; it prints how many messages arrived. Run `--listen` on the Pi too
  to see what arrives there.

**Fixing WSL's clock** (each step reversible, a 180 s probe and
`timedatectl timesync-status` before and after; nothing persistent yet):

| Fix | Steps in 180 s | Sizes | Monotonic vs corrected wall | timesyncd offset after |
|---|---|---|---|---|
| Before (16:29, up 2 h 53 min) | 5, all back, every 32.2-32.4 s | 1053-1172 ms | +3.66 % | -1121.7 ms |
| 1. `wsl --shutdown`, restart (16:32) | 3, all back, 64.5 then 32.3 s apart | 642-680 ms | +1.38 % | -641.5 ms |
| Before fix 2 (~45 min after a restart at 17:03) | 6, all back, every ~32.3 s | 490-1090 ms (median 958) | +2.89 % | -981.0 ms |
| 2. clocksource `hyperv_clocksource_tsc_page` (runtime; reverted to `tsc`) | 5, all back | 1001-1078 ms | +3.34 % | -1001 ms |

Neither helped for long: fix 1's improvement was gone within an hour, and the
clocksource made no difference.

**The cause: two clocks fighting, and Windows' clock was the one off.**
- WSL's hardware clock is fine: `CLOCK_MONOTONIC_RAW` against NTP -0.008 %
  and -0.004 % (two 3 min runs, SNTP to time.windows.com).
- Something in the VM (most likely the Hyper-V time sync, `hv_utils`, loaded
  as "TimeSync IC version 4.0") pulls WSL's clock toward Windows' time by
  rewriting the kernel's `adjtimex` tick about every 8 s: 10174-10771 us
  against 10000, i.e. 1.7-7.7 % fast, with a random +-50 ppm `freq`. That is
  the "3 % fast" of both the wall and the monotonic clock (monotonic vs raw
  +3.3 to +3.9 %). timesyncd never touches `tick`; it steps.
- Windows itself ran at the true rate but +0.80 to +0.84 s ahead of NTP. So
  the Hyper-V sync pulled WSL toward Windows (overshooting it by up to 0.3 s),
  timesyncd stepped it back to NTP on its next 32 s poll: the sawtooth.
- Why Windows was off: it polled NTP only every 9.1 h (`SpecialPollInterval`
  32768 s), and on every resume from sleep it loads its time from the RTC
  (event "synchronized with the hardware clock", always at .5 s, plus the
  RTC's drift; 13 resumes in 4 days). On 2026-10-19 the resumes at 11:37,
  11:57 and 13:32 were not followed by an NTP sync until 19:35: the second
  bench (15:47-16:12) ran on RTC time. On 2026-10-21 a resume after 39.6 h was corrected by
  -4359 ms 49 s later, only because the 9.1 h timer had run out meanwhile.
- With Windows synced (2026-10-21, +0.05 to +0.09 s): 0 steps in 180 s,
  timesyncd offset +94.7 ms (under its 0.4 s step limit). The fight stops.

**Windows time fix** (2026-10-21, admin; each step undoable):
- `SpecialPollInterval` 900 s (`reg add
  HKLM\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpClient /v
  SpecialPollInterval /t REG_DWORD /d 900 /f`; undo: `/d 32768`). Effective
  1024 s: `MinPollInterval` 10 (2^10 s) clamps it.
- A second peer: `w32tm /config /manualpeerlist:"time.windows.com,0x9
  time.cloudflare.com,0x9" /syncfromflags:manual /update`, then `w32tm /resync
  /rediscover` (undo: the list with `time.windows.com,0x9` alone).
- A scheduled task `Rover\ResyncTimeOnResume`: `w32tm /resync` 60 s after
  every resume (Power-Troubleshooter event 1, as SYSTEM; undo: `schtasks
  /delete /tn "Rover\ResyncTimeOnResume" /f`).
- **Pending checks:** after a sleep of 10 min or more, an NTP sync (Time-Service
  event 37) within ~2 min of the resume and the stripchart within +-0.1 s;
  then a 60 min `clock_step_probe` in WSL with **0 steps**. Nothing on the WSL
  side changed: with Windows synced, timesyncd and the Hyper-V sync agree.

**The bridge's own timers run on a steady clock** (publish_pending at 100 Hz,
`/diagnostics`, the 10 s summary). On rclpy's default clock (the wall clock)
a step back stalls them for as long as the step: publish_pending is what
hands STATUS to the safety core, so a step can time out an arm (the board's
"motor OFF" arrives too late; most likely what failed one WSL test run on
2026-10-19). The heartbeat and the TX thread were
never affected: they use monotonic time. **On the Pi this matters at boot:
the Orange Pi has no RTC**, so after a power-off it boots with a stale clock,
and the container (`restart: unless-stopped`) can already be running when NTP
steps the clock to the right time. A test freezes the node's clock and steps
it back 5 s: the timers must keep firing.

**Thermal drift.** On the bench (2026-10-17, dd load) the drift climbed from
+2800 to +4469 ppm over about 8 minutes, roughly 3.3 ppm/s. A straight line
over 60 s is then about r·W²/12 ≈ 1 ms off at the newest point, which matches
the 1.17 ms residual seen. **So the window is 20 s**, which is about 0.1 ms
for that ramp. A synthetic test of the ramp measured 0.49 ms median error at
20 s against 1.34 ms at 60 s; about 0.4 ms of it is the same for any window,
because even the earliest arrivals sit above the latency floor. A cool start
read +1913 to +1978 ppm, against +4469 after the dd heat, so the drift is
most likely temperature: watch it in `/diagnostics`.

**Speed estimate (deferred).** `speed cps (estimate)` is a single 20 ms
estimate, read once a second. Single 20 ms readings run up to ~17 units
(850 cps) off from the encoder's remaining ripple (firmware README), so one
sample says little: on the second 2026-10-19 bench, at DRIVE 2542 (51 units),
the median of STATE `speed_cps` was 2550 in both drives, the 1 s means
2516-2604, and 18 % of single samples were at or below 2350 (the 2350 of the
first bench was one of those). A 1 s mean next to it is planned but not
done; until then judge the speed from a raw log (median of STATE
`speed_cps`).

**Board clock and speed.** The board's clock (MSI, not locked to the
crystal; firmware README) ran +6,000-6,600 ppm fast on that bench, so the
board's own speed estimate reads ~0.6 % low and the wheel actually turns
~0.6 % faster than commanded (2550 board cps is ~2565 real). It touches only
that: odometry distance is counts x metres per count, and the Pi's
velocities and stamps come through the clock fit. Locking the MSI to the LSE
(a firmware item) is hygiene, not a blocker.

## Group 3 bench (2026-10-19)

LiPo, truck on the lift, the firmware of 2026-10-16 (text-command gate), the bridge of 2026-10-17 (group 3). All passed. The
container log was lost (`docker compose down`); the key lines:
- **Start-up:** DISARMED, steering centred, HELLO_ACK normal build, LATCHED (BOOT), vbat 8005 mV, 0 bad / 0 gaps, CPU ~50 %.
- **Drive:** `/rover/arm` succeeded → ARMED, motor RUNNING. `/diagnostics`: setpoint 2542 (the echo, see above), speed estimate 2350 (one sample ~3 s after arming), heartbeat age 23 ms.
- **Heartbeat:** Ctrl+C on `ros2 topic pub -r 20` (WSL) → `ESTOP x3: no /cmd_vel for 1 s (heartbeat)` → LATCHED (ESTOP).
- **`compose stop` while driving:** SIGTERM → `shutdown: ESTOP x3 sent` 65 ms later → `final:` → stopped in 0.9 s; the wheel stopped at once. After a restart: still LATCHED (ESTOP).
- **`docker kill -s KILL` while driving:** LATCHED (WATCHDOG).
- **Clock fit:** +3,067 → +4,951 ppm while warming, residual 0.27–0.80 ms. CPU 57–58 % while driving.
- **Open, then:**
  - Mid-drive pauses: one logged `no /cmd_vel for 300 ms` per drive (armed 274.69, stop 279.50; armed 616.18, stop 629.20) while `ros2 topic pub -r 20` ran without a break, and the wheel was seen to pause and resume. Cause: WSL's clock steps (above).
  - Only one stop line per drive: a second stop with the same text wasn't logged (fixed: every event is logged, and the stops are counted).

## Group 3 bench, second run (2026-10-19, 15:47-16:12)

LiPo, truck on the lift, the bridge of 2026-10-19, raw log on
(`bridge_20261019_204700.log`), container log kept (`tee`). Times are Pi
epoch seconds 17924430xx / 17924440xx.

| Test | Result |
|---|---|
| 1. WSL `ros2 topic pub -r 20` | Armed 095.97, stop 100.85, **ESTOP x3 (heartbeat) at 101.55**, gap 1006 ms. Probe: 5 back steps of 957/995/1454/971/992 ms, ~32.3 s apart (one 47.5 s); 5 gaps = step + 49-52 ms, 5/5 with a step inside, 0 without. **Hypothesis confirmed**, and every step now passes the ESTOP |
| 2. Control A: `ros2 topic pub` in the Pi container | Armed 387.42-621.94: **0 stops**, gap max 61 ms |
| 3. Control B: probe `--publish` (steady clock) from WSL | Armed 744.28-879.15: 7 WSL steps (948-1127 ms), **no ESTOP**, no ~1 s gap; but gap max 251 ms (between the 800.99 and 810.99 lines) and two stops at ~847.6 (433 ms, then 341 ms in the same second). Open: see below |
| 4. Pauses, 0.5 s every 10 s | 6 pauses: 6 WARN + 6 `back after 529-538 ms`; since arm 6, gap max 538. The `(+1 more in the previous 220.6 s)` was Control B's held-back 341 ms stop (now said within a second) |
| 5. Setpoints | Echo 2542, units 2550, command 2542, sent 2542 |
| 6. Speed (raw log, from 1 s after the first STATUS with 2542) | A: median 2550, 1 s means 2516-2604, 18.5 % at or below 2350, duty median 322. B: median 2550, means 2500-2600 except its two stop seconds, 18.0 % at or below 2350, duty 311 |

- **Duty 4-7 % above the ~300 expected** (294-295 at 2500 cps and 7.86 V,
  firmware README; ~300 at 2550). The pack was lower: duty x vbat is the
  motor voltage, so 7.67 V (A) and 7.62 V (B) predict 307 and 309. That is
  about half of it. The rest falls during each drive (A: 324 to 319, B:
  324 to 298 from its first to its last third), likely a cold drivetrain
  loosening up; not proven.
- **Startup seq gap** (`seq_gaps=1 t_ms_gaps=1` from the first 10 s, never
  growing): STATE seq 3, 0.02 s after HELLO_ACK, never sent by the board. It
  had booted 160 s earlier and was in text mode; the HELLO came while a text
  status line was still going out, and the 4-slot TX queue (HELLO_ACK +
  STATE 0-2) refused STATE 3 (the firmware's `tx_dropped` 1 from its first
  STATUS; 0 bad pieces). Binary mode lasts until a board reset, so only a
  bridge start against a freshly booted board hits it. The 10 s line now
  shows `(board tx_dropped=N)`, and `/diagnostics` says when a new seq gap
  was the board's.
- **Clock fit:** +4,640 to +6,624 ppm (a new high), swinging +-300 ppm within
  a minute, residual up to 0.94 ms (the WARN is at 2 ms). No bound in the fit
  itself; `/diagnostics` warns outside -1,000..+10,000 ppm.
- **CPU:** ~59 % disarmed with the raw log (~50 % without, in an earlier
  session), 65-70 % driving. Not yet compared in one session; a cheaper
  writer waits for that A/B.
- **Open: Control B's gaps.** Not the WSL clock: the 433/341 ms pair is 14 s
  from the nearest step (the 251 ms window has one, a ~30 % coincidence at 4
  steps in 13 windows). Not the WSL publisher: its local listener saw no gap
  over 150 ms. Not a stalled bridge: the raw-log reader's largest gaps there
  were 14.6-17.9 ms, and Control A (no Wi-Fi) had 61 ms max. Left: the WSL
  network, Wi-Fi, the Pi's Wi-Fi (power save?), DDS on the Pi. Next: 10 min
  disarmed with the probe `--publish` on WSL, `--listen` in the Pi container,
  and pings both ways at 20 Hz; then with the Pi's Wi-Fi power save off. No
  threshold change before that distribution.

## Shutdown

rclpy's own signal handlers are off (`SignalHandlerOptions.NO`). On SIGTERM
they shut the context down from inside `spin_once` while a timer was
publishing, giving an `RCLError` traceback on `docker compose stop` on
2026-10-17. Now SIGINT, SIGTERM and SIGHUP only set a flag. The loop then:
1. stops the timers;
2. sends ESTOP ×3 (through the TX thread, while the link and ROS are still up) and logs `shutdown: ESTOP x3 sent`;
3. logs the `final:` line;
4. closes the link and the raw log;
5. destroys the node and shuts rclpy down;
6. exits 0.

The handlers are installed before `rclpy.init`, so `NO` is what keeps them;
a test checks this. A stop signal also ends the wait for a lost board.

## `/diagnostics`

| Status | Content | Levels |
|---|---|---|
| `rover_bridge: motor` | State, latch and fault reason by name; `setpoint cps (board echo of last DRIVE)` (STATUS, as received: 2542) and `setpoint cps (board, 50-cps units)` (what the controller runs on: 2550); `speed cps (estimate)` (the latest STATE: one 20 ms estimate, see below); IMU ok, build flags, reset flags (decoded) | ERROR: FAULT, a test build, no IMU. STALE: no STATUS for 1 s |
| `rover_bridge: battery` | Firmware battery state, vbat, VDDA; `Pi shutdown watch` (the host watcher's state, or `not running`) | OK: no pack, or OK. WARN: LOW, or the watcher counting down (the message ends `; Pi SHUTDOWN in 18 s (shutdown)`). ERROR: CUTOFF, ADC fault, removed while driving |
| `rover_bridge: link` | Frames, bad, seq and t_ms gaps, stale before HELLO_ACK, queue drops, HELLOs sent, read gap max (last second and session), STATE age; the firmware's `rx_ok`, `rx_crc_errors`, `rx_frame_errors`, `rx_rejected`, `rx_seq_gaps`, `rx_duplicates`, `rx_uart_errors`, `rx_ring_drops`, `rx_text_ignored`, `tx_dropped`, `overruns`, `resyncs`, `watchdog_trips`, `latches` | WARN: new bad pieces, gaps or drops since the last report (a new seq gap matched by the board's `tx_dropped` says `dropped by the board`), or a read gap over 0.5 s. STALE: no STATE for 1 s |
| `rover_bridge: clock` | The fit (above) | WARN: still fitting, drift outside −1000..+10000 ppm, or residual over 2 ms |
| `rover_bridge: operator` | State; heartbeat age; `command cps (from /cmd_vel)` and `drive cps (sent now)` (0 during a heartbeat stop, n/a when no DRIVE goes out); `command steer % (from /cmd_vel)` and `drive steer % (sent now)`; the counts (below minimum, clamped, invalid, steering clamped / held / invalid, ESTOPs, heartbeat stops and ESTOPs, arms, refusals); `heartbeat stops since arm`, `cmd_vel gap max ms (since arm)`, `cmd_vel last stop gap ms (since arm)`, `cmd_vel gaps >= 150 ms (since arm)`; the last event and `last event age s` | WARN: shutting down; armed with no `/cmd_vel` for 300 ms; commands newly below minimum, clamped or invalid, or steering newly clamped or invalid (not held). OK: the message is the state, the last event and its age |

## Raw link log (optional)

`raw_log` parameter: `disabled` (the default) or a directory. With a
directory, the bridge writes `bridge_<date>_<time>.log` plus `.times.csv`,
the same format `serial_logger --binary` writes, so `tools/link_dump.py` reads
it.
- It's written by a writer thread (`serial_logger.QueuedWriter`): the reader never waits for the disk.
- A file that can't be opened is logged as an error, and the bridge runs without it.
- In compose: `ROVER_RAW_LOG=/logs/ docker compose -f ros_ws/docker/compose.yaml up -d` writes to `~/rover-logs`, at about 21 MB/hour.
- `ROS` parses `-p` values as YAML, so the off value is `disabled`: an empty value doesn't parse, and `off` would be a boolean.

**Log:** every 10 s, one line with:
- the motor state from STATUS, e.g. `LATCHED (HELLO)`, or `LATCHED (BOOT)` on a board that hasn't been armed since boot (HELLO doesn't overwrite an existing latch);
- vbat;
- the safety state, the command (`cmd=`) and the DRIVE being sent (`sending=`), DRIVEs and ESTOPs sent, heartbeat stops (since arm, gap max, last stop, gaps of 150 ms or more), the board's `tx_dropped` after the link counts, and the last event with its age (`last event 12.3 s ago: ...`);
- messages published, plus frame, bad, gap and stale counts;
- the longest gap between two reads;
- the clock fit's drift and residual;
- the node's CPU use.

## How it's built

- **Two threads:**
  - The reader thread only reads, decodes and queues. It never touches a file, prints, or calls ROS, so a slow SD card or a busy CPU can't hold it up. (A 2.5 s stall in `serial_logger`'s flush lost 1.6 s of frames in `b5_session_vbat_1`.)
  - A second thread sends the HELLOs.
- **Stale data:** it reuses `serial_logger.BinaryLogger` with its stale rule, so nothing before this session's HELLO_ACK is published.
- **Exclusive port:** the port is opened with `exclusive=True`, so a second bridge or a `motor_console` can't share it.
- **Port loss:** the node waits until the Nucleo is back (it watches for its USB serial number in sysfs, for up to 120 s), then exits with code 1. `restart: unless-stopped` starts it again, and the restart sees the new device.
- **Code updates:** the code isn't baked into the image. Compose mounts the checkout read-only, so `git pull` plus a restart updates it.

## Setup on the Pi (once)

udev symlink `/dev/rover-nucleo` (matches the ST-LINK's USB serial number):

    sudo cp ros_ws/docker/99-rover-nucleo.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=tty --action=add
    ls -l /dev/rover-nucleo          # -> ttyACM0

Docker Compose v2 (Debian 13 `docker-compose` 2.26 provides the `docker compose` plugin):

    sudo apt install docker-compose
    docker compose version

`~/rover.env` (from group 0) holds `ROS_DOMAIN_ID=42`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ROS_STATIC_PEERS=<laptop IP>` and
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

## Run

From the repo root on the Pi:

    docker compose -f ros_ws/docker/compose.yaml up -d --build    # build (first time) and start
    docker compose -f ros_ws/docker/compose.yaml logs -f          # follow the log
    docker compose -f ros_ws/docker/compose.yaml down             # stop (no restart until up again)

On the laptop (WSL, `~/rover_net.sh` sourced): `ros2 topic hz /rover/imu/raw`.

**Same-host discovery on WSL needs a third participant.** Two ROS processes on
the laptop whose DDS participants start seconds apart never discover each
other: a subscriber first, `ros2 topic pub` 3 s later, 0 of ~180 messages in
4 of 4 runs (2026-10-21, Pi off, the `rover_net.sh` environment). With any
third participant started before both (the ros2 CLI daemon, or an idle rclpy
node) it worked 4 of 4 and 4 of 4. The env settings change nothing (SUBNET or
LOCALHOST, static peers or not, a discovery server), plain UDP works on every
path, and the cause inside Fast DDS is not known. On the 2026-10-19 bench it
worked because the daemon was running (from `ros2 service call`), or the Pi
was. So: **`ros2 daemon start` before using ROS tools on the laptop with the Pi
off.** (Whether the Pi alone is enough is untested: next bench, `ros2 daemon
stop`, then the same pair.) If it bites in the Week 6 simulation, Cyclone DDS
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) is a possible fallback; not planned
work.

For a second container on the Pi that reads the bridge's topics, use
`--network host --ipc host`. Without the shared IPC namespace, Fast DDS's
shared-memory transport fails silently.

Outside Docker (WSL, against a fake board or a USB pass-through port):

    PYTHONPATH=ros_ws/src/rover_bridge:tools python3 -m rover_bridge.node --ros-args -p port:=/dev/ttyACM0

With a colcon install (`ros2 run rover_bridge bridge`), set `ROVER_TOOLS_DIR` to the repo's `tools/`.

## Tests

`python3 -m pytest -q ros_ws/src/rover_bridge/test`:
- The conversions, the link, the sysfs lookup, the clock fit, battery and diagnostics are ROS-free, and run in CI (job `python-tests`).
- The node tests run against a threaded fake board in real time. They need rclpy, so they skip without ROS sourced.
- **In CI, job `ros-tests`** (since 2026-10-25): a `ros:jazzy` container builds rover_bridge, rover_description and rover_ekf with colcon (rover_gazebo skipped), and `colcon test` runs the bridge's whole test folder, node tests included, and rover_ekf's node test (`test_ekf_node.py`, the built `ekf_node` against fake inputs). `colcon test-result --verbose` fails the job on any failure; `colcon test` alone would not. Domain 87, localhost-only discovery. The first runs: rover_bridge 174 tests, rover_ekf 1, 0 skipped (the first CI run, 2026-10-25).
- **Failure control** (2026-10-25, before the job was committed): with one of `test_ekf_node.py`'s assertions broken in a copy of the tree, `colcon test-result --verbose` reported the failure and exited 1.
- The node tests isolate themselves: localhost-only discovery on a domain of their own. With `~/rover_net.sh` sourced they would otherwise also receive a running Pi bridge's topics, and that happened on 2026-10-16.
- They start a **discovery anchor** first (an idle rclpy node in a process of its own, for the whole module; see same-host discovery above), and `fastdds shm clean`. Without the anchor, a test whose operator starts after a bridge subprocess fails with "same-host DDS discovery failed (no anchor participant?)": 10 of 10 isolated runs on 2026-10-21, against 0 of 10 with it.
- The steering tests (`test_bridge_safety.py`): sign, reverse, rounding, the clamp, the hold at v ≈ 0, NaN / inf, the resets, and every stop path (no DRIVE after an ESTOP or a disarm). A node test checks that `angular.z` reaches the fake board as `steer_pct` with the cfg's values. It also checks that after `/rover/estop` the fake board's 500 ms steering timeout centres it, with no DRIVE.
- The teleop tests: `test_bridge_teleop.py` (ROS-free, CI: speed and steering steps, the clamps, ω = v·tan(δ)/L giving back every percent through the bridge's mapping, A = left forward and reversing, space / x / r / q, end of input, and the deadman stages: the 0.5 s stop latched at 0, keepalives not driving again, the 2 s ESTOP, counting from the start), `test_bridge_teleop_node.py` (rclpy, one process: 20 Hz, the keys, end of input → `/cmd_vel` 0, `/rover/estop`, silence), `tools/test_teleop_keys.py` (the ssh command, the key mapping, a keepalive every 100 ms, Ctrl+C → q, waiting for the ready line).
- The shutdown tests: `tools/test_battery_watch.py` (ROS-free: the threshold, the 30 s hold, resets, pauses while running, on ADC faults and on stale data, wall power, the latch, the overrides, the synced SHUTDOWN line before the command, ACTION=log), `test_bridge_power_state.py` (the file the watcher reads, the watch file read back, the countdown in `/diagnostics`), and two node tests (the real file, with the STATUS's receive time).
- **Mutants:** `python3 tools/run_bridge_mutants.py` (WSL, ROS sourced; all 97 take ~2 h, because each runs the whole test folder, ~45 s, and a node-test kill runs twice; `R X D` is ~25 min, `K X` ~10 min, `V X` ~3 min: the watcher's mutants run only its own tests) plants each bug and requires a test to fail. The R group covers steering, the K group the teleop's deadman, the V group the low-battery shutdown. **X0 is a no-op control** (a comment change), which must survive. A killed control means the runner itself is broken.

**Open items (tests):**
- **Done: the full mutant run (97) against the new `safety.py`, the teleop and the shutdown watcher.** 2026-10-22, under the exit-1-only rule (only a failing test kills; any other exit code is BROKEN): **96 killed, X0 survived, 0 BROKEN**, 0 flaky, 0 invalid, in 80 min. G 6, A 9, B 9, C 4, D 6, S 8, H 3, W 4, T 3, R 18, K 11, V 15; 20 of the kills by the node tests, each repeated on the rerun.
- **The 10 s-line test can flake.** `test_the_10_s_line_has_the_heartbeat_numbers_and_the_event_age` checks a rounded `12.3 s ago`, so a ~10 ms delay flips it to 12.4. It failed a mutant baseline once on 2026-10-22, and passed on 5 reruns. Fix it only if it recurs.
