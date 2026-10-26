# firmware

STM32 firmware for the rover's wheel encoder, IMU, drive motor and steering
servo, running on a **NUCLEO-L432KC** (STM32L432KCU3, Cortex-M4, clocked at
32 MHz from MSI + PLL; see the open item on clock accuracy). It holds the
wheel speed in closed loop (`v`, see [Speed control](#speed-control-v)),
stops the motor when the wheel stalls, watches the LiPo's voltage and locks
the motor out when the pack runs low (see [Battery](#battery)). A hardware
watchdog (IWDG) resets the MCU if the main loop hangs.

The electronics sit on a soldered protoboard carrier on the truck (see
[Carrier hardware](#carrier-hardware)). The Orange Pi talks to the
board over USB, through the ST-LINK's virtual COM port, with the binary,
CRC-checked **Pi link** (see [Pi link (binary protocol v1)](#pi-link-binary-protocol-v1)).
The board boots in the human text mode described below (telemetry line;
`m`/`v`/`s` commands only in a bench or test build, see
[Build switches](#build-switches)) and switches to the binary link for good
(until a reset) when it receives a valid HELLO frame.

## What it does now

At boot it initializes the ICM-42688-P IMU over SPI1 (about 40 ms) and prints
one line with the result:

```
imu_init=ok who=0x47
```

On failure the first field is `err_spi`, `err_who_am_i` or `err_verify`. `who`
shows the ID byte that was actually read (0x00 or 0xFF usually means MISO, power
or CS wiring). **If IMU init fails, the firmware keeps running**: encoder,
motor, steering, battery monitor and telemetry work as usual. The IMU is
skipped and its fields print `imu_ok=0` and zeros, the green LED
double-blinks instead of the normal 500 ms blink (see [Heartbeat LED](#heartbeat-led)),
and `motor_console.py` shows `IMU FAIL` on its status line. Refusing to drive
*autonomously* without an IMU is the Pi's job (not implemented yet); the MCU
doesn't need the IMU to keep the motor safe.

The main loop runs on a fixed 1 kHz schedule driven by the HAL millisecond tick.
Every 1 ms it reads the AS5600 magnetic encoder over I2C1 (400 kHz) and gets the
raw angle (0..4095 per revolution) and the magnet magnitude. It unwraps the
angle into a continuous signed 32-bit count that carries on across revolutions,
twice: once as read (the raw count, used only by the capture build), and once
after removing the sensor's 4-per-revolution angle error (see
[Encoder angle correction](#encoder-angle-correction-h4)); the corrected count
feeds `count`, `d100` and the 20 ms speed estimate. The unwrapper assumes less
than half a revolution between good samples, and a failed read is skipped
rather than fed in. Every 100 ms it also reads the magnet
STATUS flags (MD/ML/MH) and AGC. Before the motor state machine, every 1 ms,
the speed controller (its PI runs every 10 ms) and the stall check act on the
previous ms's estimate. After the encoder, every 1 ms, it burst-reads
the IMU's temperature, accel and gyro (one 15-byte SPI frame at 2 MHz). Every
10 ms it collects one battery-voltage reading from the ADC and starts the next
one, without ever waiting for it (see [Battery](#battery)). Every
100 ms (10 Hz) it sends one telemetry line over the ST-LINK virtual COM port,
non-blocking via `HAL_UART_Transmit_IT`:

```
t_ms=... raw=... count=... d100=... MD=. ML=. MH=. AGC=... mag_min=... mag_max=...
i2c_err=... overrun=... resync=... skipped=... i2c_us_max=... loop_us_max=...
imu_ok=. ax=... ay=... az=... gx=... gy=... gz=... temp=...
gsum_x=... gsum_y=... gsum_z=... n_imu=... imu_err=... spi_us_max=...
cmd_pct=... duty=... mstate=. wd_trip=... cmd_ok=... cmd_bad=... cmd_clamp=... rx_err=...
rst=... s_pct=... s_us=... sstate=. s_ok=... s_wd=...
vbat=... bstate=. vdda=...
vset=... vest=... ctl=. vok=. vi=... stall_n=... enc_bad=... mfault=.
```

(This is one line in the real output, ending in `\r\n`. This is **telemetry
v5**: v1 was the encoder and IMU fields (420 bytes at most), v2 appended the
motor fields (548 bytes), v3 the reset cause and the steering fields (618
bytes), v4 the battery fields (651 bytes), v5 appends the speed control
fields (see [Speed control fields](#speed-control-fields-v5)). The longest v5
line is 786 bytes counting the v5 fields at their full 32-bit width, about
68 ms at 115200 baud; even with **every** field at its full 32-bit width it is
956 bytes, so it always fits the 1024-byte buffer (`main.c`, comment on
`telemetry_line`; `tools/test_firmware_sources.py` recomputes both from the
field list). A typical v5 line is about 510 bytes (~44 ms). Earlier fields are
unchanged in position, and the tools accept v1 to v5 lines
(`tools/serial_logger.py`). The field order is set in `main.c`, where the line
is built, and `tools/test_firmware_sources.py` checks that it matches the
tools.)
`count` is the unwrapped **corrected** count (the angle error removed; see
[Encoder angle correction](#encoder-angle-correction-h4)) and `d100` is its
change since the last line that was sent; `raw` is the angle as read,
uncorrected. Over whole revolutions the corrected and raw counts agree
exactly, so distances (and `meters_per_encoder_tick`) are unchanged. The error counters (`i2c_err`, `overrun`, `resync`, `skipped`,
`imu_err`) are totals since boot. `skipped` counts report windows whose line was
not sent: the UART was still sending the previous line when the window closed
(that window is then merged into the next line, so no counts are lost), or a
line was dropped because it did not fit the 1024-byte buffer (it is never sent
cut short) or the UART refused to start sending it. The `mag_*`, `*_us_max`, `gsum_*` and `n_imu`
fields cover the current report window, timed with the DWT cycle counter. If the
loop falls 5 ms or more behind (for example on an I2C timeout), it restarts the
schedule instead of running the missed iterations in a burst (see
[Loop scheduling](#loop-scheduling-due-times) for how the 100 ms tasks survive
that). The green LED (LD3) shows the loop is alive (see
[Heartbeat LED](#heartbeat-led)). Everything is in raw counts. The Pi converts
counts to meters using `vehicle.cfg`, so the wheel size is not built into the
firmware.

**In binary mode** (after a HELLO from the Pi link) this text line is not
built or sent at all; the board sends STATE frames every 10 ms and STATUS
frames every 100 ms instead (see [Pi link](#pi-link-binary-protocol-v1)).
Everything in this section describes text mode, which is byte-for-byte the
same as before the link existed.

A single `snprintf` of the whole line would not fit in a 1 ms slot. The line is
therefore built in five consecutive iterations: the encoder part in the report
iteration, the IMU part in the next one, the motor part in the one after that,
the reset cause, steering and battery part in the fourth, and the speed control
part plus the start of sending in the fifth (each field costs ~14 µs to format
at `-Og`). The later parts are formatted from copies taken when the window
closed, so every part describes the same 100 ms. The battery sample (phase 5)
stays clear of all five.

### Loop scheduling (due times)

The 100 ms tasks (telemetry report at phase 0, magnet health reads at phase
50), the 10 ms battery sample (phase 5, so it never shares a ms with the
report or health iterations) and the heartbeat LED are **due by time** (`Core/Src/loop_schedule.c`,
pure C, host-tested): a task runs at the first iteration whose scheduled time
has reached its next mark (the marks are still the times where
`t % 100 == phase`), and the next mark is then the first one after that
iteration. In a normal run (one iteration per ms) they run on exactly the same
ms as before, so `t_ms` is still a multiple of 100 and health still runs at
...50.

Why: they used to run when `t % 100 == phase` held, which needs an iteration
at exactly that ms. When iterations take much longer, the loop resyncs every
time and the scheduled times jump. With a stuck encoder I2C bus every HAL read
waits its 25 ms busy timeout, so each iteration takes ~52 ms (see
[Why 250 ms](#hardware-watchdog-iwdg)) and the times step by exactly 52 ms.
gcd(52, 100) = 4, so in about three boots out of four `t % 100 == 0` and
`t % 500 == 0` were never true again: **no telemetry and no heartbeat, while
the loop ran, refreshed the IWDG and would still accept `m 0` / `m 20`.** The
board looked dead but was not. This is the **likely, unconfirmed** cause of
the 2026-10-14 boot (`carrier_usb_1.log`: two boot lines 13.8 s apart, the
second probably the reset button, nothing else) with the IMU and encoder both
unplugged (they share the 3V3 pigtail). It was not reproduced on purpose.

What changes when iterations are long (52 ms steps):

- reports are due about every 100 ms (every second iteration, 104 ms; now and
  then after 52 ms). A line is built over five iterations (four when this was
  simulated), so with 52 ms iterations about every other report finds the
  previous line still being built and is counted in `skipped` (merged into
  the next line, as usual): expect a line every 208–260 ms (about 4.6 per
  second; from a simulation of the loop's order with the four-step line, not
  re-simulated for five, not measured), `t_ms` no longer on multiples of 100,
  and `resync`, `i2c_err` and `skipped` climbing;
- the heartbeat keeps its pattern, at roughly the right speed;
- after a single resync jump, each task runs **once** (late), never as a
  burst of the missed ones. Its `t_ms` is then the late time, not a multiple
  of 100; the logger's `t_ms_gaps` and `analyze_static_imu.py`'s segments
  count any step that isn't exactly 100 (after a resync the old code skipped
  the window instead: one step of 200).

**Verified by host tests only** (`tests/test_loop_schedule.c`: 1 ms steps
match the old rule exactly; 52 ms steps from every starting ms; every step
size 1–250 ms; a 10 s jump; the 49.7-day tick wrap). The stuck-bus case was
not run on the hardware; the sensors are not unplugged on purpose.
`tools/test_firmware_sources.py` fails if a `%` comparison comes back into
the main loop.

### Heartbeat LED

The green LED (LD3) steps through a pattern every 100 ms (its own due-time
schedule), so it shows the board's state without a PC:

| Pattern (1 s cycle) | Meaning |
|---------------------|---------|
| 500 ms on, 500 ms off | loop running, IMU OK |
| on 100, off 100, on 100, off 700 (double blink) | loop running, **IMU init failed** (`imu_ok=0`); everything else runs as usual |
| 100 ms on, 100 ms off (fast) | **motor FAULT** (`mstate=3`), needs a reset. A battery lockout (cutoff, ADC fault, removed while driving) is a motor FAULT too |
| off or steady | loop not running (hang until the IWDG resets it, or a debug halt) |

FAULT wins over IMU missing: it is the one that needs action before the motor
can run again. A missing IMU changes nothing on the MCU.

### IMU fields

| Field | Meaning | To physical units (DS-000347 rev 1.2) |
|-------|---------|----------------------------------------|
| `imu_ok` | 1 if init succeeded (fixed after boot) | |
| `ax ay az` | latest accel sample, raw int16 | g = raw / **8192** (±4 g, §3.2) |
| `gx gy gz` | latest gyro sample, raw int16 | deg/s = raw / **65.5** (±500 dps, §3.1) |
| `temp` | latest temperature, raw int16 | deg C = raw / **132.48** + 25 (§14.5–14.6) |
| `gsum_x/y/z` | sum of gyro raw over the window | window mean = gsum / n_imu, then / 65.5 |
| `n_imu` | good IMU samples in the window (~100) | |
| `imu_err` | failed IMU reads since boot: HAL error/timeout, or all 14 data bytes 0x00 or all 0xFF (dead MISO line, which SPI itself can't detect) | |
| `spi_us_max` | longest IMU SPI read in the window, µs | |

The sums stop at 65535 samples so an int32 sum can never overflow, even if
lines are skipped for a minute. In the normal case `n_imu` is about 100.
`spi_us_max` is not part of `i2c_us_max`, but `loop_us_max` includes both.

### Motor fields

Snapshot taken when the window closes. Counters are totals since boot.

| Field | Meaning |
|-------|---------|
| `cmd_pct` | last valid **`m`** as received, -100..100 (before the 30 % cap; also shown when it was ignored because the motor was latched). **`v` doesn't change it**: while `v` drives, it still shows the last `m` (often 0, or a stale value from a bench test) |
| `duty` | duty applied right now, signed timer ticks: 1600 = 100 %, so the 30 % cap is ±480 and 20 % is 320. Positive = forward |
| `mstate` | 0 = off (driver disabled, armed), 1 = running (driver enabled: ramping, holding, reverse dwell or ramping down), 2 = latched (driver disabled until `m 0` or `v 0`; at boot, after a watchdog trip, a stall, or an estimate that stayed invalid under `v`), 3 = FAULT (driver disabled until reset, whatever is commanded; see [FAULT](#fault-mstate3)) |
| `wd_trip` | command watchdog trips |
| `cmd_ok` | valid `m` **and `v`** commands, including clamped ones and ones ignored while latched |
| `cmd_bad` | rejected lines, whatever the letter (bad syntax, \|value\| > 100, too long, hit by a receive error, or a letter the firmware doesn't act on) |
| `cmd_clamp` | valid `m` commands above the cap, applied as the cap (`v` has no clamp count: the controller's output is clamped to the cap) |
| `rx_err` | received bytes lost or corrupted: UART framing/noise/overrun errors, receive-queue overflows, and receptions the main loop had to restart. Normally 0 |

`loop_us_max` also includes the motor section (command input, speed control
and stall check, motor and steering state machines and pin writes), which runs
at the start of every iteration; `i2c_us_max` still starts at the encoder
read, so it means the same as before (it now also includes the ~12 µs angle
correction and speed estimate, which sit between the two encoder reads).

### Reset cause and steering fields (v3)

Snapshot taken when the window closes. Counters are totals since boot.

| Field | Meaning |
|-------|---------|
| `rst` | why the MCU last restarted: the `RCC_CSR` reset flags read once at boot (then cleared), bits 0..7 = FW, OBL, PIN, BOR, SFT, **IWDG** (32), WWDG, LPWR. The same in every line until the next reset. See [Reset cause](#reset-cause-rst) |
| `s_pct` | last valid `s` command as received, -100..100 (+ = left) |
| `s_us` | servo pulse width applied right now, µs; **0 = no pulses** |
| `sstate` | 0 = off (no pulses: nothing commanded since boot), 1 = active (following `s`), 2 = timed out (no `s` for 500 ms: centre), 3 = FAULT (no pulses until reset) |
| `s_ok` | valid `s` commands |
| `s_wd` | steering timeouts (ACTIVE → centre) |

### Battery fields (v4)

Snapshot taken when the window closes (`main.c`).

| Field | Meaning |
|-------|---------|
| `vbat` | pack voltage in mV, **filtered** (the value the thresholds are checked against, see [Battery](#battery)). A few mV on USB with no pack |
| `bstate` | 0 = no battery, 1 = OK, 2 = low (warning), 3 = cutoff, 4 = ADC fault, 5 = removed while driving. 3, 4 and 5 are latched until reset and lock the motor out (`BatteryState`, `Core/Inc/battery_monitor.h`) |
| `vdda` | the MCU's analog supply in mV, measured against the chip's internal reference (`VREFINT`). Kept even when it is implausible, so a fault can be diagnosed |

End of a real v4 line (2026-10-14, LiPo connected, motor stopped):

```
... rst=4 s_pct=0 s_us=0 sstate=0 s_ok=0 s_wd=0 vbat=7902 bstate=1 vdda=3315
```

### Speed control fields (v5)

Snapshot taken when the window closes (`main.c`, `SpeedReport`).

| Field | Meaning |
|-------|---------|
| `vset` | last valid `v` as received, speed units (1 unit = 50 counts/s; see [Units](#units)). 0 before the first `v`, and set back to 0 by any `m` (open loop) |
| `vest` | the speed estimate at window close: corrected count now minus 20 ms ago, in speed units. **Meaningful only when `vok=1`**; while invalid it can show nonsense (a value of −1513 was seen right after a frozen-angle test) |
| `ctl` | what the speed controller is doing (table below) |
| `vok` | 1 = the estimate is valid, 0 = not (a failed read, a jump, no magnet or a timing step in the last 21 ms; see [Speed estimate](#speed-estimate-and-the-odometer)) |
| `vi` | the PI's integrator in duty ticks (rounded; same sign as `vset`) |
| `stall_n` | stalls since boot, **both** rules (locked wheel and saturation, see [Stall protection](#stall-protection)) |
| `enc_bad` | latches because the estimate stayed invalid for 50 ms under `v` |
| `mfault` | the motor's FAULT reason (table below), 0 unless `mstate=3` |

`ctl` (`SpeedControlState`, `Core/Inc/speed_control.h`; append only):

| `ctl` | Console | Meaning |
|---|---|---|
| 0 | `open` | open loop: `m` (or nothing yet). The controller writes nothing |
| 1 | `run` | `v` with an effective setpoint. **Shown even while the motor is latched** (`mstate=2`): it says what the controller wants, `mstate` says whether the motor follows |
| 2 | `stop` | `v 0`, or `|v|` below the 20-unit minimum: coasting |
| 3 | `GATE` | a `v` start (or sign change) waiting for the wheel to (nearly) stop |

`mfault` (`MotorFaultReason`, `Core/Inc/motor_control.h`; append only, pinned
by `tools/test_firmware_sources.py`):

| `mfault` | Console | Reason |
|---|---|---|
| 0 | – | none |
| 1 | `PWM PERIOD` | TIM1 ARR + 1 ≠ 1600, checked at boot |
| 2 | `CAP GUARD` | output-stage guard: a duty above the cap reached the pins |
| 3 | `DIRECTION GUARD` | output-stage guard: a direction change with a duty set |
| 4 | `LOW BATTERY` | battery cutoff (`bstate=3`) |
| 5 | `BATTERY ADC` | battery ADC fault (`bstate=4`) |
| 6 | `BATTERY REMOVED WHILE DRIVING` | `bstate=5` |
| 7 | `REPEATED STALL` | 3 stalls (either rule) within 60 s |

End of a real v5 line (2026-10-15, `v 50` on the lift):

```
... vbat=7862 bstate=1 vdda=3315 vset=50 vest=48 ctl=1 vok=1 vi=293 stall_n=0 enc_bad=0 mfault=0
```

### IMU configuration and why

| Setting | Value | Why |
|---------|-------|-----|
| Gyro | ±500 dps, 1 kHz ODR, low-noise mode | An RC truck yaws well under 500 dps; 65.5 LSB/dps keeps resolution for the static bias log |
| Accel | ±4 g, 1 kHz ODR, low-noise mode | Covers bumps and launches without clipping at 1 g gravity + driving loads |
| Filters | Reset defaults (UI LPF BW = max(400 Hz, ODR)/4 = 250 Hz) | Not tuned yet |
| I2C interface | Disabled (`INTF_CONFIG0.UI_SIFS_CFG = 11`) | SPI traffic can never look like an I2C start condition |
| SPI output slew | 12–36 ns (`DRIVE_CONFIG.SPI_SLEW_RATE = 001`, reset is < 2 ns) | 2 MHz bits are 500 ns long, so slow edges cost nothing, and they ring less on 21 cm jumpers (the breadboard length; unverified on the carrier) |
| SPI | Mode 3, 2 MHz, 8-bit, MSB first, software CS | Chip supports modes 0 and 3 by default (§14.1); 2 MHz is modest for the wire length |

### Mounting

The IMU's axes are aligned with the vehicle: **X forward, Y left, Z up** (ROS REP
103). The firmware does no axis remapping. At rest on level ground, `az` reads
about +8192 (+1 g).

### Known limitation: IMU sampling isn't synchronized

The IMU produces a sample every 1 ms on its own clock, and the loop reads every
1 ms on the MCU's clock. The two drift relative to each other, so now and then
the loop reads the same sample twice or misses one. Window means are barely
affected, but sample-to-sample timing is not exact. The fix is to read on the
INT1 data-ready interrupt (PB0 is already wired and configured as an input) or
to drain the IMU's FIFO.

## Carrier hardware

The electronics moved from the breadboard onto a soldered protoboard
**carrier** on the truck. Wiring and measured values here can't be checked
from the code; they are as built and measured.

### Layout and rails

- The NUCLEO-L432KC and the TB6612 board (#2) sit in **female headers**, so
  either can be pulled out. Wiring is 22 AWG solid.
- **Male breakout strips** are soldered along both Nucleo rows; header pins
  such as A2 and A5 are reached there.
- **VM rail** (red tape): LiPo + via the keyed battery connector.
  **GND rail**. **Bottom + rail = 3.3 V** (from the Nucleo).
- Female Dupont sockets are soldered onto the VM and GND rails; they feed
  buck #2 (servo power).
- **One 3V3 pigtail feeds the IMU, the AS5600 and the TB6612's VCC.**
  Unplugging it takes down both sensors *and* the I2C pull-ups (they are on
  the AS5600 board), which is what makes the encoder bus hang (see
  [Loop scheduling](#loop-scheduling-due-times)).

### Motor driver (TB6612 #2, both channels paralleled)

| TB6612 | Nucleo | MCU | Role |
|---|---|---|---|
| PWMA + PWMB | D9 | PA8 | TIM1_CH1, 20 kHz |
| AIN1 + BIN1 | D10 | PA11 | MOTOR_IN1 |
| AIN2 + BIN2 | D2 | PA12 | MOTOR_IN2. **No jumper cap on D2** (see [warning](#drive-motor-tb6612fng)) |
| STBY | D6 | PB1 | MOTOR_STBY, **10 kΩ pull-down to GND** |
| VCC | 3V3 | | logic supply |
| VM | VM rail | | motor supply. **470 µF 16 V electrolytic VM–GND beside the TB6612, stripe to GND** |
| AO1+BO1 / AO2+BO2 | | | motor black / motor red |

Why the STBY pull-down: between reset and `MX_GPIO_Init()` (and while
flashing or holding reset) PB1 isn't driven. The pull-down keeps the driver
disabled whenever the MCU isn't actively enabling it. The 470 µF supplies the
motor's PWM current pulses locally, so less ripple reaches the battery leads
and the voltage reading.

### Steering servo power (buck #2)

- LM2596 buck #2, output **5.57 V** unloaded (measured 2026-10-14). Input
  from the VM/GND rail sockets.
- OUT+ (**blue** wire) → servo red; OUT− (**green**) → servo brown; servo
  signal (**white** jumper) → **A2** breakout pin (PA3, TIM15_CH2). In the
  3-pin housing the order green/blue/white = brown/red/yellow.
- **1000 µF 16 V electrolytic across buck #2's OUT+/OUT− pads, stripe on
  OUT−.** Why: on 2026-10-09 the servo rail sagged from 5.56 V to about
  4.45 V when the wheels were forced (before the capacitor). The dip with the
  capacitor fitted has not been re-measured.
- **Never connect the buck's output to the Nucleo's 3V3 or 5V pins.**

### Battery voltage divider (2026-10-14)

```
VM ──22 kΩ──┬──10 kΩ── GND
            └── A5 (PA6, ADC1_IN11)
```

- Built as a free-hanging, heat-shrunk Y harness: the 22 kΩ leg has **green**
  heat shrink (plugs into the VM rail), the 10 kΩ leg **black** (GND rail),
  the junction a red female onto A5.
- Nominal ratio 10 / 32 = 0.3125; **measured 0.3102** (VM 7.92 V, A5
  2.457 V). The measured value is the firmware constant
  (`BATTERY_DIVIDER_NUM` / `BATTERY_DIVIDER_DEN` = 3102 / 10000). A full 2S
  pack (8.4 V) gives about 2.61 V at the pin, safely under the 3.3 V rail.
- Draws about 0.25 mA while the pack is connected.
- **No filter capacitor on A5** (no ceramics in hand). The firmware filters
  instead (see [Battery](#battery)).

### D5 must stay empty

Solder bridge **SB16** on the Nucleo joins D5 (PB6) to A5 (PA6). So PB6 is
configured as **analog** in CubeMX, labelled `SB16_A5_KEEP_ANALOG`: an analog
pin neither drives nor pulls, so it can't fight the divider.
`tools/test_firmware_sources.py` checks that PB6 is never driven or pulled.
**Anything plugged into D5 corrupts the battery reading**, and with it the
cutoff.

### Sensors

IMU (ICM-42688-P, SPI1) and AS5600 encoder (I2C1), both powered from the 3V3
pigtail. Pin by pin: [Sensor wiring](#sensor-wiring).

### Buck #1 and the Pi link

- **Buck #1** (5.1 V) powers the Orange Pi from the pack, through the Pi's
  header pins 4 (5 V) and 6 (GND), since 2026-10-16. So the Pi drains the pack
  too, also after a cutoff. The Pi shuts itself down when the pack is flat at
  rest ([Low-battery shutdown](../docs/ros-bridge.md#low-battery-shutdown-group-5)),
  which slows the drain but doesn't stop it.
- Pi ↔ Nucleo link = **USB** (ST-LINK virtual COM port, `/dev/ttyACM0` on the
  Pi). A direct UART was rejected: USART2 is on the VCP, D0/D1 are the
  encoder's I2C, and LPUART collides with the servo on A2.

## Battery

The MCU measures the 2S LiPo itself and stops the motor when the pack is too
low: the cutoff is 6.6 V, 3.3 V per cell (`battery_monitor.h`), so the motor
can't keep draining a flat pack. The Pi may not be running or may have
crashed, so the
thresholds are **firmware constants** (`Core/Inc/battery_monitor.h`), not
settings in `vehicle.cfg`: the MCU enforces them without the Pi.

### Connecting the battery (every time)

1. **Everything wired except the battery.** No loose wire ends that could
   become live. Unpowered checks for anything new or touched, for example:
   - VM ↔ GND, VM ↔ VCC, VM ↔ Nucleo 3V3 and D2 ↔ GND all **silent**
   - the signal wires and the A/B ties **beep**
2. (Optional) meter on the pack: about +8 V. Then put the meter away; one
   task in the hands at a time.
3. Nucleo on USB, console running **with a new log name** (the tools
   overwrite an existing log), reset → `mstate=2`; the status line shows
   `NO BAT`.
4. **Mate the keyed connector.** Watch for 5 s. Within about 1–2 s the status
   line shows `vbat=7.9xV bst=1`.
5. **To finish:** Esc (stops the motor) → unplug the keyed connector → wait
   for `NO BAT` → Ctrl+C.
6. **Heat check:** TB6612, motor, Nucleo, servo, buck #2.

Rules:

- **Never flash or debug with the LiPo connected** (see [Safety](#safety)).
- **The LiPo is plugged in only while testing.** The cutoff stops only the
  motor; the Pi (buck #1), steering, buck #2 and the divider keep drawing
  from the pack.
- Why the keyed connector: on 2026-10-06 a hand-wired, reversed LiPo destroyed
  TB6612 #1.
- **A fuse is still open and required before floor driving** (see
  [Open items](#open-items)).

### How the voltage is measured

ADC1's **injected** group converts two channels per sequence: rank 1 the
chip's internal reference (`VREFINT`), rank 2 the divider on PA6
(`ADC1_IN11`), 640.5 cycles sampling each, ADC clock = SYSCLK 32 MHz, about
41 µs per sequence. The regular group isn't used. (`MX_ADC1_Init` in
`main.c`, `rover_firmware.ioc`.)

Conversion, integer math only (`battery_monitor.c`):

- `VDDA = 3000 mV × VREFINT_CAL / VREFINT reading`. `VREFINT_CAL` is the
  reading ST stored in the chip at the factory at exactly 3.0 V, so this gives
  the real analog supply. Why: the Nucleo's "3.3 V" isn't exact (about 3.31 V
  here), and every ADC reading is relative to it. With the correction, the
  battery reading doesn't depend on the rail.
- `battery = VDDA × reading / 4095 / 0.3102` (the measured divider ratio).

Timing (`main.c`):

- **Every 10 ms** the loop reads the previous sequence's result if it is
  ready and starts the next. It **never waits** for the ADC. A sample that
  isn't ready is counted as missed.
- **At boot:** ADC self-calibration, then one throwaway sequence (waited for,
  2–3 ms timeout), so the filter isn't seeded with a reading taken before the
  reference settled.

**Filter:** IIR, **1/64 per 10 ms sample** (time constant about 0.64 s),
seeded with the first sample so it doesn't ramp up from 0 at boot. Why this
heavy: A5 has no capacitor, so the motor's 20 kHz PWM ripple on VM reaches
the samples. With a light filter the value near a threshold jitters across it
and keeps restarting the 3 s hold. `test_battery_monitor.c` ("ripple") feeds
±200 mV of ripple onto a pack 5 mV under the cutoff: with 1/64 the jitter is
±1.6 mV and the cutoff still comes; with 1/8 it would be ±13 mV, more than
the 5 mV margin, so the hold would keep restarting.

### States and thresholds

All checked on the **filtered** voltage, each only after its condition has
held without a break for the whole time:

| Event | Condition | Effect |
|---|---|---|
| Battery present ("armed") | ≥ 5.5 V for 0.5 s | `bstate` 0 → 1; warning and cutoff now active |
| Battery removed | < 5.0 V for 0.5 s | back to `bstate=0` (`NO BAT`) |
| Warning on | < 7.0 V for 3 s | `bstate=2` (`LOW BAT`), **no lockout** |
| Warning cleared | ≥ 7.2 V for 3 s | back to `bstate=1` |
| **Cutoff** | < 6.6 V for 3 s, while armed | `bstate=3`, **motor lockout, latched until reset** |

Why these rules:

- **Nothing happens without a pack.** On USB alone the reading is about 0 V,
  so it starts as "no battery" and neither warns nor cuts off. The no-LiPo
  bench checks keep working.
- **Removal (0.5 s) is faster than cutoff (3 s).** Unplugging a pack makes
  the voltage fall through 6.6 V, and it must disarm rather than latch a
  cutoff. A compile-time check enforces that order.
- **The cutoff is latched.** A resting LiPo's voltage recovers, so without
  the latch the motor would restart, sag, cut off, and so on. Only a
  reset clears it, and that is a deliberate act.
- **Warning and clear are 200 mV apart**, so a pack hovering near 7.0 V
  doesn't flicker.

`bstate` values (`BatteryState`, `battery_monitor.h`):

| `bstate` | Console | Meaning | Motor |
|---|---|---|---|
| 0 | `NO BAT` | no battery (boot, or removed while stopped) | allowed (nothing to drive it) |
| 1 | – | OK | allowed |
| 2 | `LOW BAT` | warning | allowed |
| 3 | `CUTOFF` | cutoff | **locked out until reset** |
| 4 | `ADC FAULT` | the reading can't be trusted | **locked out until reset** |
| 5 | `REMOVED WHILE DRIVING` | pack lost while the motor had a duty | **locked out until reset** |

If several apply, the order is ADC fault > cutoff > removed while driving >
no battery > low > OK.

**ADC fault** (single sample latches): calibration word outside 1600–1700,
ADC calibration/start failing at boot, a conversion error, a reading above 12
bits, VDDA outside 3000–3600 mV, or **3 missed samples in a row**. Why
latched: with the reference wrong, every voltage is wrong, the cutoff
included.

**Removed while driving:** judged at the sample where the removal is
*confirmed* (filtered value under 5.0 V for 0.5 s), using that ms's motor duty
(`battery_monitor.c`, `main.c`). Non-zero duty → `bstate=5`, motor locked out
until reset. Why: a normal unplug stops the motor first (Esc), so losing the
pack while driving means a loose connector or a pack collapsing under load.
Removed while stopped → just `NO BAT`.

**Lockout = motor FAULT.** A cutoff, ADC fault or removal while driving puts
the motor into FAULT (`mstate=3`, fast blink), reset-only (see
[FAULT](#fault-mstate3)). It stops **only the motor**: steering keeps
following `s` and centring on link loss, and the Pi (buck #1), buck #2 and
the divider keep drawing. **Unplug the pack promptly** after a cutoff. The Pi's
own low-battery shutdown (6.8 V at rest for 30 s) slows the drain, but doesn't stop it.

### Cutoff test build

`#define BATTERY_CUTOFF_TEST 0` in `Core/Inc/battery_monitor.h`. When 1, the
cutoff/warning/clear thresholds become 8500 / 8600 / 8800 mV (above a full
pack), so a real pack trips the cutoff on the bench, about 4–5 s after
plug-in (measured). Presence and removal are unchanged. The build shows a
`#warning`, and both `tools/test_firmware_sources.py` and
`test_battery_monitor.c` fail if it is committed as 1. **Never commit it as
1.**

### Bench results (2026-10-14, the carrier build)

| Test | Result |
|---|---|
| USB only | `vbat=5 bstate=0 vdda=3313`, `loop_us_max` 656 |
| LiPo at rest | meter 7.92 V = telemetry 7.92 V |
| Under load (`m 20`) | meter 7.89 V = telemetry 7.89 V: no visible ripple offset at 20 % duty with the 470 µF fitted |
| Plug-in | armed after about 1.1 s, no warning on the way up |
| Unplug while stopped | `NO BAT`, no false cutoff |
| Unplug decay | filtered `vbat` reaches "removed" about **0.85 s** after unplugging, against the 3 s cutoff hold (about 3.5× margin), so no bleed resistor is needed. It is this fast because buck #2 draws from VM. **If buck #2 is ever disconnected, re-measure** |
| Cutoff test build | motor stopped by itself while driving, `mstate=3`, fast blink; still latched after unplugging; reset clears it; steering worked throughout |

## Drive motor (TB6612FNG)

The stock 88T brushed motor is driven by one TB6612FNG board with **both
channels in parallel** (PWMA+PWMB, AIN1+BIN1, AIN2+BIN2, AO1+BO1, AO2+BO2).
VCC is 3.3 V from the Nucleo, VM is the 2S LiPo (cutoff 6.6 V, full 8.4 V)
via the keyed connector and the VM rail (wiring:
[Carrier hardware](#motor-driver-tb6612-2-both-channels-paralleled)).
Motor **red → AO2 + BO2** (tied), **black → AO1 + BO1** (tied). With this
wiring `m +N` drives the wheels forward with `d100` positive, and `m -N` drives
backward with `d100` negative, so `MOTOR_DIRECTION_INVERTED` stays 0 (verified
by driving on 2026-10-06; see [Direction](#direction)).

> **Warning: Nucleo spare jumper on D2/GND.** The NUCLEO-L432KC arrived with a
> black jumper cap across D2 and the GND pin next to it (CN3), probably ST's
> parking spot for a spare jumper (unverified; to be checked in UM1956). With
> the cap on, D2 = PA12 = `MOTOR_IN2` is shorted to GND: forward still works
> (it needs IN2 low anyway), but reverse can't set IN2 high, so the TB6612 sees
> IN1 = IN2 = low and **coasts**. The motor doesn't move and the chip stays
> cool, while telemetry shows e.g. `duty=-240 mstate=1`. **The cap must be
> removed and must never be put back on D2/GND.**

### MCU pins

| Signal | MCU pin | Setting |
|--------|---------|---------|
| PWM | PA8 (TIM1_CH1) | 20 kHz (32 MHz / 1600), 1600 duty steps, PWM mode 1, compare preload on |
| IN1 | PA11 | push-pull, starts low |
| IN2 | PA12 | push-pull, starts low |
| STBY | PB1 | push-pull, starts low (driver disabled) |

Code (each pure-C module host-tested, see [Host tests](#host-tests)):
`Core/Src/command_line.c` (line collector and parser, pure C),
`Core/Src/command_dispatch.c` (routes each line to the motor or the steering,
pure C), `Core/Src/motor_control.c` (the motor state machine, pure C),
`Core/Src/tb6612_plan.c` (the output stage's own guards, pure C),
`Core/Src/tb6612.c` (reads and writes the pins and the PWM compare register,
no decisions), `Core/Src/byte_ring.c` (receive queue).

### Commands

Sent over the same VCP as the telemetry (115200 8N1), one per line:

```
v 50      closed loop: hold 50 speed units forward (see Speed control)
v -30     30 units reverse
v 0       stop (coast; also re-arms after a watchdog trip, a latch or a reset)
m 20      open loop: 20 % duty forward (bench/diagnostic only)
m -15     15 % reverse
m 0       stop (also re-arms)
s 50      steer 50 % left (+ = left, ROS REP 103)
s -50     steer 50 % right
s 0       centre
```

This is the **text-mode** (bench) grammar, and it acts **only in a bench or
test build**: the committed build (`BENCH_TEXT_COMMANDS 0`, the truck build)
counts every text line in `cmd_bad` and acts on none, so its text mode only
waits for a HELLO; set `BENCH_TEXT_COMMANDS 1`, or use a test build, for
these commands (see [Build switches](#build-switches)). The gate is
`LinkRx_TextCommandAllowed` (`link_rx.c`, host-tested), asked for every
finished line before anything acts. **The Pi doesn't use this grammar**: it
sends binary DRIVE frames (speed in counts/s, steering, explicit arm; see
[Pi link](#pi-link-binary-protocol-v1)), and once the board is in binary mode
these lines are ignored. `m` is a bench/diagnostic command only (there is no
open-loop message on the Pi link). The last of `m` / `v` wins: any `m`
switches the speed controller off (open loop) and any `v` switches it on.
Test builds add their own letters (`h` hang test, `c` capture, `e` encoder
faults); a normal build rejects them like any unknown letter.

Grammar (`Core/Inc/command_line.h`), the same for every letter: one lowercase
letter, one space, optional `-`, 1 to 3 digits, then `\n` (a `\r` before it is
accepted). Anything else is **rejected and counted** (`cmd_bad`) and never
acted on, including |value| > 100, lines longer than 16 bytes, and letters the
firmware doesn't act on. A motor value within ±100 but above the cap is
**clamped** to the cap and counted (`cmd_clamp`); it is still a valid command.
Empty lines are ignored. Use `tools/motor_console.py` to send commands while
logging (only one program can hold the COM port).

**Each letter feeds exactly one controller** (`command_dispatch.c`): `m` and
`v` feed the motor's command watchdog, `s` feeds the steering's own timeout,
and nothing else feeds either (the speed controller's own output doesn't feed
the watchdog). So a sender that keeps sending `s` but has stopped sending
`m`/`v` can't keep the motor running: the motor trips 500 ms after the last
`m` or `v` (`tests/test_command_dispatch.c` checks this end to end, through
the real parser; bench 2026-10-15: `-v` with `s` keepalives flowing → trip).
The reverse also holds: only `m`/`v` arriving, the steering centres after
500 ms while the motor runs on. `tools/test_firmware_sources.py` checks that
only the dispatcher feeds the motor watchdog in text mode, and that over the
Pi link only a fully validated DRIVE does (`MotorController_AcceptDriveCommand`
is called once, from `link_rx.c`, after the mode and payload checks).

### Behavior

| Rule | Value | Constant (`Core/Inc/motor_control.h`) |
|------|-------|-----------|
| Duty cap | 30 % (480 ticks) | `MOTOR_DUTY_CAP_PERCENT`: the one number to lower to be more careful |
| Ramp | 2 ticks per ms: 0 → 30 % in 240 ms | `MOTOR_RAMP_TICKS_PER_MS` |
| Reversal | ramp to 0, short brake for 100 ms, switch IN1/IN2 with duty 0, ramp up | `MOTOR_REVERSE_DWELL_MS` |
| Command watchdog | no valid `m` or `v` command (text mode) or validated DRIVE frame (binary mode) for 500 ms while running → driver disabled at once (no ramp) and latched | `MOTOR_COMMAND_TIMEOUT_MS` |
| Latch | at boot, after a watchdog trip, a stall ([Stall protection](#stall-protection)), an estimate that stayed invalid under `v`, and on the Pi link's ESTOP and every HELLO: non-zero commands are ignored until `m 0` or `v 0` (text mode) or a DRIVE with speed 0 **and the arm bit** (binary mode). Why it latched is kept as the latch reason (STATUS, [Latch reasons](#latch-reasons-and-stall-counters)) | `MotorController_Latch` |
| Stopped | always driver disabled: STBY low, IN1 = IN2 = low, duty 0 (the motor coasts) | |
| FAULT | driver disabled until reset, whatever is commanded (`m 0` included) | see below |

During the low part of each PWM period the TB6612 short-brakes the motor (slow
decay), so ramping down brakes actively. The bridge is only ever enabled with
duty 0 during the 100 ms reverse dwell (a short brake on purpose) and for the
1 ms between enabling the driver and the first non-zero duty.

### Direction

`MOTOR_DIRECTION_INVERTED` in `main.c` is **0**: a positive command drives
IN1 high and IN2 low. On 2026-10-06 the direction was checked by driving the
wheels, and the motor leads were placed to match the firmware (red → AO2/BO2,
black → AO1/BO1) rather than flipping the switch, so the documented wiring and
the default build agree. With that wiring `m +N` turns the wheels forward and
`d100` is positive. If a future motor or wiring change turns the wheels the
wrong way, either swap the motor leads back to this convention or set
`MOTOR_DIRECTION_INVERTED` to 1, and update this section to match.

### Output-stage guards (`tb6612_plan.c`)

A second layer behind the state machine, which does not trust what it is asked
for. Every ms, `TB6612_Apply` reads the IN1/IN2 outputs, the compare register
and the timer period, and `TB6612_Plan` decides:

| Guard | Fires when | This ms | Then |
|-------|------------|---------|------|
| Period | TIM1 ARR + 1 ≠ `MOTOR_PWM_PERIOD_TICKS` (1600) | driver disabled | FAULT |
| Cap | a duty above `MOTOR_DUTY_CAP_TICKS` (480) is asked for | clamped to 480 | FAULT |
| Direction | the IN pins would change while the compare register is not 0 | coast: IN1 = IN2 = low, compare 0 | FAULT |
| Direction | the IN pins change (compare already 0) but a non-zero duty is asked for in that same ms | new pins, compare 0 (the duty is not applied) | FAULT |

Why the direction guard: TIM1 has compare preload on, so a new compare value
only takes effect at the next PWM period (up to 50 µs), but the IN pins change
at once. Without the guard, an old non-zero duty could drive the bridge in the
new direction for up to 50 µs (a plug-brake pulse). IN1 = IN2 = low is OFF
whatever PWM does (TB6612FNG datasheet, H-SW control function table), so it is
safe at once. It can't coast forever: it fires only on a non-zero compare and
always writes 0. If the compare is already 0 (written on an earlier ms, so
already active), the pins change with duty 0, as before. The state machine
always asks for duty 0 on the ms the pins change (the wake-up ms and the end
of the reverse dwell); asking for more there would mean the duty lands on the
next ms with no ramp, so that is a direction fault too.

None of these fire on a working state machine: `tests/test_tb6612_plan.c` runs
every normal sequence plus a random hour and checks that the new output stage
writes exactly what the old one did, every ms, with no guard firing. So if one
fires, the state machine has a bug, and the motor is not driven on it again.

### FAULT (mstate=3)

The driver stays disabled until the board is reset (with fixed firmware);
commands are still received and counted (`cmd_ok`, `cmd_pct`), but nothing,
`m 0` and `v 0` included, re-arms it. Causes (`MotorController.fault_reason`,
reported as `mfault` since telemetry v5; codes in
[Speed control fields](#speed-control-fields-v5)):

- `MOTOR_FAULT_PERIOD_MISMATCH`: checked once at boot, right after
  `MX_TIM1_Init`. The duty cap is in timer ticks, so it means 30 % only with a
  1600-tick period; if ARR is changed in CubeMX (for example 3999 for 80 MHz, or
  a lower ARR for a higher PWM frequency) without `MOTOR_PWM_PERIOD_TICKS`, the
  motor never runs. A lower ARR would otherwise make 480 ticks *more* than 30 %.
- `MOTOR_FAULT_CAP_GUARD`, `MOTOR_FAULT_DIRECTION_GUARD`: an output-stage guard
  fired (table above).
- `MOTOR_FAULT_LOW_BATTERY` (`bstate=3`), `MOTOR_FAULT_BATTERY_ADC`
  (`bstate=4`, also from boot), `MOTOR_FAULT_BATTERY_REMOVED_WHILE_DRIVING`
  (`bstate=5`): the battery monitor locked the motor out (see
  [Battery](#battery)). These also show as `bstate`.
- `MOTOR_FAULT_REPEATED_STALL` (`mfault=7`): three stalls, of either kind,
  within 60 s (see [Stall protection](#stall-protection)).

Signs: `mstate=3` and `duty=0` in every telemetry line, and the green LED
blinking fast (100 ms instead of 500 ms). Encoder, IMU, steering and telemetry
keep running. The first cause is kept if several happen.

### Interrupt-shared variables

Received command bytes are the firmware's first variables of its own that an
interrupt writes and the main loop reads (the `-Og` switch was made when there
were none). USART2 on the L432 has no receive FIFO and a byte arrives every
86.8 µs, so each byte is received by interrupt (`HAL_UART_Receive_IT`, one byte
at a time) into a 64-byte queue that the loop drains every 1 ms. The rules, in
`byte_ring.h` and `main.c`: every shared variable has exactly **one writer**,
is **`volatile`** (so the optimizer can't keep a stale copy in a register), and
is a 32-bit aligned value or a byte (so a read can never see half an update). A
UART error or a full queue makes the main loop reject the line it is assembling
when it notices the error (usually the damaged byte's own line; see the open
item on error attribution); reception is restarted by the interrupt, and by the
loop as a fallback. In binary mode the same error drops the frame being
received instead (it fails anyway on its CRC), so the attribution problem
doesn't apply to the Pi link.

The Pi link's transmit side adds a second shared structure: `link_tx_queue`
(`tx_queue.h`, four 128-byte frame slots). The main loop pushes whole frames;
`HAL_UART_TxCpltCallback` (USER CODE 4) pops a frame only after its last byte
has left and starts the next one. The "a queued frame is on the wire" flag
(`link_tx_queue_sending`) has two writers, but the main loop writes it only
with interrupts masked (`LinkTx_Kick`), so the two can't interleave. HAL sets
`gState` back to READY before it calls the callback
(`stm32l4xx_hal_uart.c`, `UART_EndTransmit_IT`), and
`HAL_UART_Transmit_IT` takes no lock, so starting the next frame from the
callback is safe.

### Safety

> **Never flash or debug with the LiPo connected.** Flash and end the debug
> session first, then connect the battery (see
> [Connecting the battery](#connecting-the-battery-every-time)). Disconnect it
> again before the next flash or debug session. The LiPo stays plugged in only
> while testing.

- **Reverse polarity: keyed connector, no electrical protection.** The TB6612
  has none. On 2026-10-06 one TB6612 board was destroyed when the LiPo leads
  were connected reversed by hand on the breadboard. The Nucleo was checked
  afterwards and is undamaged: 3V3 ↔ GND not shorted, normal LED and telemetry
  on USB, encoder and IMU OK, and all four motor pins switching correctly.
  Since then the pack connects through a **keyed connector** that can't be
  mated the wrong way round.
- **No fuse yet.** Required before floor driving (see [Open items](#open-items)).
- **Low-voltage cutoff** (see [Battery](#battery)): the MCU locks the motor
  out below 6.6 V, on its own, without the Pi.
- **Stall protection** (see [Stall protection](#stall-protection)): a locked
  wheel (500 ms) or, under `v`, a wheel held at the cap below half its
  setpoint (1 s) latches the motor; three within 60 s are a reset-only FAULT.
  Until the stall current is measured, this and the 30 % cap are what bound
  it.
- **STBY pull-down** (10 kΩ, PB1 to GND): the driver stays disabled while the
  MCU isn't driving STBY (reset, flashing).
- **Fault handlers** (`Error_Handler`, HardFault, MemManage, BusFault,
  UsageFault, NMI) call `Board_EmergencyStop()` first, using register writes
  only. Motor: STBY low, IN pins low, PWM output off. Without it TIM1 would
  keep making PWM while the CPU spins in the handler. Servo: TIM15 compare 0
  and nothing else, so the pulse in progress finishes and no pulse follows
  (see [Steering servo](#steering-servo-tim15)). The handlers then spin until
  the IWDG resets the MCU (~250 ms), which boots with the motor latched and
  the servo without pulses.
- **Debugging: never run a debug session with the LiPo plugged in.** At a
  breakpoint the CPU stops, so the command watchdog stops too. As a second
  layer TIM1 is frozen on a debug halt (`__HAL_DBGMCU_FREEZE_TIM1`), which
  disables its output like clearing MOE: PA8 floats and the TB6612's input
  pull-down makes PWM low, so the motor short-brakes instead of running on.
  TIM15 is frozen the same way (no servo pulses), and so is the IWDG, so a
  breakpoint doesn't reset the board.
- **Hardware watchdog (IWDG):** a hung main loop resets the MCU within
  ~250 ms (see [Hardware watchdog](#hardware-watchdog-iwdg)).
- **Output-stage guards and FAULT** (above): the 30 % cap, the "no drive while
  the direction pins change" rule and the PWM period are checked again at the
  pins, independently of the state machine; any of them firing latches FAULT.
- **Bench first:** wheels off the ground; plug the LiPo in only after flashing
  and with the debug session ended.
- **Motor console in a classic console window:** it turns QuickEdit off for its
  own window, because selecting text there freezes the console (and with it the
  keepalives and the Esc/Ctrl+C keys). See `tools/motor_console.py` and
  [Motor console](#motor-console-and-quickedit) below.

### Measurement notes (multimeter)

The bench meter is now an **AstroAI DM6000AR** (6000 counts, true RMS, auto
and manual ranging, Hz and duty cycle). Probes: **black on COM, red on the
V/Ω/Hz input**, black lead on GND unless noted.

- **It reads millivolts directly**, so low pins and PWM averages are read from
  GND as they are (for example the servo pin at centre read 246 mV). The old
  TM-510's ~1 V display floor no longer applies.
- **PWM is read directly with Hz / Duty**: servo (PA3, A2) and motor PWM (PA8,
  D9). The servo at centre read **50.32 Hz and 7.5 %** (1500 µs of 20 ms).
  Duty resolution is 0.1 %, which is 20 µs of servo pulse: enough to tell
  the endpoints from the centre, not the 3 µs steps of single percents. How
  well the duty reading holds up at the motor's 20 kHz is unverified.
- **Pulse width from DC mV:** average = rail × pulse / 20 ms. Measure the 3V3
  rail first (246 mV at 1500 µs means a rail of about 3.28 V).
- **Fallback only** (a meter that can't show low voltages, like the TM-510):
  measure from the 3.3 V rail (red on 3V3, black on the pin); it reads the rail
  minus the pin's average.
- Low resistances (the motor winding, which the TM-510 couldn't resolve below
  ~50 Ω) haven't been tried with the new meter yet.
- Continuity checks only with everything unpowered. If a beep is surprising,
  probe in both directions.
- **Record each pin's reading separately**; don't log a group pass/fail. (On
  2026-10-06 a grouped "all read 3.3" missed the D2 short.)

### Bench results (2026-10-06)

Wheels off the ground, 2S LiPo at about 8.0 V. Speeds use
`meters_per_encoder_tick` = 3.9336e-5 from `config/vehicle.cfg`; counts are
`d100` (per 100 ms).

| Test | Result |
|------|--------|
| 10 % | turns (rough at low duty), about 150–185 counts ≈ 0.07 m/s |
| 20 % | +245…+343 counts forward, −256…−344 reverse ≈ 0.12 m/s, symmetric. The ±15 % swing within a run is once-per-revolution unevenness |
| Reverse sequence | ramp → 100 ms dwell → switch → ramp matched the design to about 1 ms in telemetry |
| Watchdog under load | console hard-killed while the motor was spinning → trip and latch (`wd_trip=1`, `mstate=2`) |
| Temperature | motor and driver cool to the touch in all runs |

## Speed control (`v`)

`v <units>` holds the wheel speed in closed loop. The PI controller writes the
motor's target **through** the motor state machine
(`MotorController_SetClosedLoopTarget`), never around it, so the 30 % cap,
the ramp, the reverse dwell, the command watchdog, the latch, FAULT and the
output-stage guards all apply exactly as for `m`.
`tools/test_firmware_sources.py` checks that the speed controller and the
stall check reach the motor only through its API.

**The Pi uses closed loop only**, through the Pi link's DRIVE frame (speed in
counts/s; see [Pi link](#pi-link-binary-protocol-v1)), which sets the same
speed controller as `v`. `m` (open loop, duty) stays as a text-mode bench and
diagnostic command; the last of `m` / `v` wins, and both feed the motor
watchdog.

Code (pure C, host-tested): `Core/Src/speed_control.c` (the PI),
`Core/Src/speed_estimator.c` (the 20 ms estimate), `Core/Src/stall_detect.c`
(both stall rules), `Core/Src/angle_correction.c` (the encoder's H4
correction).

### Units

**1 speed unit = 1 count per 20 ms = 50 counts/s** (the estimator's window, so
there is no conversion and no rounding in the firmware). The firmware knows
nothing about the wheel size; the Pi converts:

```
units = speed_m_per_s / (50 × meters_per_encoder_tick)
```

With `meters_per_encoder_tick` = 3.9336e-5 (`config/vehicle.cfg`), 1 unit ≈
1.97 mm/s and `v 100` ≈ 0.197 m/s. The grammar allows −100..100. **On the Pi
link the speed is in counts/s** (±5,000, the same envelope): the firmware
converts with |cps| < 1000 → 0 (on the raw counts/s), else round to nearest,
ties toward zero (`LinkProtocol_CpsToUnits`), so DRIVE 999 coasts and
1000–1025 are 20 units. HELLO_ACK sends 1000 / 5000 / 50 so the Pi doesn't
hard-code them. On the lift,
the 30 % cap reaches about **93 units** (≈ 0.18 m/s) at ~7.9 V, so anything
above that saturates there (and on the floor the reachable speed will be
lower; see the floor note under [Stall protection](#stall-protection)).
`meters_per_encoder_tick` was calibrated on raw counts over ~6.2 turns; the
angle correction leaves whole-turn distances unchanged (worst endpoint effect
2 × 44 counts = 0.35 %).

### Controller

| Rule | Value | Constant (`Core/Inc/speed_control.h`) |
|------|-------|-----------|
| Period | PI every 10 ms, on its own due-time schedule (not `%`), integer Q8, no D | `SPEED_CONTROL_PERIOD_MS` |
| Gains | Kp = 164/256 = 0.641 ticks per unit; Ki = 55/256 = 0.214 ticks per unit per 10 ms | `SPEED_KP_Q8`, `SPEED_KI_Q8` |
| Output clamp | to the setpoint's sign: [0, +480] for `v > 0`, [−480, 0] for `v < 0`. It never drives the other way to slow down; a lower duty already brakes (slow decay) | |
| Minimum setpoint | \|v\| < 20 counts as 0 (below the lift's breakaway, ~21–23 units) | `SPEED_MIN_SETPOINT_UNITS` |
| `v 0` | coast, like `m 0`: target 0, the ramp-down brakes, then the driver is off | |
| Zero-speed gate | any start and any sign change: target 0 until \|estimate\| ≤ 2 units, or the wheel already turns the new way | `SPEED_GATE_UNITS` |
| Preload | a start from rest begins the integrator at the breakaway duty, 160 ticks | `SPEED_PRELOAD_TICKS` |
| Invalid estimate | target held and integrator frozen for up to 50 ms; still invalid → target 0, latch (`m 0` / `v 0`), `enc_bad`+1 | `SPEED_INVALID_HOLD_MS` |

**Gains (lambda tuning)** for a first-order plant with dead time,
K e^(−θs) / (τs + 1): Kp = τ / (K (λ + θ)), Ki = 1 / (K (λ + θ)), with

- K = 11.7 counts/s per tick on the lift = 0.234 units per tick,
- τ = 30 ms (the top of the 0–30 ms range the step captures allow; a smaller
  real τ only adds margin),
- θ = 50 ms (the loop's own ~18 ms: estimator window 10, controller period 5,
  sensor and read 3, plus margin),
- λ = 150 ms (the closed-loop time constant).

So Ki = 1 / (0.234 × 200 ms) = 0.0214 per ms = 0.214 per 10 ms → Q8 **55**, and
Kp = 30 × 0.0214 = 0.641 → Q8 **164**. The crossover is ~5 rad/s (0.8 Hz),
where even 48 ms of delay costs only ~14° of phase. `tests/test_speed_control.c`
pins both numbers and recomputes them from this derivation.

**Anti-windup and integration:**

- The integrator is clamped to the same sign range as the output.
- It integrates only up to the point where P + I reaches the output limit in
  the error's direction (so a saturated output sits exactly at the cap), and
  never moves against the error.
- It doesn't integrate while `motor_control` is still ramping toward the
  target, unless the motor is RUNNING, or on an invalid estimate.
- It resets on `v 0`, a setpoint sign change, any `m`, LATCHED and FAULT. An
  `m` → `v` switch starts it at the duty applied at that moment (bumpless).

**Preload, for and against.** For: from rest the wheel doesn't move until
~156–188 ticks (lift breakaway); without the preload the integrator climbs
there at ~4 ticks per 10 ms (`v 20`), ~0.4 s of nothing, then overshoots as
the static friction lets go (simulated: 342 ms to settle without, 132 ms
with). Against: if the real breakaway is lower, it can overshoot a low
setpoint briefly (bench: `v 20` peaks at 26 for one line); it is only 10 %
duty and the PI pulls it back within ~0.2 s. On the floor the breakaway is
probably higher.

### Speed estimate and the odometer

`vest` = corrected count now minus 20 ms ago (`speed_estimator.c`), updated
every ms right after the encoder read. Resolution 1 unit; delay about 10 ms
(half the window) plus the sensor's filter. It is **valid** only when all 21
samples of the window were good: none of them a failed read, a jump of more
than 20 counts in 1 ms (four times what the cap reaches: a glitch), MD = 0 at
the last health read, or an update not exactly 1 ms after the previous one.
One bad update keeps it invalid for the next 21 ms. ML and AGC are not inputs.
With the 50 ms hold, a read outage of up to ~29 ms is ridden through under
`v`; a longer one latches.

**Odometer caveat.** The unwrapper assumes less than half a revolution (2048
counts) between good reads. If the reads fail (or the angle freezes) for
longer than that at speed, about **0.44 s at the cap (~93 units)** and longer
at lower speeds, the count can come out **one revolution (4096 counts) short
or long**, and stays so. Seen on the bench: the `e 3` test under `v 50`
(2026-10-15) froze the angle while the wheel turned ~2,570 counts, and the
unwrapper logged −1,513. The estimate flags it (`vok=0` on that line), and
under `v` the motor latches long before that. **The Pi should treat any
`vok=0` stretch as "distance uncertain"** and not trust the odometer across
it. On the Pi link this is STATE's `odometer_epoch`: it goes up by 1 at every
valid → invalid change of the estimate, so **a changed epoch between two
STATE frames means "distance uncertain across them"** (and `ENC_INVALID_IN_PERIOD`
says the same per frame). Re-measured over the link on 2026-10-16
(`b3_encfault_2`, `e 3` under DRIVE 2500): the count jumped −1,384 at the
unfreeze while the wheel was turning forward, i.e. it travelled about 2,712
counts during the freeze and the count came out **one revolution short**;
the epoch went up exactly there. A frozen sensor only shows at the unfreeze:
while the angle is frozen the estimate is a *valid* 0.

### Stall protection

Two rules in `stall_detect.c`, checked every ms before the motor state
machine. Either one stops the motor at once (driver off), latches it until
`m 0` / `v 0` (`mstate=2`; on the Pi link: DRIVE 0 with the arm bit), and
adds 1 to `stall_n`.

| Rule | Mode | Condition, without a break | Time | Constants (`Core/Inc/stall_detect.h`) |
|---|---|---|---|---|
| Locked wheel | `m` and `v` | RUNNING, \|duty\| ≥ 128 ticks (8 %), estimate valid, \|vest\| < 2 units | 500 ms | `STALL_MIN_DUTY_TICKS`, `STALL_MAX_SPEED_UNITS`, `STALL_HOLD_MS` |
| Saturation | `v` only | RUNNING, duty **at the cap (480)** in the setpoint's direction, estimate valid, speed **in the setpoint's direction** below **½** of the effective setpoint | 1000 ms | `STALL_SAT_HOLD_MS`, `STALL_SAT_SPEED_NUMERATOR` / `_DENOMINATOR` |
| Repeated | both | the 3rd stall (either kind) within 60 s of the first of the three | – | `STALL_REPEAT_COUNT`, `STALL_REPEAT_WINDOW_MS` |

The repeated rule → **FAULT, `mfault=7` (REPEATED STALL), reset only**: the one
thermal risk the firmware can bound while the stall current is unknown (the
winding resistance isn't measured yet).

- **Locked wheel.** An invalid estimate means "can't tell", not "stopped": the
  condition is false and the 500 ms start again. A sensor that reads fine but
  is frozen looks exactly like a stall and is treated as one (the safe way
  round; that is what the `e 3` test uses).
- **Saturation**, why: a held wheel that slips slowly (bench, 2026-10-15: a
  gloved hold at `v 20` crept at 3–9 units) never stays below 2 units, so the
  first rule's 500 ms never complete, while the PI winds the duty up to the
  cap and keeps it there. The 1000 ms restart when the duty leaves the cap,
  the speed reaches half the setpoint, the estimate goes invalid, on `v 0` or
  a setpoint below the minimum, on any `m`, and on a sign change (the duty
  can't be at the new sign's cap without ramping through 0). A setpoint
  magnitude change of the same sign does **not** restart it. "In the
  setpoint's direction" means a wheel pushed backwards while at the cap
  counts as below half, so a push the wrong way latches too. A normal step
  reaches the cap only briefly: simulated with a vehicle-like τ of 400 ms, a
  `v 80` step spent up to 827 ms at the cap but the rule counted at most
  127 ms of it (the speed passes half the setpoint early).
- **Floor note, by design:** a setpoint the wheel can't reach at the 30 % cap
  (for example `v 100` on a slope, or against an obstacle it slips on) leaves
  the duty at the cap below half the setpoint, and **latches after 1 s like a
  stall**. On the lift the cap gives ~93 units, so any `v` up to 100 stays
  above half. **The Pi must command reachable speeds**, and re-arm (`v 0`) on
  purpose after a latch; three within 60 s are a reset-only FAULT.
- **`m` mode** has only the locked-wheel rule: a wheel slipping slowly at full
  duty under `m` isn't caught (bench/diagnostic only, on purpose).
- **Which rule fired** is in the Pi link's STATUS frame: `latch_reason`
  (STALL_LOCKED or STALL_SATURATION) and two counters, `locked_stalls` and
  `saturation_stalls`, which always add up to `stall_n`
  ([Latch reasons](#latch-reasons-and-stall-counters)). The text line (v5,
  frozen) still has only `stall_n`; in a text log: the saturation rule leaves
  duty 480 and `vest` ≥ 2 in the second before the latch; the locked-wheel
  rule leaves `vest` 0 for the last 0.5 s.

### Encoder angle correction (H4)

**Why.** The wheel angle as read carries an error that repeats **4 times per
revolution**: about ±44 counts of angle, which looks like **±24 % speed
ripple**. It is the same at every speed and in both directions, and its
phase is tied to the sensor's own angle frame (the speed-ripple peaks fall on
raw ≈ 0, 1024, 2048, 3072; the angle error itself peaks a quarter period
later, at raw ≈ 265, 1289, 2313, 3337). That points to the sensor/magnet
geometry rather than the wheel; **likely, not proven**. Uncorrected, it put
14–19 % of noise on `d100` and ~20 % on the 20 ms estimate, straight into the
PI.

**Model and correction** (`Core/Inc/angle_correction.h`), in counts:

```
eps(raw)  = A × sin(2π (raw + r0) / 1024)
corrected = (raw − eps(raw)) mod 4096
```

| Constant | Value |
|---|---|
| `ANGLE_H4_AMPLITUDE_Q4` | 703 (A = 43.94 counts, in 1/16 counts) |
| `ANGLE_H4_PHASE_COUNTS` | −9 (r0) |

Fitted with `tools/fit_angle_correction.py` from `cap_m30_c2_1.log`
(2026-10-15, steady 30 % duty, 6.4 revolutions; the other captures of that
session agree within ±5 % amplitude and ±5° phase). Integer only: a 257-entry
quarter-wave sine table (Q15), the same on the host and the target, bit for
bit. Only H4 is removed: the once-per-turn part is partly a real mechanical
tight spot (it shrinks with speed), and H2/H3 aren't the same from capture to
capture. The correction's slope is at most 0.27 count per count, so the
corrected angle never runs backwards, and |eps| ≤ 44 is far inside the
unwrapper's half-turn rule.

**What uses it:** the corrected angle feeds its own unwrapper, which gives
`count`, `d100` and the speed estimate. `raw` in the telemetry and the capture
build stay **uncorrected**.

**Tied to the board's seating.** The encoder board sits on its pegs and is
**not glued** (the owner's decision), so the constants belong to its current
seating. `tools/test_firmware_sources.py` and `tests/test_angle_correction.c`
pin them, so a change is always deliberate. **Diagnostic: if `vest` wobbles
4 times per wheel turn again, the board has moved**; also re-fit if `ML` or
the magnitude at rest change.

**Re-fit**, on the lift:

1. Capture build: `SPEED_CAPTURE 1` (see [Capture build](#capture-build)),
   flash with the LiPo disconnected, then connect it.
2. Console, a new log: `m 0`, `c 2`, `m 30`. Hold it until the capture has
   finished and been sent (3000 samples at 2 ms = 6 s, then ~15 s of `cap`
   lines), then `m 0`. Repeat at `m 20` and `m -20` into further logs.
3. `python3 tools/fit_angle_correction.py /mnt/c/dev/rover-logs/cap_*.log`
   (WSL). It warns if H4 differs between captures (the board moved during the
   session, or a bad capture), takes the constants from the capture with the
   most steady revolutions (or `--from <log>`), and predicts the residual.
4. Put the new values in `angle_correction.h` (and its comment: log, date),
   update both pins, set `SPEED_CAPTURE` back to 0, rebuild, flash, and check
   `d100` std and `vest` at `m 30`.

### Bench results (2026-10-15, on the lift)

**Identification** (capture build, morning): static map **11.7 counts/s per
tick above a 66-tick dead zone**, breakaway from rest at 156–188 ticks, τ in
0–30 ms. The same afternoon's baseline logs ran **~6–8 % slower** at the same
duty (`m 30` 92.6 vs 98.4 units, `m 20` 53.1 vs 57.7; a two-point fit gives
12.3 counts/s per tick above ~105 ticks). The local gain near the operating
points (~0.247 units per tick) is still close to the 0.234 the gains were
designed with.

**Angle correction** (30 s steady, 100 ms windows):

| | `d100` std before → after | `vest` std after |
|---|---|---|
| `m 30` (~93 units) | 14.4 % → **4.1 %** | 6.35 units |
| `m 20` (~54 units) | 19.0 % → **6.2 %** | 4.25 units |

The mean of `vest` matched `d100` / 5 within 0.1 units.

**Closed loop** (`d_v_1`, `d_drag_1`, `d_stall_1`, `d_wd_1`, `s2_*`):

| Test | Result |
|---|---|
| `v 50` from rest | at 50 (100 ms average) in ~0.5 s; duty 301 = `vi` |
| `v 20` | at 20 within 0.1–0.2 s; duty 154 |
| `v 80` | at ~80 in ~0.8 s; duty 421 |
| `v 100` | saturates: duty 479, `vest` 93.4, `vi` 477; hands off for 10.6 s: no latch |
| `v 100` → `v 50` | duty off the cap in the first line; within ±5 in ~0.4–0.6 s |
| `v 50` → `v −50` | gate ≤ 0.2 s, reverse drive started with the wheel at 0, −50 in 0.4 s |
| Stops (`v 0`, `m 0`, gate) | 0.1–0.2 s: the ramp-down brakes; even with the driver off (a watchdog trip) the wheel stopped within ~70 ms. **Much faster than the 400 ms coast the first plant model assumed** |
| Overshoot | +3 to +4 units on the 100 ms average; single 20 ms readings run up to 17 units over from the remaining ripple. Judge overshoot by `d100` |
| Gloved drag at `v 50` | held ≥ 43 units while the duty rose 285 → 436; on release 71 units (100 ms), back within ~0.4 s. The release overshoot scales with the drag |
| Locked wheel, `m 10` | latched 0.5 s after the wheel stopped |
| Locked wheel at `v 100` (clamped) | latched ~0.6 s after the lock (locked-wheel rule) |
| Slipping hold, `v 20` (sitting 1) | crept at 3–9 units: **no latch**, duty to 480 (the gap the saturation rule closes) |
| Slipping hold, `v 50` | slipping at 4–24 units at the cap: **saturation rule** latched ~1.0–1.1 s after the slip settled below 25 |
| Watchdog | `-v` with `s` keepalives flowing: trip within 500 ms, `mstate=2` |

**Encoder faults** (`ENCODER_FAULT_TEST` build, see
[Encoder fault test build](#encoder-fault-test-build)):

| Test | Result |
|---|---|
| `e 1` (20 ms outage) under `v 50` | `vok=0` on one line, no latch, `enc_bad` 0, stayed at ~50 |
| `e 2` (100 ms) under `v 50` | latched, `enc_bad` 1, `stall_n` unchanged, `i2c_err` 0; wheel stopped within 0.2 s |
| `e 2` under `m 30` | no latch, `vok=0` on one line |
| `e 3` (angle frozen 700 ms) under `m 20` | stall latch ~0.52 s after the freeze, `stall_n`+1 |
| `e 3` under `v 50` | duty to 480 (the PI saw 0), latch ~0.52–0.62 s; `count` came out one revolution off (see the odometer caveat) |
| `e 3` three times within 30 s | latch, latch, then `mstate=3`, `mfault=7`; `m 0` doesn't clear it |

**Loop timing** (profiler build, `s2_profile_1`, `v 50` and idle): the worst
iteration is **report step 1**, median 676 µs, worst 728 µs: encoder reads
282, IMU 120, line formatting 228–266 µs. The health iteration is next (median
665, worst 699; STATUS + AGC reads 230). Speed control + stall check take
**8 µs** (worst 19) and the angle correction + estimate **12 µs** (worst 19),
inside a motor section of 47 µs (worst 81). Across all the 2026-10-15 logs,
`loop_us_max` stayed at 660–735 µs and `i2c_err` at 0.

## Steering servo (TIM15)

The stock Axial AS-1 micro servo (AXI31619, the SCX24's own; no electrical
datasheet found) steers the truck. Signal on **PA3 (header A2) = TIM15_CH2,
AF14** (ST's pin data for STM32L432K(B-C)Ux; PA3 is the only TIM15_CH2 pin on
this package). Servo power: LM2596 buck #2 (5.57 V unloaded, 1000 µF on its
output), fed from the 2S LiPo via the VM rail; grounds common with the Nucleo
(wiring: [Carrier hardware](#steering-servo-power-buck-2)). **Never connect
the buck's output to the Nucleo's 3V3 or 5V pins.** The signal is 3.3 V logic, which the AS-1 accepts
(bench, 2026-10-09): no level shifter needed.

| Setting | Value | Why |
|---------|-------|-----|
| Timer | TIM15, 32 MHz / (PSC 31 + 1) = **1 MHz tick**, ARR 19999 → **20 ms** (50 Hz) | Standard RC servo frame; the compare value is the pulse width in µs |
| Mode | PWM mode 1, polarity high, compare preload on | A new pulse width starts at the next period, so a pulse is never cut short or stretched |
| Limits | `SERVO_PULSE_MIN_US` 1200, `SERVO_PULSE_CENTER_US` 1500, `SERVO_PULSE_MAX_US` 1800 (`Core/Inc/steering_control.h`) | **Calibrated 2026-10-10** (see [Steering calibration](#steering-calibration)): `s ±100` is full usable lock, 50 µs inside the clean locks. A compile-time check keeps 1000 ≤ MIN < CENTER < MAX ≤ 2000; `tools/test_firmware_sources.py` pins the three values |
| Mapping | `s` percent → µs, each half scaled on its own: +100 → MAX, 0 → CENTER, −100 → MIN (3 µs per percent with these limits) | Centre and both endpoints are exact even when the calibrated halves differ |
| Direction | `STEERING_DIRECTION_INVERTED` in `main.c`, **0**: `s +N` (left) gives a longer pulse | Confirmed on the bench: `s 100` steers left (seen from behind), also at full lock |

Code: `Core/Src/steering_control.c` (state machine, mapping and output guard,
pure C, host-tested) and `Core/Src/servo.c` (reads the timer's real tick and
period, runs the guard, writes the compare register; no decisions).

### Behavior

| Situation | Steering |
|-----------|----------|
| Boot | PWM runs with compare 0: the line is driven low, **no pulses**, until the first valid `s`. The servo never moves unless commanded, not on a reset either |
| `s` commands | follows the last one (`sstate=1`) |
| Link loss | **its own timeout**: no valid `s` for 500 ms (`STEERING_COMMAND_TIMEOUT_MS`) → **centre** (`sstate=2`, `s_wd`+1) until the next `s`. Fed only by `s`; independent of the motor's watchdog. Before the first `s` nothing times out (still no pulses) |
| Motor watchdog trip or motor FAULT | unaffected: the truck can still be steered while it coasts |
| Battery cutoff, ADC fault or removed while driving | unaffected (only the motor is locked out); buck #2 keeps drawing from the pack |
| Output guard | every ms `Steering_PlanOutput` refuses a timer that isn't 1 MHz / 20 ms and clamps a pulse outside [MIN, MAX]; either one locks the steering in **FAULT** (`sstate=3`, no pulses until reset). Also checked once at boot |
| Emergency stop (fault handlers) | TIM15 compare 0 **only**: the pulse in progress ends normally, none follows. Clearing MOE or re-muxing PA3 would cut the pulse short, and a short pulse is itself a command toward one end or beyond |
| Debug halt | TIM15 frozen: output disabled, no pulses |
| IWDG reset | the pin resets mid-frame, so one pulse may be cut short (unavoidable); after boot, no pulses until `s` |

"No pulses" means the servo stops driving if it is the analog kind; some
digital servos hold their last position instead. The AS-1 goes **limp** with no
pulses (bench, 2026-10-09); with pulses it holds its position and springs back
when the wheels are pushed.

**No slew limit**: the servo's own transit speed already limits how fast it
moves, and the endpoints are the safety limit; a firmware slew limit would only
add steering lag.

**Calibration lives where it's needed.** The firmware owns the safety limits
and the centre, because it must enforce them whatever the Pi sends and must
know "straight" to centre on link loss without the Pi. The steering-angle ↔
percent mapping belongs to the Pi (`config/vehicle.cfg`, next to
`max_steering_angle_radians`). If MIN, MAX or CENTER change, the Pi's mapping
must be re-checked; `s_us` in the telemetry shows the pulse actually applied.

### Steering calibration

**Done 2026-10-10**, wheels off the ground, servo horn on its linkage:

| | Value | How it was set |
|---|---|---|
| `SERVO_PULSE_CENTER_US` | **1500** | `s 0` looked dead straight, sighted from behind. `s ±5` (1510 / 1490 µs) gave no visible movement, so the servo's deadband is at least ±10 µs and a finer centre can't be seen; it stays 1500 |
| `SERVO_PULSE_MAX_US` (left) | **1800** | 50 µs inside 1850, a clean lock (below) |
| `SERVO_PULSE_MIN_US` (right) | **1200** | 50 µs inside 1150, a clean lock (below) |
| `STEERING_DIRECTION_INVERTED` | **0** | `s 100` steers left (seen from behind), confirmed at full lock |

**`s ±100` is now full usable lock**, not a placeholder: the Pi can command the
whole range.

**Method actually used.** Instead of the 25 µs steps below, one temporary
calibration build with MIN 1150 / MAX 1850 (never committed):

1. No LiPo: `s 100 / 50 / -100 / 0` read `us=` 1850 / 1675 / 1150 / 1500 on
   the console's status line.
2. Under power: both locks at 1850 and 1150 looked clean, with no visible
   linkage flex.
3. Final limits set **50 µs inside** each of those, as margin against slop in
   the linkage and flex under load: a lock that is just clean with the wheels
   off the ground may load the linkage with the truck's weight on them.

**Risk it accepted:** while the wide build was flashed, its firmware limits
were 1150 / 1850, so a mistyped `s ±100` went straight to the widest point in
one step. Had that been past a mechanical stop, the servo would have driven
into it and stalled there (heat, current) until the next `s` or Esc. The 25 µs
stepping below exists to avoid exactly that; this time it was skipped, and the
locks turned out clean.

**Final build check** (1200 / 1500 / 1800): no LiPo, `s 100 / 50 / -100 / 0`
read `us=` 1800 / 1650 / 1200 / 1500. Under power, silent while holding at
both locks; servo case cool.

`max_steering_angle_radians` in `config/vehicle.cfg` was re-measured at these
limits on 2026-10-22 (`s ±100`, turning circles, 6 runs): **0.204** rad, the
smaller side mean (left 0.204, right 0.207; replaces the stale 0.246 from the
stock radio's endpoints). The runs are in the cfg's comment.

#### Recalibrating

After a servo, horn or linkage change. **Every powered step uses the
[battery procedure](#connecting-the-battery-every-time), and every change is a
reflash with the LiPo disconnected.** There is deliberately no runtime command
to widen the limits: that would be a path around the very limits being set.

1. **Centre:** `s 0`. Check the wheels are straight (by eye or a straight edge
   along the wheels). Adjust `SERVO_PULSE_CENTER_US` in 10 µs steps until they
   are.
2. **Direction:** `s 100` must steer **left**. If it steers right, set
   `STEERING_DIRECTION_INVERTED` to 1.
3. **Widen one side at a time** by **+25 µs** per step (MAX up, or MIN down):
   reflash, command full lock on that side (`s 100` or `s -100`) and hold 2 s.
   **Stop** at the first step where the servo buzzes at rest, the knuckle
   touches its stop, or the linkage visibly flexes. Set the final value
   **50 µs back** from that step.
4. Repeat for the other side.
5. Record MIN, CENTER and MAX here and in the commit message, update the pin in
   `tools/test_firmware_sources.py`, and re-measure the steering angle in
   `config/vehicle.cfg`.

The meter checks each value without the servo moving: Hz / Duty on A2 reads
duty = pulse / 20 ms (1200 / 1500 / 1800 µs = 6.0 / 7.5 / 9.0 %).

### Bench checks (no LiPo, meter only)

With `motor_console.py` (the servo needn't be connected):

1. **Boot:** `s_us=0 sstate=0`; A2 reads about 0 mV, no frequency.
2. **`s 100`, `s 0`, `s -100`:** duty 9.0 / 7.5 / 6.0 % (`STEERING_DIRECTION_INVERTED 0`).
3. **`-s`** (stop the steering keepalive): within 0.5 s `sstate=2`, `s_wd`+1,
   `s_us=1500`, duty 7.5 %.
4. **`s` can't keep the motor alive:** `m 0`, `s 50`, `m 20`, then `-m`: within
   0.5 s `wd_trip`+1 and `mstate=2`, while `s_ok` keeps climbing and `sstate=1`.

Then the separate **"servo moves"** step, with the LiPo and buck #2: wire buck
OUT+ → servo red, OUT− → servo brown/black and Nucleo GND, servo signal → A2.
Unpowered checks first: buck OUT+ ↔ GND, ↔ 3V3 and ↔ Nucleo 5V all silent;
signal ↔ A2 and the ground ties beep. After power-on: the servo does **not**
move until `s 0` (then centre); `s ±50`, `s ±100` turn it with no buzzing at
the ends; `-s` centres it after ~0.5 s; Esc sends `m 0` and `s 0`. Note whether
it goes limp or holds when pulses stop, whether it gets warm, and whether the
buck output sags.

### Bench results, servo under power (2026-10-09)

The "servo moves" step, with the limits then in force (1300 / 1500 / 1700 µs):

| Check | Result |
|-------|--------|
| Power-on | no movement or twitch (no pulses at boot) |
| 3.3 V logic | the AS-1 responds: no level shifter needed |
| Direction | `s +N` = left, seen from behind |
| Pulses stop / resume | limp with no pulses; with pulses it holds and springs back when pushed |
| Link loss (`-s`) | back to centre after ~0.5 s and holds there (`us=1500 sst=2`) |
| Steering vs latched motor | steering commands never woke the latched motor (`mstate=2` throughout) |
| Servo rail | buck #2 holds 5.56 V; dips to about **4.45 V** briefly when the wheels are forced. The meter averages, so the real minimum may be lower. A 1000 µF capacitor has since been fitted at buck #2's output; the dip with it is not re-measured yet (see [Open items](#open-items)) |
| Temperature | servo and buck cool |

## Hardware watchdog (IWDG)

If the main loop ever hangs (as opposed to running slowly), the command
watchdog can't run either and the motor would keep its last duty. The IWDG
resets the MCU instead; after a reset the motor pins float (the TB6612's input
pull-downs keep the bridge off), the motor boots latched (`mstate=2`, needs
`m 0`), and the servo gets no pulses until `s`.

| | |
|---|---|
| Clock | LSI (32 kHz nominal), independent of the main clock |
| Setting | prescaler 32 (1 ms per count), reload 249 → **250 ms**; window off (4095). In CubeMX, with "Do Not Generate Function Call" ticked for `MX_IWDG_Init` |
| Real timeout | 235–271 ms with the LSI's spread (L4 datasheet LSI 29.5–34 kHz over the full range, taken from the L412 datasheet; the L432 figure is assumed the same, unverified) |
| Start | first thing in `USER CODE 2`, right after `__HAL_DBGMCU_FREEZE_IWDG()`, so it also covers the ~40 ms boot. **Once started it can't be stopped** except by a reset (`stm32l4xx_hal_iwdg.c`, "IWDG Generic features") |
| Refresh | **exactly one**, the last statement of the main loop: reached only if every part of the iteration returned, including the deadline busy-wait (which would spin forever if the ms tick stopped). Never in an interrupt: SysTick and USART2 keep interrupting a stuck main loop, so a refresh there would hide the hang |
| Debug | frozen at a halt (`DBG_IWDG_STOP`, `DBGMCU_APB1FZR1` bit 12), so breakpoints don't reset the board |

**Why 250 ms, not 50–100 ms.** The longest legitimate gap between refreshes is
set by the HAL's I2C timeouts, not by our own 2 ms: `HAL_I2C_Mem_Read` waits up
to 25 ms for a busy bus (`I2C_TIMEOUT_BUSY`) and up to 25 ms more for a STOP
after a NACK (`I2C_TIMEOUT_STOPF`). A stuck encoder bus therefore makes a normal
iteration ~52 ms long and the health iteration (4 reads) ~104 ms; a contrived
worst case is ~220 ms. (Telemetry and the heartbeat keep going in that state;
see [Loop scheduling](#loop-scheduling-due-times).) Normal iterations take ≤ 0.75 ms (worst 728 µs, profiler 2026-10-15), and the boot to the first
refresh takes ~40 ms (~90 ms if the boot line's UART send timed out). At 250 ms
none of these reset the MCU, so **an IWDG reset always means a real hang or a
fault handler**, never a known slow path. A hang leaves the motor at its last
duty (≤ 30 %) for at most ~0.27 s, about 5 cm. It also stays well under the
500 ms command watchdog.

**Fault handlers end in an IWDG reset.** `Error_Handler` and the fault handlers
stop the actuators and spin, so ~250 ms later the IWDG resets the MCU. The
motor's FAULT state (`mstate=3`) is different: the loop keeps running, so it is
**not** reset and still needs a manual reset.

### Reset cause (`rst`)

`RCC_CSR` reset flags, read once at boot and cleared (`RMVF`), reported in every
telemetry line (so a log opened after the boot still shows it). Bits: 0 FW,
1 OBL, 2 PIN, 3 BOR, 4 SFT, 5 **IWDG**, 6 WWDG, 7 LPWR (`stm32l432xx.h`,
`RCC_CSR_*RSTF`). Values confirmed on the hardware:

| `rst` | Bits | Cause |
|---|---|---|
| 4 | PIN | reset button, or the reset after flashing |
| 12 | BOR + PIN | USB power-up |
| 36 | IWDG + PIN | IWDG reset: a hung loop **or** a fault handler (they can't be told apart yet; see [Open items](#open-items)) |

`motor_console.py` decodes it on its status line (`rst=36 (IWDG+PIN)`). The boot
line `imu_init=... who=0x..` is unchanged, so reset detection in the tools is
unchanged.

### Hang test

`#define IWDG_HANG_TEST 0` in `Core/Inc/command_dispatch.h`. **Set to 1 only for
this test** (the build shows a `#warning`; the host tests fail if 1 is ever
committed). In a hang-test build, `h <n>` stalls the main loop for n × 10 ms
with interrupts on; the motor and servo keep their last outputs, as in a real
hang. In a normal build `h` doesn't exist and is rejected (`cmd_bad`).

No LiPo, `motor_console.py` running:

1. `m 0`, `m 20` (STBY high, PWM on D9).
2. `!h 20` (200 ms): **no reset**; `resync`+1.
3. `!h 30` (300 ms): **reset** within ~0.3 s: a boot line, then `rst=36`,
   `mstate=2`, STBY low.
4. F5, stop at a breakpoint for ≥ 2 s, continue: **no reset** (debug freeze).
5. Set `IWDG_HANG_TEST` back to 0, rebuild, flash: `!h 30` → `cmd_bad`+1.

## Pi link (binary protocol v1)

The Pi talks to the board over the same VCP (115200 8N1) with a framed binary
protocol: COBS framing, a CRC-16 on every frame, fixed-length messages and
sequence numbers. Code (pure C, host-tested, no HAL): `Core/Src/link_codec.c`
(CRC, COBS, frames), `Core/Src/link_protocol.c` (message layouts, the checks
on what the Pi sends, cps → units, packing), `Core/Src/link_rx.c` (modes,
checks and actions: the only place that decides what a link message does),
`Core/Src/tx_queue.c` (outgoing frames); wired into `main.c` in USER CODE
sections only. The PC/Pi side is `tools/rover_link.py` (standard library
only, so a ROS 2 node can import it as is). Both sides are tested against the
same vectors (`tests/link_vectors.txt`, `tests/link_message_vectors.txt`).

### Why binary

- **Bandwidth.** 115200 8N1 carries 11,520 B/s. The v5 text line already
  takes 5,110 B/s typical (44 %) and 7,860 B/s worst (68 %) for 10 Hz of data.
  The EKF wants the IMU and encoder at 100 Hz: as text that is ~110 B per
  frame, ~11 kB/s, which doesn't fit. Binary: STATE 49 B × 100 Hz + STATUS
  124 B × 10 Hz ≈ 6.1 kB/s, 53 % of the link.
- **Loop time.** Text formatting costs ~14 µs per field (profiler,
  `s2_profile_1`): the text report steps were the worst iterations (median
  676 µs, worst 728 µs, of which 228–266 µs formatting), leaving ~270 µs of
  the 1 ms slot. Packing, CRC and COBS have no per-field formatting; the
  measured binary-mode loop times are under
  [Loop timing in binary mode](#loop-timing-in-binary-mode-measured).
- **Integrity.** The text grammar has no checksum: a corrupted `s` could
  become a valid `m` or `v` (see [Open items](#open-items)). Every frame here
  carries a CRC-16 and is only acted on whole.

### Frame format

```
0x00  COBS( type | seq | payload | crc_lo | crc_hi )  0x00
```

- `type` u8, `seq` u8, then a payload of a **fixed length per type** (any
  other length is rejected), all multi-byte fields **little-endian**. The Pi
  uses one `seq` counter for every frame it sends; the board one per frame
  type.
- **CRC-16/CCITT-FALSE**: polynomial 0x1021, initial value 0xFFFF, no
  reflection, no final XOR (check value 0x29B1 for "123456789"; in Python
  `binascii.crc_hqx(data, 0xFFFF)`), over `type | seq | payload`, low byte
  first. Table-driven (512 B of flash).
- **COBS** removes every 0x00 from the body, so 0x00 only ever means "frame
  boundary": a receiver that joins mid-stream, or loses a byte, is in step at
  the next 0x00. The body is at most 124 bytes (payload ≤ 120), so COBS adds
  one byte and the largest frame is 127 bytes on the wire.
- **A 0x00 before and after every frame.** Two 0x00 in a row are an empty
  frame and are skipped.

### Modes

- **TEXT** (from boot): the v5 text line. `m`/`v`/`s` act only in a bench
  build (`BENCH_TEXT_COMMANDS 1`) or a test build; in the truck build every
  text line is counted in `cmd_bad` and ignored, so text mode only waits for
  a HELLO. A 0x00 can never be part of a text line, so it starts a frame. Only HELLO
  acts in this mode; any other frame is rejected. A frame that grows longer
  than a HELLO (6 COBS bytes) is given up at once and the link is back to
  text, so a stray 0x00 (noise, a NUL typed in a terminal) costs at most 6
  bytes of text instead of swallowing every text line after it.
- **BINARY**: entered by a valid HELLO, **left only by a reset**. The text
  line is not built; STATE and STATUS frames go out instead. Text is ignored
  and counted once per line (`rx_text_ignored`), except in a test build, where
  `e`, `c` and `h` still work (never `m`, `v` or `s`).

### Messages

| Type | Direction | Payload | When | Meaning |
|---|---|---|---|---|
| HELLO `0x01` | Pi → board | 1 B: protocol version (1) | at connect; the tools retry every 0.5 s until a HELLO_ACK, and again after a board reset | enters binary mode; **latches the motor every time**, also in binary mode (a restarted Pi bridge must re-arm on purpose); no effect in FAULT; answered by HELLO_ACK; feeds no watchdog |
| DRIVE `0x10` | Pi → board | 4 B: `speed_cps` i16, `steer_pct` i8, `flags` u8 (bit 0 `steer_valid`, bit 1 `arm`, bits 2–7 must be 0) | 50 Hz from the tools (25 per watchdog timeout) | **the only thing that feeds the motor watchdog**; with `steer_valid` also the steering timeout. \|speed\| ≤ 5,000 cps and \|steer\| ≤ 100 (also when `steer_valid` is 0), else rejected. `arm` only together with speed 0 |
| ESTOP `0x11` | Pi → board | 0 B | on demand | latches the motor at once (driver off, coast); feeds nothing; it stays latched until a DRIVE with `arm` |
| HELLO_ACK `0x81` | board → Pi | 13 B: version, `build_flags`, `rst`, `imu_ok`, IMU WHO_AM_I, min_cps (1000), max_cps (5000), cps_per_unit (50), STATE period (10 ms), STATUS period (100 ms) | after each HELLO | `build_flags`: bit 0 `IWDG_HANG_TEST`, 1 `SPEED_CAPTURE`, 2 `ENCODER_FAULT_TEST`, 3 `LOOP_TIMING_PROFILE`, 4 `BENCH_TEXT_COMMANDS`. **The Pi must refuse to drive whenever `build_flags` ≠ 0** (a test or bench build) |
| STATE `0x82` | board → Pi | 42 B (below) | every 10 ms, at t % 10 == 3 | odometer, speed, IMU sums, motor state |
| STATUS `0x83` | board → Pi | 117 B (below) | every 100 ms, at t % 100 == 8 | everything slower, and the counters |

**STATE** (42 B):

| Field | Type | Meaning |
|---|---|---|
| `t_ms` | u32 | the iteration's scheduled time at frame close (board clock: see [Board clock vs the PC](#board-clock-vs-the-pc-measured)) |
| `count` | i32 | corrected odometer, as `count` in v5 |
| `speed_cps` | i16 | the 20 ms estimate × 50. **Meaningful only with `VALID_NOW`**: an invalid estimate is packed as it is and can saturate (seen: −32,768 at the `e 3` unfreeze) |
| `enc_flags` | u8 | bit 0 `VALID_NOW` (estimate valid at frame close), bit 1 `INVALID_IN_PERIOD` (invalid at some ms since the previous STATE), bit 2 `READ_FAILED` (an angle read failed since the previous STATE) |
| `odometer_epoch` | u8 | +1 at every valid → invalid change of the estimate (wraps): a changed epoch means "distance uncertain" ([odometer caveat](#speed-estimate-and-the-odometer)) |
| `gyro_sum` x, y, z | i32 ×3 | raw gyro summed over exactly the IMU samples of this period; ÷ `imu_samples` = the period's mean |
| `accel_sum` x, y, z | i32 ×3 | the same for the accelerometer |
| `imu_samples` | u8 | samples in the sums (normally 10) |
| `imu_flags` | u8 | bit 0 `imu_ok` (init result), bit 1 a burst read failed in the period |
| `duty` | i16 | applied duty, ticks, signed |
| `mstate` / `ctl` | u8 | low nibble `mstate`, high nibble `ctl` (same codes as v5) |
| `drive_seq_echo` | u8 | `seq` of the last DRIVE acted on |

**STATUS** (117 B): `t_ms` u32; `setpoint_cps` i16 (the last DRIVE as
received); `vi` i16; `s_pct` i8, `s_us` u16, `sstate` u8; `mstate`, `ctl`,
`mfault`, **`latch_reason`**, `bstate`, `imu_ok`, magnet flags (bit 0 MD, 1
ML, 2 MH), `AGC`, `rst` (u8 each); `vbat` and `vdda` in mV (u16); IMU
temperature (raw, i16); `mag_min`, `mag_max`, `loop_us_max`, `i2c_us_max`,
`spi_us_max` (u16; window maxima since the previous STATUS: in binary mode
STATUS closes the window, as the text line does in text mode); then 20 u32
counters since boot (they wrap): `watchdog_trips`, `latches`,
`locked_stalls`, `saturation_stalls`, `enc_bad`, `i2c_errors`, `imu_errors`,
`overruns`, `resyncs`, `tx_dropped`, `steering_timeouts`, `rx_ok`,
`rx_crc_errors`, `rx_frame_errors` (COBS, too short, too long, poisoned),
`rx_rejected` (CRC fine but refused: type, mode, length, flags, range, arm
with a speed), `rx_seq_gaps`, `rx_duplicates`, `rx_uart_errors` (UART errors
and the reception restarts after them), `rx_ring_drops`, `rx_text_ignored`.

Packing **saturates** each field to its wire type (pinned, never wrapped into
a plausible value: `vdda`, kept up to 5,100,000 in v5, is sent as 65,535).
The order and the codes are pinned by `tests/test_link_protocol.c`,
`tools/test_rover_link.py` (also against the C enums) and the shared vectors.

### Safety rules

- **Nothing acts on a frame** until its closing 0x00 has arrived and it has
  passed the COBS, length, CRC, type, mode and payload checks. A frame that
  fails any of them is dropped and counted (`rx_crc_errors`,
  `rx_frame_errors` or `rx_rejected`) and changes nothing else, not even a
  watchdog.
- **Only a validated DRIVE feeds the motor watchdog** (500 ms, as in text
  mode); only a DRIVE with `steer_valid` feeds the steering timeout. HELLO and
  ESTOP latch and feed nothing.
- **Explicit arm.** A latched motor re-arms only on a DRIVE with speed 0
  **and** the `arm` bit. DRIVE 0 without `arm` keeps it latched (and still
  feeds the watchdog); `arm` with a non-zero speed is rejected as a whole.
  **Nothing over the link clears FAULT** (reset only).
- **Every HELLO latches** (unless in FAULT), so a restarted bridge can't
  inherit a running motor.
- **Raw-cps minimum:** \|speed_cps\| < 1000 coasts (see [Units](#units)).
- **ESTOP** latches at once, whatever the speed; plain DRIVEs after it keep it
  latched (bench: 743 DRIVE 0 over 14.9 s), only an arming DRIVE re-arms.
- A UART error or a dropped byte inside a frame **poisons** that frame.
- **Sequence numbers** are counted (gaps, duplicates), never acted on: a
  restarted Pi shows as one gap.

### Latch reasons and stall counters

`latch_reason` (STATUS; `MotorLatchReason` in `Core/Inc/motor_control.h`,
append only) is set exactly while the motor is LATCHED; re-arming and FAULT
clear it. If a second latch comes while already latched, the first reason
stays (a HELLO right after boot shows BOOT).

| Code | Reason | Set by |
|---|---|---|
| 0 | none | not latched |
| 1 | BOOT | `MotorController_Init` |
| 2 | WATCHDOG | no valid motor command for 500 ms |
| 3 | STALL LOCKED | stall rule 1 (duty on, wheel not turning) |
| 4 | STALL SATURATION | stall rule 2 (at the cap below half the setpoint) |
| 5 | ESTIMATE INVALID | the speed estimate stayed invalid under closed loop |
| 6 | ESTOP | an ESTOP frame |
| 7 | HELLO | a HELLO frame |

`locked_stalls` + `saturation_stalls` always add up to `stall_n` (the text
line keeps only `stall_n`). `latches` counts the latches that stalls, the
estimate, ESTOP and HELLO caused; watchdog trips count in `watchdog_trips`.

### Transmit side and schedules

STATE (t % 10 == 3) and STATUS (t % 100 == 8) never land on the health
iteration (50), a battery sample (t % 10 == 5) or each other
(`tools/test_firmware_sources.py` checks the marks). Frames go through
`tx_queue` (four 128-byte slots), because a STATUS frame (124 B, ~10.8 ms on
the wire) outlasts the 10 ms between two STATE frames; see
[Interrupt-shared variables](#interrupt-shared-variables). HELLO_ACK goes out
in the iteration the HELLO is acted on. In binary mode the text report is
skipped (not counted in `skipped`) and a half-built line is dropped. In a
`SPEED_CAPTURE` build the capture lines still go out as text between frames
(decoders skip them; a frame can wait ~13 ms); in a `LOOP_TIMING_PROFILE`
build the profiler keeps measuring but never sends in binary mode: use
STATUS's `loop_us_max`.

### Loop timing in binary mode (measured)

`loop_us_max` per STATUS window, 2026-10-15/16 (see
[bench results](#bench-results-2026-10-15-and-10-16-pi-link)):

| Load | `loop_us_max` |
|---|---|
| Idle (nothing received; `b15_hour`, 35,170 windows) | 707–708 (707–711 across all logs) |
| DRIVEs at 50 Hz (`b3_soak_1`, 6,282 windows) | median 713, 99th percentile 790, max **820**; 6.2 % of windows above 760 (`b2_bin_nolipo`: max 795) |
| Continuous receive (`inject crc 999` / `inject type 999`, ~11 kB at line rate) | **822–854** in every window the burst covers |

**Cause** (indirect evidence; the split between interrupt and decoding time
isn't measured): the health iteration (t % 100 == 50, the two extra AS5600
reads, already the longest at ~709 µs) overlapping received bytes. Each byte
costs a receive interrupt, and each frame its COBS/CRC decoding at the start
of the next iteration: about 11 µs per byte all-in at line rate. In
`b3_load_1`, windows whose burst missed ms 50 stayed at 708 even with 40–56 ms
of continuous receive, while a single frame that overlapped ms 50 lifted its
window to 768. CRC-failing and CRC-passing-but-rejected frames cost the same.
With DRIVEs flowing, the elevated windows come in **runs of 2–5 consecutive
windows about 4.5 s apart**: the 50 Hz DRIVE phase slides past the board's
ms 50 at the difference between the PC and board clocks (20 ms / ~0.45 % ≈
4.5 s). The worst seen, 854 µs, leaves 146 µs of the 1 ms slot.

### Board clock vs the PC (measured)

`b15_hour` (2026-10-15): 351,702 STATE `t_ms` values fitted against the PC's
receive times (`<log>.times.csv`): the board clock runs **+0.35 % fast on
average, drifting from +0.40 % to +0.21 %** over the hour (5-minute segments;
smooth, not receive latency). Which clock moved is unknown: MSI is an RC
oscillator that drifts with temperature, and Windows may slew `time.time()`.
**The Pi must map `t_ms` to its own clock continuously** (offset and rate,
re-fitted), not with a fixed factor; over that hour the rate moved by about
2 ms per second. This replaces the earlier ~0.6 % estimate from the meter
(see [Open items](#open-items)).

### Sizes

With the Pi link complete (2026-10-16, Debug): **FLASH 37,248 B, RAM 5,800 B**, 0 warnings; the link
added +4,256 / +1,080 B over the 2 KB-stack build of 2026-10-15 (32,992 / 4,720). Static RAM:
`link_tx_queue` 532, `link_rx` 308, the STATUS message 172, the STATE
message 68. `main()`'s frame is 976 B (`-fstack-usage`). Minimum stack 2 KB
since 2026-10-15; the deepest chain is estimated at ~1.6 KB (`main` 976 +
STATUS send 136 + frame send 152 + frame encode 152 + COBS 20, plus an
interrupt on top); the real high-water mark is not measured.

### Bench results (2026-10-15 and 10-16, Pi link)

On the lift, `tools/motor_console.py --binary` / `tools/serial_logger.py
--binary`, decoded with `tools/link_dump.py` (every log below: 0 bad frames,
0 sequence gaps, 0 `t_ms` gaps, exit code 0).

| Log | What | Result |
|---|---|---|
| `b2_bin_nolipo` (10-15, no LiPo) | handshake, `s 30`, `-s`, `inject crc 5`, `!` text | `s 30`: 1500 → 1590 µs, `sstate` 1; `-s`: centred by the 500 ms steering timeout (`steering_timeouts` +1); `rx_crc_errors` +5 exactly; `rx_text_ignored` +1 |
| `b15_hour` (10-15, 1 h, `serial_logger --binary`) | idle, motor latched | 351,702 STATE (100.0 Hz) + 35,170 STATUS; every event counter 0 and unchanged all hour (`rx_ok` 5 and `rx_seq_gaps` 1 were already there at the start); `loop_us_max` 707–708 |
| `b3_load_1` (10-16) | `inject crc 999`, `inject type 999` | `rx_crc_errors` +999, then `rx_seq_gaps` +1 at the next good frame; `rx_rejected` +999; `loop_us_max` 822–854 during the bursts; motor latched throughout |
| `b3_drive_1` (10-16) | speeds, the five latches | DRIVE 2500: `speed_cps` median 2500 (duty 295); 1000: 1000 (duty 156); 999: coasts (`ctl` stop). Latches in order: **ESTOP** (stayed latched under plain DRIVE 0 until `arm`), **HELLO** (with DRIVE 2500 flowing), **WATCHDOG** (435–534 ms after the last DRIVE; STATUS resolution 100 ms), **STALL LOCKED** at DRIVE 1000 (gloved hold; condition held 510 ms), **STALL SATURATION** at DRIVE 2500 (duty 480, speed 650–750 < 1250; held 1030 ms); `locked_stalls` 1, `saturation_stalls` 1 |
| `b3_soak_1` (10-16) | DRIVE 2500 for 10.8 min | 64,537 STATE + 6,453 STATUS; `loop_us_max` 709 / 713 / 820 (min / median / max); vbat 7,871 → 7,809 mV (meter 7.89 V vs vbat 7.87 V); IR, motor and TB6612: 25.3 → 27.6 °C; no event counter changed; `odometer_epoch` 0 throughout |
| `b3_encfault_2` (10-16, `ENCODER_FAULT_TEST`) | `!e 2` and `!e 3` under DRIVE 2500 | HELLO_ACK `build_flags` 4 (`ENCODER_FAULT_TEST`). `e 2`: **ESTIMATE INVALID** latch 41–59 ms after the reads started failing (code: the 52nd invalid ms), `odometer_epoch` +1 at once, `enc_bad` 1. `e 3`: **STALL LOCKED** 510–530 ms after the freeze, `odometer_epoch` +1 at 691–710 ms (the unfreeze), `count` one revolution short (jump −1,384), `locked_stalls` 1 |

Timings from STATE are to its 10 ms period. In `b3_encfault_1` (no LiPo) the
locked-wheel rule caught a real no-power stall (duty to the cap, `count` 0)
0.57 s after the start.

## Open items

**Done** (carrier build, up to 2026-10-14): protoboard carrier, keyed battery
connector, 10 kΩ STBY pull-down, 470 µF on VM, 1000 µF on the servo rail,
servo on the carrier, battery monitor + low-voltage cutoff, due-time
scheduling. **Done** (up to 2026-10-15): encoder angle correction
(H4), 20 ms speed estimate, closed-loop speed control (`v`), stall protection
(locked wheel + saturation, repeated → FAULT), telemetry v5 with `mfault`,
the capture and encoder-fault test builds, tools for v5 (`v` in the console).
**Done** (up to 2026-10-16, the Pi link): the Pi link (binary protocol v1, see
[Pi link](#pi-link-binary-protocol-v1)), which also closes the framed protocol
with a checksum, the stall-cause field (STATUS `latch_reason`,
`locked_stalls` / `saturation_stalls`) and the 2 KB stack reservation
(2026-10-15). **Done** (2026-10-16): `LOOP_TIMING_PROFILE` is
guarded like the other switches (`#warning`, committed-as-0 and compiled-out
pins). **Done** (2026-10-16, after the profiler guard): the text `m` gate: the truck build
(`BENCH_TEXT_COMMANDS 0`, committed) acts on no text command before a HELLO;
bench builds show bit 4 in HELLO_ACK.

Not done yet:

- **A "shut down" line from the Nucleo to the Pi (GPIO).** The Pi's
  low-battery shutdown reads the battery through the ROS bridge, so it can't
  act while the bridge isn't running (a motor_console session). A GPIO the
  MCU drives at cutoff would work without the bridge. Needs wiring and firmware.
- **Inline fuse (required before floor driving).** In the + lead, as close to
  the pack as possible (3–5 A was the earlier plan). A wiring short on a LiPo
  is a fire risk, and the TB6612 has no current limiting. Check the Dupont
  adapter's current rating at the same time (about 1–3 A, unverified).
- **Series Schottky diode: not planned.** The keyed connector was chosen
  instead: a series diode would cost ~0.4 V and sit in front of the divider,
  so the cutoff would see less than the pack voltage.
- **Framed serial protocol with a checksum: done** (the Pi link). It removes
  the two text-mode items below for the Pi; they remain for the bench text
  grammar only.
- **Stack: `_Min_Stack_Size` (2 KB since 2026-10-15) is a link-time reservation,
  not a limit.** The stack can grow from `_estack` (0x2000C000) down into
  the free RAM above `.bss` and the heap: `.bss` ends at `_end` = 0x20000CA8,
  the heap reservation (`_Min_Heap_Size` 0x200) at 0x20000EA8, which leaves
  **45,400 B (about 44 KB)** for the stack, the 2 KB reservation included
  (`rover_firmware.map`, `._user_heap_stack`, with the code of 2026-10-16). The
  deepest chain (a STATUS send) is estimated at **~1.6 KB** with an interrupt
  on top (`main()`'s frame is 976 B, `-fstack-usage`). **The high-water mark
  is not measured (deferred):** fill the stack region with a pattern at boot,
  find the deepest overwritten word with a small incremental scan, and
  report it (a u16 in STATUS, which has 3 bytes left, would mean protocol
  version 2; see below).
- **STATUS is nearly full: 117 of the 120-byte payload limit**
  (`LINK_MAX_PAYLOAD`). Anything more needs either a bigger limit (the TX
  slots are 128 B, so 121 B of payload at most without changing them) or a
  second, slower frame type.
- **Test builds accept `e`, `c` and `h` in binary mode** (on purpose, so the
  encoder-fault, capture and hang tests can run against the Pi link), and
  test and bench builds accept `m`/`v`/`s` in text mode. HELLO_ACK reports
  both (`build_flags` bits 0–4): **the Pi must refuse to drive whenever
  `build_flags` ≠ 0**. Not implemented on the Pi side yet.
- **Loop headroom with heavy receive.** The worst `loop_us_max` seen is 854 µs
  (continuous receive overlapping the health iteration; 820 µs with normal
  50 Hz DRIVEs), leaving 146 µs. Not a problem yet; if more is added to the
  health iteration, or the Pi sends much more than DRIVEs, measure again
  (the interrupt vs decoding split isn't measured: an `!` + long text burst
  is the missing step).
- **Encoder I2C time: 282 µs of every 1 ms** (both reads, profiler,
  2026-10-15), the largest part of every iteration. Later: read the magnitude
  only in the health iteration, or run the bus faster. Not needed yet (the
  worst iteration is 728 µs).
- **Floor re-identification.** The gains, the preload, the stop times and the
  release overshoot were all measured on the lift. On the floor: re-run the
  step captures with the truck's weight on the wheels (gain, τ, breakaway,
  stopping), check `v` steps, the saturation rule against real inertia and
  slopes, and the overshoot after a load is released (bench: 71 units after
  releasing a drag at `v 50`).
- **A separate stall-cause field: done** in the Pi link's STATUS
  (`latch_reason` STALL LOCKED / STALL SATURATION, `locked_stalls`,
  `saturation_stalls`; bench `b3_drive_1`). The text line (v5, frozen) still
  has only `stall_n`. The Pi should react differently to the two (an
  obstacle vs an unreachable setpoint).
- **Receive-error attribution, text mode only** (`Commands_ProcessReceived`
  in `main.c`; in binary mode the damaged frame fails its CRC anyway).
  A UART error or dropped byte is noticed by comparing a counter after each
  byte is popped, and poisons the line being assembled *at that moment*. That
  is exact when the queue is near empty. But if several bytes are queued and a
  `\n` lies between the oldest queued byte and the damaged one, the **wrong**
  line is rejected and the damaged one is parsed. Worst case: `m -20` loses its
  `-` and is accepted as `m 20`. Bounded by the cap, ramp and reverse dwell,
  and unlikely (the queue is drained every 1 ms; keepalives are ~6 bytes per
  100 ms). Fixed for the Pi by the framed protocol; for the bench grammar the
  fix would be the interrupt pushing an in-band marker into the queue at the
  error.
- **A corrupted letter can turn a steering line into a motor line**, text
  mode only (accepted for the bench; the Pi link's DRIVE carries speed and
  steering in one CRC-checked frame). `s` and `m` differ in 3 bits, and **`s` and `v` in only 2**
  (0x73 / 0x76), so a byte corrupted in a way the UART doesn't flag could turn
  `s 30` into a valid `m 30` or `v 30`. USB has a CRC and the ST-LINK → MCU
  UART is a short trace, so this is very unlikely. Bound:
  - while armed (`mstate` 0 or 1) **with motor keepalives flowing** (`m` or
    `v`), a corrupted command acts for at most ~100 ms (ramp: 2 ticks/ms →
    200 ticks, ~12 % duty) before the next real keepalive sets the target
    back;
  - armed with **no** motor keepalives (e.g. after `v 0` and the console's
    `-v` test aid), the motor runs until its own watchdog, 500 ms after the
    corrupted line: it reaches the 30 % cap after 240 ms;
  - latched (`mstate=2`) a corrupted `m` or `v` is ignored; a reversal still
    goes through the gate, the ramp-down and the 100 ms dwell.
  Fixed for the Pi by the framed protocol (above).
- **Clock accuracy (measured 2026-10-15; the drift's cause unknown).** Against
  the PC over one hour (`b15_hour`, see
  [Board clock vs the PC](#board-clock-vs-the-pc-measured)) the board runs
  **+0.21 to +0.40 % fast, +0.35 % on average, drifting** during the hour.
  That replaces the earlier estimate from the meter: the servo PWM measured
  **50.32 Hz** against 50.00 nominal (+0.64 %), which was never confirmed and
  may be the meter's own accuracy. Every firmware time base comes from
  SYSCLK, so `t_ms` advances that much fast and speeds computed from `d100`
  per 100 firmware-ms read that much low. **The Pi must re-fit `t_ms`
  against its own clock continuously**, not apply a fixed factor. SYSCLK is the PLL fed
  by **MSI** (range 6, 4 MHz, × 16 / 2 = 32 MHz): a factory-trimmed internal
  RC oscillator, **not locked to the LSE** (MSI PLL mode, `MSIPLLEN`, is never
  enabled, and the LSE isn't switched on even though PC14/PC15 are reserved
  for the Nucleo's 32.768 kHz crystal). The IWDG runs from LSI, so it isn't
  affected. Fixes: enabling the LSE and MSI PLL mode (a CubeMX change, then
  re-measure the same way), or correcting it on the Pi (needed anyway for the
  offset).
- **Re-measure the servo-rail dip** with the 1000 µF fitted. Before it, buck
  #2 dipped from 5.56 V to about 4.45 V when the wheels were forced
  (2026-10-09; the meter averages, so the real minimum may be lower). Same
  method: meter on buck #2's output while the wheels are forced.
- **I2C bus-hang recovery.** With the 3V3 pigtail unplugged (no pull-ups) every
  encoder read waits out HAL's timeouts; the loop keeps going (see
  [Loop scheduling](#loop-scheduling-due-times)), but nothing frees a stuck
  bus.
- **Encoder mount check.** The magnet field strength varied on 2026-10-14;
  check the AS5600/magnet mounting (`MD`/`ML`/`MH`, `AGC`, `mag_min`/`mag_max`
  in telemetry). The angle correction is tied to the board's current seating
  (see [Encoder angle correction](#encoder-angle-correction-h4)), so anything
  that moves the board means a re-fit.
- **Centre the steering on ESTOP.** An ESTOP latches only the motor. The
  steering centres by its own 500 ms timeout once the Pi stops sending DRIVEs
  (`docs/ros-bridge.md`, Steering). Centring it in the ESTOP handler would
  remove that half second of turned wheels on a coasting truck.
- **Steering at part lock.** `max_steering_angle_radians` (0.204, 2026-10-22)
  was measured only at full lock; the bridge treats percent → angle as linear.
  A turning circle at `s ±50` would check that.
- **IWDG reset cause is ambiguous** between a hung loop and a fault handler
  (both read `rst=36`). A `.noinit` RAM marker written by the fault handlers
  would tell them apart.

## Toolchain

- STM32CubeMX (project file: `rover_firmware.ioc`, target toolchain = CMake)
- VS Code with the STM32Cube extensions
- CMake + Ninja, GNU Tools for STM32 (arm-none-eabi-gcc 14.3.1+st.2), all
  installed as STM32Cube bundles (see `.settings/bundles.store.json`)

## Build and flash

1. Open **`firmware/`** (not the repo root) in a **Windows** VS Code window. The
   ST-LINK is attached to Windows.
2. Build with **F7** and choose the **Debug** preset. Output goes to
   `build/Debug/`, which git ignores.
3. Flash and run with **Ctrl+F5**. LiPo disconnected (see [Safety](#safety)).

The Debug build uses `-Og -g3`, set in the top-level `firmware/CMakeLists.txt`
(it overrides the toolchain file's `-O0`; see the comment there for why).
Size with the text-command gate (2026-10-16, after the profiler guard): **FLASH
37,320 B, RAM 5,800 B**, 0 warnings (the linker's memory report); at
the Pi link (2026-10-16) it was 37,248 / 5,800. Before the Pi link, with the 2 KB stack (2026-10-15): 32,992 /
4,720; earlier on 2026-10-15: 32,936 / 3,696 (1 KB stack); at the carrier build (2026-10-14): 30,488 /
3,440. RAM includes the 2 KB minimum stack reservation since 2026-10-15.

### Build switches

Each switch is a `#define` that is **committed as 0** and set to 1 only for its
own bench test. Every one of them prints a `#warning` when 1, and
`tools/test_firmware_sources.py` (and the module's host test, where there is
one) fails if one is committed as 1; for `LOOP_TIMING_PROFILE` the pins also
check that none of the profile code is left in a normal build (only its type
declarations, which cost nothing).
**Never commit any of them at 1**, and reflash a normal build when the test is
done. Sizes measured on 2026-10-15, before the 2 KB stack (only one switch at 1):

| Switch | Where | Adds | FLASH / RAM |
|---|---|---|---|
| none (normal build) | | | 32,936 / 3,696 B |
| `IWDG_HANG_TEST` | `Core/Inc/command_dispatch.h` | `h <n>`: stall the loop ([Hang test](#hang-test)) | not re-measured |
| `BATTERY_CUTOFF_TEST` | `Core/Inc/battery_monitor.h` | thresholds above a full pack ([Cutoff test build](#cutoff-test-build)) | not re-measured |
| `SPEED_CAPTURE` | `Core/Inc/speed_capture.h` | `c <p>`: motor step captures ([Capture build](#capture-build)) | 34,416 / 16,088 B |
| `ENCODER_FAULT_TEST` | `Core/Inc/encoder_fault.h` | `e 1/2/3`: injected encoder faults ([Encoder fault test build](#encoder-fault-test-build)) | 33,152 / 3,712 B |
| `LOOP_TIMING_PROFILE` | `Core/Src/main.c` (USER CODE PD) | one `prof ...` line per window ([Loop profiler](#loop-profiler)) | 35,336 / 5,256 B |
| `BENCH_TEXT_COMMANDS` | `Core/Src/main.c` (USER CODE PD) | `m`/`v`/`s` over USB in text mode, before a HELLO (bench driving without `--binary`; HELLO_ACK bit 4) | not measured |

`SPEED_CAPTURE` can't be combined with `ENCODER_FAULT_TEST` or with
`LOOP_TIMING_PROFILE` (`#error`): a capture must record the real encoder, and
the profile and capture lines would both go out at ms 51.

The sizes in the table are from before the Pi link (2026-10-15, 1 KB stack);
the normal build is now 37,320 / 5,800 B. Every switch set to 1 also shows in
HELLO_ACK's `build_flags` (`BATTERY_CUTOFF_TEST` excepted), and test builds
keep their own letters (`e`, `c`, `h`) in binary mode. **Text commands:** the
four test builds (`IWDG_HANG_TEST`, `SPEED_CAPTURE`, `ENCODER_FAULT_TEST`,
`LOOP_TIMING_PROFILE`) enable `m`/`v`/`s` in text mode by themselves (the
capture needs `m` steps, the profiler `v` under load); in a normal build
only `BENCH_TEXT_COMMANDS 1` does. The **committed build is the truck build**:
its text mode acts on no command and only waits for a HELLO (every text line
counts in `cmd_bad`), so a bench session without `--binary` needs
`BENCH_TEXT_COMMANDS 1` flashed (and a normal build reflashed afterwards). Flip a switch without
touching anything else: the working tree has CRLF line endings, so match the
`0` without anchoring the line end, and check that exactly one byte changed
(from the repo root, in WSL):

```
cp firmware/Core/Inc/encoder_fault.h /tmp/encoder_fault.h.orig
sed -i 's/^#define ENCODER_FAULT_TEST  0/#define ENCODER_FAULT_TEST  1/' firmware/Core/Inc/encoder_fault.h
cmp -l /tmp/encoder_fault.h.orig firmware/Core/Inc/encoder_fault.h    # exactly one line: <offset> 60 61
git checkout -- firmware/Core/Inc/encoder_fault.h                     # back to 0 when the test is done
```

### Capture build

`SPEED_CAPTURE 1` records the motor's response to one `m` step into RAM (12 KB:
3000 samples) and sends it as text lines for `tools/fit_motor_step.py` and
`tools/fit_angle_correction.py`. On the lift, console running:

```
m 0
c 2       arm: one sample every 2 ms (1..10; c 1 = 3 s at 1 kHz for a step, c 4 = 12 s)
m 30      the next m that changes the target starts the recording
m 0       (later) also records the ramp down and the stop; c 0 cancels
```

After 3000 samples it sends a `caph ...` header and then one `cap ...` line
(20 samples) after each telemetry line: `d` = counts since the previous good
sample (`x` = failed read, not zero motion), `u` = applied duty in ticks
(`Core/Inc/speed_capture.h`). The capture records the **raw** (uncorrected)
count. `serial_logger.py` counts `caph`/`cap` lines as bad; that is expected.

### Encoder fault test build

`ENCODER_FAULT_TEST 1` adds `e <n>` (`Core/Inc/encoder_fault.h`). It changes the
angle read of each ms from the ms the command arrives in, before both
unwrappers. An injected failure is not counted in `i2c_err`; a real failed read
during a freeze stays a failed read.

| Command | Fault | Expected (bench 2026-10-15) |
|---|---|---|
| `!e 1` | the angle read fails for 20 ms | under `v`: held, no latch (`vok=0` ~41 ms), `enc_bad` unchanged |
| `!e 2` | the angle read fails for 100 ms | under `v`: latch at the 52nd invalid ms, `enc_bad`+1; under `m`: nothing |
| `!e 3` | the angle freezes for 700 ms (the reads succeed) | a valid estimate of 0: with duty ≥ 128 ticks a stall latch ~0.52 s after the command, `stall_n`+1; three within 60 s → `mfault=7` |

Any other value, or an `e` while a fault is still running, is refused and
counted in `cmd_bad`. `e` feeds no watchdog. Send it with the console's `!`
(`!e 2`): the console itself only sends `m`, `v` and `s`. In binary mode
(`motor_console.py --binary`) the same `!e 2` works in this build (the line
also counts in `rx_text_ignored`), and STATE/STATUS show the result:
`latch_reason`, `enc_bad`, `locked_stalls`, `enc_flags` and the
`odometer_epoch` change. Bench 2026-10-16 over the link (`b3_encfault_2`,
DRIVE 2500): `e 2` → ESTIMATE INVALID 41–59 ms after the reads failed, epoch
+1 at once; `e 3` → STALL LOCKED 510–530 ms after the freeze, epoch +1 at the
unfreeze (~0.70 s), count one revolution short (see
[bench results](#bench-results-2026-10-15-and-10-16-pi-link)). Run `e 2` and `e 3`
**with the LiPo connected and the motor running**: latched, `e 2` only shows
the epoch change (the controller doesn't run while latched).

### Loop profiler

`LOOP_TIMING_PROFILE 1` times each section of each kind of iteration and sends
one extra `prof ...` line per window with the window maxima, read by
`tools/summarize_loop_profile.py`. None of the profile code is compiled in a
normal build. Kinds: `idle`, `busy` (UART sending), `hlth`, the five report
steps `head`, `tail`, `motr`, `strg` (reset cause + steering + battery) and
`spd` (speed control part + UART start, `spd_tx`), and `bld` (the profile
line's own building). Window-wide maxima: `mot` (the motor section, every
iteration), `ctl_us` (speed control + stall check, inside `mot`) and `est_us`
(angle correction + corrected unwrap + estimate, inside the encoder section).
Since v5 the ~520-byte profile line is built from ms 6 of the window and sent
at ms 51–53 (after the health iteration), so it is out before the next
telemetry line; one that can't be sent by ms 53 is dropped, never delayed.
Profile lines from firmware before v5 (with `strg_tx`, without `spd_*`) are no
longer read. Results: see [Bench results](#bench-results-2026-10-15-on-the-lift).
**In binary mode the profiler measures but never sends** (its window only
advances when a text line starts, and none is built): use the text mode for
profiling, and STATUS's `loop_us_max` in binary mode.

## Host tests

From `firmware/` in WSL (GCC, with AddressSanitizer and UBSan). With
`-fno-sanitize-recover=all` every sanitizer finding ends the run with an error
instead of a printed warning. Each suite prints "ALL PASSED". The two suites
that link `as5600.c` need `-Itests/stubs` (a stand-in `stm32l4xx_hal.h`):

```
F="-std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -ICore/Inc"
gcc $F tests/test_telemetry_format.c Core/Src/telemetry_format.c -o /tmp/test_telemetry_format && /tmp/test_telemetry_format
gcc $F tests/test_motor_control.c Core/Src/motor_control.c Core/Src/byte_ring.c -o /tmp/test_motor_control && /tmp/test_motor_control
gcc $F tests/test_tb6612_plan.c Core/Src/tb6612_plan.c Core/Src/motor_control.c -o /tmp/test_tb6612_plan && /tmp/test_tb6612_plan
gcc $F tests/test_command_line.c Core/Src/command_line.c -o /tmp/test_command_line && /tmp/test_command_line
gcc $F tests/test_steering_control.c Core/Src/steering_control.c -o /tmp/test_steering_control && /tmp/test_steering_control
gcc $F tests/test_command_dispatch.c Core/Src/command_dispatch.c Core/Src/command_line.c Core/Src/motor_control.c Core/Src/steering_control.c Core/Src/speed_control.c -o /tmp/test_command_dispatch && /tmp/test_command_dispatch
gcc $F tests/test_loop_schedule.c Core/Src/loop_schedule.c -o /tmp/test_loop_schedule && /tmp/test_loop_schedule
gcc $F tests/test_battery_monitor.c Core/Src/battery_monitor.c -lm -o /tmp/test_battery_monitor && /tmp/test_battery_monitor
gcc $F tests/test_speed_capture.c Core/Src/speed_capture.c Core/Src/telemetry_format.c -o /tmp/test_speed_capture && /tmp/test_speed_capture
gcc $F tests/test_speed_estimator.c Core/Src/speed_estimator.c -lm -o /tmp/test_speed_estimator && /tmp/test_speed_estimator
gcc $F -Itests/stubs tests/test_angle_correction.c Core/Src/angle_correction.c Core/Src/as5600.c Core/Src/speed_estimator.c -lm -o /tmp/test_angle_correction && /tmp/test_angle_correction
gcc $F tests/test_speed_control.c Core/Src/speed_control.c Core/Src/motor_control.c Core/Src/speed_estimator.c -lm -o /tmp/test_speed_control && /tmp/test_speed_control
gcc $F tests/test_stall_detect.c Core/Src/stall_detect.c Core/Src/motor_control.c Core/Src/speed_control.c -o /tmp/test_stall_detect && /tmp/test_stall_detect
gcc $F tests/test_saturation_stall.c Core/Src/stall_detect.c Core/Src/speed_control.c Core/Src/speed_estimator.c Core/Src/motor_control.c -lm -o /tmp/test_saturation_stall && /tmp/test_saturation_stall
gcc $F -Itests/stubs tests/test_encoder_fault.c Core/Src/encoder_fault.c Core/Src/as5600.c Core/Src/angle_correction.c Core/Src/speed_estimator.c Core/Src/speed_control.c Core/Src/stall_detect.c Core/Src/motor_control.c -lm -o /tmp/test_encoder_fault && /tmp/test_encoder_fault
gcc $F tests/test_link_codec.c Core/Src/link_codec.c -o /tmp/test_link_codec && /tmp/test_link_codec
gcc $F tests/test_tx_queue.c Core/Src/tx_queue.c -o /tmp/test_tx_queue && /tmp/test_tx_queue
gcc $F tests/test_link_protocol.c Core/Src/link_protocol.c Core/Src/link_codec.c -lm -o /tmp/test_link_protocol && /tmp/test_link_protocol
gcc $F tests/test_link_rx.c Core/Src/link_rx.c Core/Src/link_protocol.c Core/Src/link_codec.c Core/Src/motor_control.c Core/Src/speed_control.c Core/Src/steering_control.c -o /tmp/test_link_rx && /tmp/test_link_rx
gcc $F tests/test_link_fuzz.c Core/Src/link_rx.c Core/Src/link_protocol.c Core/Src/link_codec.c Core/Src/motor_control.c Core/Src/speed_control.c Core/Src/steering_control.c -o /tmp/test_link_fuzz && /tmp/test_link_fuzz
```

`test_link_codec` and `test_link_protocol` read their vectors
(`tests/link_vectors.txt`, `tests/link_message_vectors.txt`) relative to
`firmware/`, so run them from there.

| Suite | Covers | Checks on 2026-10-16 |
|-------|--------|-------|
| `test_telemetry_format` | the formatter is byte-identical to `snprintf` | 17,431 |
| `test_motor_control` | motor state machine (ramp, reversal, watchdog, latch, FAULT, a random hour), the closed-loop API (`v` feeds the watchdog, the target alone doesn't; latch), the latch reasons (LATCHED exactly when a reason is set, pinned codes), the explicit-arm DRIVE command and the receive queue | 279 |
| `test_tb6612_plan` | the motor output-stage guards | 65 |
| `test_command_line` | the line grammar, for every letter | 73 |
| `test_steering_control` | steering states, timeout, mapping, limits, output guard, a random hour against a model | 68 |
| `test_command_dispatch` | routing (`m`, `v`, `s`, `h`); a stream of `s` can't keep the motor running; `m` → `v` bumpless; the hang switch is 0 | 32 |
| `test_loop_schedule` | due-time scheduling (1 ms steps as before, 52 ms steps, any step size, jumps, tick wrap) and the heartbeat patterns | 312,026 |
| `test_battery_monitor` | battery voltage conversion against floating point, ADC plausibility and fault latch, filter, presence, warning, cutoff latch (thresholds and hold times to the ms, tick wrap), re-arming after a removal, motor-PWM ripple can't hold off the cutoff, a random hour; pins the decided thresholds and the cutoff-test switch at 0 | 119 |
| `test_speed_capture` | the capture recorder (trigger, deltas, missing samples, period) and its line format; the switch at 0 | 31 |
| `test_speed_estimator` | the 20 ms estimate, every validity rule entering and leaving, count and time wrap, a random hour against a model | 60 |
| `test_angle_correction` | the H4 correction against floating point for every raw value, zero mean, periodic, monotonic, the peak; end to end through a second unwrapper and the estimator; the pinned constants | 35 |
| `test_speed_control` | the PI against a plant model (τ 0–30 ms, both signs): steps, saturation without windup, load, gate, minimum setpoint, preload, invalid estimate (latch reason ESTIMATE INVALID), latch/FAULT/watchdog, pinned gains and derivation, `ctl` and `vi` | 110 |
| `test_stall_detect` | both stall rules placed exactly (timing, thresholds, every reset, sign handling, `m` unchanged), which rule latched (latch reason, per-rule counts adding up to `stall_n`) and the 3-in-60 s FAULT across the tick wrap | 148 |
| `test_saturation_stall` | the saturation rule in closed loop with a plant: a slipping hold latches, `v 100` on the lift never does, drags, transient caps with inertia, mixed escalation | 106 |
| `test_encoder_fault` | the `e` injection, then each fault through the whole chain in `main.c`'s order | 1,314 |
| `test_link_codec` | CRC-16 (check value, every byte against a bit-by-bit reference), COBS (the published examples, every length to 300, strict decoding, exact capacities), frames (every payload length, type and seq; the shared vectors; all 162,832 single and 76,560 double bit flips rejected) | 106 |
| `test_tx_queue` | order, full and empty, length limits, the frame being sent never overwritten, the count wrap, a million random operations against a FIFO model | 21 |
| `test_link_protocol` | cps → units (every value in ±5000 against a floating-point reference, the raw-cps minimum and the ties), every DRIVE flag byte and the full speed and steering ranges, HELLO/ESTOP checks, HELLO_ACK/STATE/STATUS packing against the shared vectors (from Python `struct`), saturation | 132 |
| `test_link_rx` | the text-command gate (every mode × letter × switch); with the real motor, speed and steering: TEXT/BINARY modes, a stray 0x00 in text mode, every-HELLO-latches, the explicit arm, ESTOP then plain DRIVE 0 stays latched, only DRIVE feeds the watchdog, steer_valid, dropped frames change nothing, poison, sequence counting, text in binary mode | 75 |
| `test_link_fuzz` | 3,000,000 random bytes act on nothing; 1,000,000 random bodies in valid COBS (the few that pass the CRC by chance are counted: 19, one an ESTOP); 200,000 real frames half damaged: every frame acted on was sent unchanged; no watchdog ever moves without a DRIVE | 8 |

All with 0 failed (run on 2026-10-16 with the Pi link complete, GCC 13.3 in WSL Ubuntu
24.04). A different count after a change is expected when tests are added; a
failure never is.

Tools, from the repo root (in WSL): `python3 -m pytest tools/` (**459 passed**
on 2026-10-16; 158 on 2026-10-15). This includes
`tools/test_firmware_sources.py`, which reads the C sources and checks the rules
that live in the code's structure: one IWDG refresh, as the last statement of
the main loop and in no interrupt handler; the IWDG started from user code after
its debug freeze, with prescaler 32 and reload 249; the hang switch off; the
calibrated steering limits; the servo guard locking the steering out; the emergency stop's servo write; every
fault handler stopping the actuators first; no `%` schedules in the main loop
(each periodic task checked once per iteration through `loop_schedule`); the
battery rules (cutoff-test switch off, samples never on a report or health ms,
sampling never waits and gets the real duty, the lockout reaching the motor
every ms, an ADC failure at boot being a fault, the boot ADC waits timing out
well under the IWDG, 640.5-cycle sampling in the `.ioc` and the code, PB6
never driven or pulled); the closed loop (speed control and stall check reach
the motor only through its API and read the speed controller read-only, only
the dispatcher feeds the motor watchdog, the loop order read → correction →
corrected unwrap → estimate and commands → speed control → stall check →
motor, `count`/`d100` corrected while `raw` and the capture stay raw, the
pinned angle-correction and saturation constants, the `mfault` and `ctl`
codes and the console's names for them); the capture and encoder-fault
switches off and compiled out of a normal build; the telemetry (v5) field
order, its five build steps and the worst-case line against the 1024-byte
buffer; the profile field order matching the tools; and the Pi link's wiring
in `main.c` (the link sources in the build; only a validated DRIVE, called
from `link_rx.c` after its checks, feeds the motor watchdog; STATE and STATUS
never on a health, battery or each other's ms; the text line only in text
mode and frames only in binary mode; every byte to the link first, a frame
start dropping the partial text line, a UART error poisoning the frame or
else the text line; text in binary mode only in a test build and only `e`,
`c`, `h`; the transmit queue popped only by the transmit-complete callback,
which never refreshes the IWDG; HELLO_ACK's build flags covering every test
switch; every STATUS counter filled once; the STATE sums restarted every
period). The tools' own tests cover `rover_link.py` (the shared vectors, the
message layer, names checked against the C enums), the console's and the
logger's binary modes against fake ports, and `link_dump.py`.

**Mutation testing** (in WSL, from the repo root; about 2.5 min):
`python3 tools/run_link_mutants.py` plants one bug at a time in a temporary
copy (the tree is never touched): in the link's C code (rebuilt and run with
the same sanitizer flags in every suite that compiles the file), in
`main.c`'s wiring (killed by the source pins, a weaker check than a behaviour
test), and in the tools (their pytest file). It requires each one to make a
test fail; the unmutated baseline must pass first. **88 mutants, 88 killed**, 0 survived, 0 invalid,
on 2026-10-25 (75 of 75 on 2026-10-16). `python3 tools/run_link_mutants.py rx_` runs only the ids
containing `rx_`. `tools/test_run_link_mutants.py` (part of pytest) checks
that each mutant's text still occurs exactly once in its file.

## Serial monitor

ST-LINK VCP (the board's USB COM port), **115200 8N1**, no flow control.

For logging and to check line integrity, use `tools/serial_logger.py` at the
repo root (close the VS Code Serial Monitor first; only one program can hold the
port):

```
py -m pip install pyserial
py -m serial.tools.list_ports
py tools\serial_logger.py COM5 gyro_static.log
```

It writes the raw bytes unchanged to `gyro_static.log`, a host timestamp per
line to `gyro_static.log.times.csv`, and prints running good/bad line counts.

**Use a new log name every time.** Both `serial_logger.py` and
`motor_console.py` open the log for writing (`'wb'`), so an existing log of
the same name is **overwritten** without a warning.

### Motor console and QuickEdit

From the repo root, in a Command Prompt window:

```
py tools\motor_console.py COM5 C:\dev\rover-logs\<new-name>.log
```

It logs the same way and sends motor and steering commands, which **act only
on a bench or test build** (`BENCH_TEXT_COMMANDS 1`, or any test switch): the
committed truck build counts them in `cmd_bad`, and the console then says so
once (`cmd_bad` +3 with `cmd_ok` unchanged) and suggests `--binary`. Its loop is single-threaded on purpose: if the console
freezes, the keepalives stop, the firmware watchdog stops the motor and the
steering centres within 500 ms (a dead man's switch).

| Keys | What is sent |
|------|--------------|
| `v <n>` / `m <n>` / `s <n>` + Enter | the command at once, then every 100 ms as a keepalive. **`m` and `v` share one keepalive slot** (they feed the same firmware watchdog): the last one typed is the one resent, so typing `v 50` stops resending `m 20`. The motor slot and `s` each have their own schedule; nothing is sent until the first command, and no `s` unless you typed one |
| Esc | `m 0` at once and from then on; also `s 0` (centre) if steering was commanded |
| Ctrl+C | `m 0` ×3 (and `s 0` ×3 if steering was being sent), then quits |
| `-v` / `-m` / `-s` + Enter | test aid: stop that keepalive (`-v` and `-m` both stop the shared motor slot), to watch the firmware's motor watchdog or steering timeout act |
| `!text` + Enter | `text` once, as-is (to test rejection, or a test build's own letter: `!e 2`) |

A board reset (boot line) stops both keepalives; no `s 0` is sent after it,
because that would start pulses.

The status line is one row, redrawn in place, and never wider than the window
(a wrapped row couldn't be redrawn). It shows the logger's counts, what is
being sent, `IMU FAIL` if the board booted without its IMU, `MFAULT <code>
<reason>` when the motor is in FAULT (v5), `duty mstate d100`, the speed
control (v5: `open vest=..` under `m`, or `run`/`stop`/`GATE vset=.. vest=..
vi=..`, then `EST BAD` while `vok=0`, and `stall=` / `enc_bad=` once
non-zero), the steering, the battery (`vbat` in V and `bst`, with `NO BAT`,
`LOW BAT`, `CUTOFF`, `ADC FAULT` or `REMOVED WHILE DRIVING` in front when it
isn't OK), the decoded reset cause, and what you are typing. In a narrow
window the reset-cause decode and the counts are dropped first; the IMU and
fault markers, motor, speed control, steering and battery are always kept
(`status_line_text` in `tools/motor_console.py`). Examples:

```
… | sent: - | duty=0 mstate=2 d100=0 | s=0 us=0 sst=0 | vbat=7.90V bst=1 | rst=4 (PIN) | >
… | sent: - | duty=0 mstate=2 d100=0 | s=0 us=0 sst=0 | NO BAT vbat=0.01V bst=0 | rst=4 (PIN) | >
… | sent: v 50 | duty=294 mstate=1 d100=250 | run vset=50 vest=48 vi=293 | s=0 us=0 sst=0 | vbat=7.86V bst=1 | rst=4 (PIN) | >
… | sent: m 0 | MFAULT 7 REPEATED STALL | duty=0 mstate=3 d100=0 | open vest=0 stall=3 | s=0 us=0 sst=0 | vbat=7.87V bst=1 | rst=4 (PIN) | >
```

Everything after the first `|` is the console's own `status_line_text` output
for real lines: the first two from a 2026-10-14 v4 log (`unplug_decay_1.log`:
pack connected, then unplugged; nothing typed, matching `cmd_ok=0`), the last
two from 2026-10-15 v5 logs (`s2_creep_2.log` at `v 50`, and `s2_e3x3_2.log`
after the third `e 3` stall, with `m 0` being sent); the logger's counts are
shown as `…`.

In the classic Windows console window (conhost), **QuickEdit** mode makes a
mouse click or drag start a text selection that pauses the program's output.
That froze the console twice on the bench, and while a selection is active
Esc and Ctrl+C go to the selection, not to the program. So at startup the
console turns QuickEdit off for its own window (`SetConsoleMode` with
`ENABLE_EXTENDED_FLAGS` = 0x0080 and without `ENABLE_QUICK_EDIT_MODE` = 0x0040,
as [Microsoft documents](https://learn.microsoft.com/en-us/windows/console/setconsolemode)),
prints a note saying so, and puts the original mode back on every exit it
controls (Ctrl+C key, KeyboardInterrupt, any exception).

- **Not covered:** Edit → Mark from the window menu, and the Pause key, can
  still pause a classic console. The firmware watchdog then stops the motor.
- **Can't be turned off** (stdin isn't a console window, e.g. redirected, or
  the call fails): the console prints a note and carries on without the
  protection.
- **Windows Terminal:** selection belongs to the terminal, not to the console
  mode, so the flag probably does nothing there. Whether a selection pauses
  output in Windows Terminal is not documented that I could find (unverified).
  Its [selection docs](https://learn.microsoft.com/en-us/windows/terminal/selection)
  do say **Esc clears an active selection**, so with text selected the first
  Esc may only clear the selection instead of stopping the motor. Don't
  select text in the console while driving, whatever the terminal.
- **Task Manager kill:** nothing can run on the way out, so the `m 0` exit
  stops are not sent (the watchdog stops the motor within 500 ms) and the mode
  is not restored. The mode belongs to the console window, not the process, so
  that window stays without QuickEdit until it is closed (a shell started in
  it continues without mouse selection). Saved console defaults are not
  touched; new windows behave as usual.

### Motor console on Linux (the Pi, over SSH)

```
python3 tools/motor_console.py /dev/ttyACM0 ~/rover-logs/<new-name>.log --binary
```

Same commands, keys, status line and exit stops as on Windows (text mode
works the same way without `--binary`). The keys come from the terminal
(termios: no echo, each key at once; Ctrl+C arrives as a key, Ctrl+Z and
Ctrl+S do nothing), and the terminal settings come back on every way out the
console controls. Differences:

- **Arrows, Home/End, Delete, F-keys and Alt+key are Esc (stop) on Linux**;
  on Windows they are ignored. A Linux terminal sends them as Esc sequences,
  and a lone Esc differs from the start of one only in timing, so every Esc
  counts (`decode_keys`): a real Esc can never be lost.
- **SIGHUP** (the SSH session dropped, the window closed) and **SIGTERM**
  stop the console like Ctrl+C: ESTOP ×3 (or `m 0` ×3 in text mode) goes out
  before the port is closed. **SIGKILL** can't be caught: nothing is sent,
  and the firmware's 500 ms motor watchdog stops the motor
  (`MOTOR_COMMAND_TIMEOUT_MS`, `Core/Src/motor_control.c`).
- The port is opened with pyserial's `exclusive=True`, an advisory lock: a
  second `motor_console.py` can't open it, but `serial_logger.py` (which
  doesn't ask for the lock) still could. Run one program on the port.
- It needs a terminal: over SSH, log in and run it at the prompt, or use
  `ssh -t host '…'`. No QuickEdit on Linux.
- The user needs access to the port (`sudo usermod -aG dialout $USER`, then
  log in again).

### Binary mode tools (Pi link)

**Logging** without moving anything:

```
py tools\serial_logger.py COM5 C:\dev\rover-logs\<new-name>.log --binary
```

It sends HELLO (every 0.5 s until the HELLO_ACK, and again 0.1 s after a
board reset's boot line), never anything else, so the motor stays latched.
The raw bytes go to the log unchanged; `<log>.times.csv` gets one row per
frame (host time, piece number, `OK` or why it was dropped, type, seq). The
boot line and text lines a reset produces between frames are logged as
`TEXT`, not as bad frames.

**Disk stalls and shutdown.** Both tools, in both modes, keep their read
loop away from the disk:
- A writer thread writes the files. On 2026-10-16, a 2.5 s flush stall
  during a docker pull lost 1.6 s of frames, because the buffers before the
  port only hold about 0.9 s. If the disk falls more than 32 MB behind, data
  is dropped and a `WARNING: writing … that file is incomplete` line
  follows the summary, with exit code 1.
- The status line and the summary show `read_gap_max`, the longest time
  between two reads. It should stay well under 0.9 s.
- On Linux, `serial_logger.py` stops on SIGHUP (SSH dropped) and SIGTERM
  (Pi shutdown) as it does for Ctrl+C: the files are written to the end and
  the summary is printed.

**Stale data.** When the port is reopened while the board is already in
binary mode, the ST-LINK bridge first delivers what it held while the port
was closed: about 1–2 KB of frames with valid CRCs, possibly minutes old,
spliced onto the live stream. That gives one bad piece and one STATE and one
STATUS seq gap at the start (seen on the Pi on 2026-10-16, `b5_pi_drop_1` and
`b5_pi_after_drop_1`). So in both binary tools and `link_dump.py`,
everything before the first HELLO_ACK is **stale**:
- It stays in the raw log, and gets the result `STALE` in `<log>.times.csv`.
- It's counted on its own in the summary (`stale before HELLO_ACK: frames=…
  bad=… (… bytes)`), not as frames, bad pieces or gaps.
- It's never shown on the console's status line.

The tools send their first HELLO 0.1 s after opening the port, so the
leftover, including any old HELLO_ACK in it, arrives before the HELLO. As a
result, a session that starts in binary mode marks the first ~0.1 s of live
frames as stale too. Remaining risk: a leftover HELLO_ACK that arrives after
the HELLO would be taken as the answer.

**On the way out**, `motor_console.py` (both modes) keeps logging for 0.2 s
after its exit stops, so the log ends showing their effect (the latch). It
prints nothing meanwhile, and a failing port just ends it.

**Decoding** a raw binary log (from either tool):

```
py tools\link_dump.py C:\dev\rover-logs\<name>.log [--out <prefix>]
```

writes `<prefix>.state.csv`, `.status.csv` and `.hello_ack.csv` (`<prefix>`
is the log's path by default; columns: piece, seq, then every field by
name) and prints frames per type, bad pieces by reason, sequence gaps per
type, STATE `t_ms` steps other than 10 ms, boot and text lines, and the
stale part. Stale frames aren't in the CSV files. Exit code 0 only with no bad
piece and no gap after the first HELLO_ACK; 1 also for a log with no HELLO_ACK
at all (`no HELLO_ACK in this log`; a text-only log gives that too). Stale data
alone gives 0. The `piece` column matches
`<log>.times.csv`, so the two files join on it.

**Driving** over the link, as the Pi will:

```
py tools\motor_console.py COM5 C:\dev\rover-logs\<new-name>.log --binary
```

| Keys | What is sent |
|------|--------------|
| (start) | HELLO, until the board answers; the HELLO_ACK is shown, and says `TEST BUILD (…): never drive the truck with it` for a test build |
| `v <cps>` + Enter | speed setpoint in counts/s (−5000..5000; below 1000 coasts). One DRIVE carries speed and steering, every 20 ms (50 Hz), from the first `v` or `s` typed and only once the board has answered |
| `s <n>` + Enter | steering −100..100, sent with `steer_valid` in the same DRIVE |
| `arm` + Enter | one DRIVE 0 with the arm bit (the only thing that re-arms), then `v 0` keepalives |
| Esc / `estop` + Enter | ESTOP: latched at once, until `arm`; the keepalive goes on at `v 0` (Esc also centres the steering if it was commanded) |
| `hello` + Enter | HELLO again (latches the motor) |
| `-v` / `-s` + Enter | test aids: stop all DRIVEs (motor watchdog and steering timeout act within 500 ms) / DRIVEs without `steer_valid` (steering centres) |
| `inject <kind> <N>` + Enter | bench aid: N frames the firmware must drop: `crc`, `cobs`, `length`, `type`, `reserved`, `range`, `arm`; the console says which STATUS counter must grow by N. `range` and `arm` ask for motion if wrongly accepted, so inject them with the motor latched |
| `!text` + Enter | the text once (binary mode ignores it, except `e`, `c` and `h` in a test build: `!e 2`) |
| Ctrl+C | ESTOP ×3, then quits |

`m` is refused in binary mode (open loop is a text-mode bench command). A
board reset stops the DRIVEs until something is typed again; the console
sends HELLO again by itself. The status line (redrawn at most 10 times a
second) decodes STATE and STATUS: the motor state with its latch or fault
reason, duty, `vest` in counts/s, `cnt`, the odometer epoch `ep`, `EST BAD`,
`IMU FAIL`, the controller (`run vset=… vi=…`), steering, battery, stalls by
rule, the link's receive counters (`rx ok= crc= fr= rej= gap= txt=`) and
`loop=` (`loop_us_max`). In a narrow window the logger's counts go first,
then `loop`, the receive counters and what is being sent. Logs and the
times file are written as by `serial_logger.py --binary`.

## Pin map

Labels as in `rover_firmware.ioc` / `Core/Inc/main.h`.

| Pin       | Header | Function              | Notes                                   |
|-----------|--------|-----------------------|-----------------------------------------|
| PA9       | D1     | I2C1_SCL              | AS5600                                  |
| PA10      | D0     | I2C1_SDA              | AS5600                                  |
| PA2       | A7     | USART2_TX             | VCP_TX: ST-LINK VCP TX                  |
| PA15      | — (not on a header, VCP RX) | USART2_RX | VCP_RX: ST-LINK VCP RX         |
| PA3       | A2     | TIM15_CH2 (AF14, 50 Hz PWM) | SERVO_PWM → steering servo signal (no pulses until the first `s`) |
| PA6       | A5     | ADC1_IN11             | battery divider midpoint (see [divider](#battery-voltage-divider-2026-10-14)) |
| PB6       | D5     | GPIO analog           | `SB16_A5_KEEP_ANALOG`: joined to A5 by SB16. **Keep D5 empty** (see [D5](#d5-must-stay-empty)) |
| PB3       | D13 (LD3) | GPIO out           | LD3 green LED (heartbeat), on the board |
| PA1       | A1     | SPI1_SCK              | IMU                                     |
| PA7       | A6     | SPI1_MOSI             | IMU                                     |
| PB4       | D12    | SPI1_MISO             | IMU                                     |
| PA4       | A3     | GPIO out, starts high | IMU_CS (breakout has no CS pull-up)     |
| PB0       | D3     | GPIO in, pull-up      | IMU_INT1 (wired, not used yet)          |
| PA8       | D9     | TIM1_CH1 (20 kHz PWM) | MOTOR_PWM → TB6612 PWMA+PWMB            |
| PA11      | D10    | GPIO out, starts low  | MOTOR_IN1 → AIN1+BIN1                   |
| PA12      | D2     | GPIO out, starts low  | MOTOR_IN2 → AIN2+BIN2. **No jumper cap on D2/GND** (see [warning](#drive-motor-tb6612fng)) |
| PB1       | D6     | GPIO out, starts low  | MOTOR_STBY → STBY, **10 kΩ pull-down to GND** fitted |
| PA13/PA14 | –      | SWDIO/SWCLK           | **SWD: never reuse**, or you lose flashing/debugging |

The `.ioc` also reserves PA0 (MCO input from the ST-LINK) and PC14/PC15 (LSE
crystal) as part of the Nucleo board defaults. Neither is used right now (the
LSE is a candidate for correcting the clock; see the open item on clock
accuracy).

### Sensor wiring

Both sensors (and the TB6612's VCC) are powered by **one 3V3 pigtail**:
unplugging it takes down both sensors and the I2C pull-ups.

IMU (ICM-42688-P breakout, SPI1):

| IMU pin | Nucleo header | MCU pin |
|---------|---------------|---------|
| VCC | 3V3 | |
| GND | GND | |
| SCL/SCLK | A1 | PA1 |
| SDA/MOSI | A6 | PA7 |
| AD0/MISO | D12 | PB4 |
| CS | A3 | PA4 |
| INT1 | D3 | PB0 |
| INT2 | not connected | |

On the breadboard the jumpers were about 21 cm long (unverified on the
carrier), which is why SPI runs at only 2 MHz with slow IMU output edges.

AS5600 (I2C1, 400 kHz):

| AS5600 pin | Nucleo header | MCU pin |
|---------|---------------|---------|
| VCC | 3V3 | |
| GND | GND | |
| DIR | GND | |
| SDA | D0 | PA10 |
| SCL | D1 | PA9 |

The I2C pull-ups are on the AS5600 board.

## CubeMX regeneration rule

Regenerating code from the `.ioc` rewrites every generated file (`Core/Src/main.c`,
`stm32l4xx_it.c`, `cmake/stm32cubemx/CMakeLists.txt`, ...). **In generated files,
only code between `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */` survives.**
Anything outside those markers is silently lost. Files CubeMX doesn't generate
(`as5600`, `icm42688`, `telemetry_format`, `byte_ring`, `command_line`,
`command_dispatch`, `motor_control`, `tb6612`, `tb6612_plan`,
`steering_control`, `servo`, `loop_schedule`, `battery_monitor`,
`speed_capture`, `speed_estimator`, `angle_correction`, `speed_control`,
`stall_detect` and `encoder_fault`, each `.c` in `Core/Src` and `.h` in `Core/Inc`)
and the top-level `CMakeLists.txt` (generated only once) are safe. Add new
sources to the top-level `CMakeLists.txt` under `target_sources`.

Settings that must survive a regeneration (`tools/test_firmware_sources.py`
checks the IWDG ones, the ADC sampling time and PB6; the TIM15 ones are
checked by the firmware at run time):

- IWDG: prescaler 32, reload 249, window 4095, and **"Do Not Generate Function
  Call" ticked for `MX_IWDG_Init`** (Project Manager → Advanced Settings), with
  "Visibility (Static)" unticked. The IWDG is started from `USER CODE 2`
  instead, after its debug freeze. If CubeMX generated the call again, the
  watchdog would start twice and before the freeze.
- TIM15: PSC 31, ARR 19999, PWM mode 1 on CH2, compare 0, ARR preload on. If
  they change, the servo guard refuses to pulse (`sstate=3`).
- ADC1: regular conversion disabled; injected group with 2 ranks, rank 1
  VREFINT and rank 2 IN11 (PA6), 640.5 cycles each, software trigger; ADC
  clock = SYSCLK. The firmware reads rank 1 as VREFINT and rank 2 as the
  battery, so swapping them would give nonsense voltages (most likely an ADC
  fault, which locks the motor out; reasoned from the conversion math, not
  tested).
- PB6: **GPIO_Analog**, label `SB16_A5_KEEP_ANALOG` (see
  [D5 must stay empty](#d5-must-stay-empty)).
