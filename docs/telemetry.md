# Run telemetry (Week 8)

Record a run, import it into Postgres after the run, and look at it on one Streamlit page: the
EKF's path overlaid with the sim's run of the same route, battery voltage over time, speed over
time, and a summary. No live streaming.

    record (ros2 bag + run.json)  ->  import_run.py  ->  Postgres (rover)  ->  dashboard.py
    tools/record_run.sh               tools/import_run.py                      tools/dashboard.sh

| Piece | Where it runs | Needs |
|---|---|---|
| `tools/record_run.sh real ...` | Pi, inside the `rover-bridge` container | the compose services up |
| `tools/record_run.sh sim ...` | WSL, next to the sim (domain 77) | ROS sourced |
| `tools/import_run.py` | WSL, system Python | ROS sourced (rosbag2_py), apt `python3-psycopg` |
| Postgres 16, database `rover` | WSL (apt, systemd service) | peer auth: the WSL user's own role |
| `tools/dashboard.sh` | WSL, venv `~/venvs/telemetry` | `tools/requirements-dashboard.txt` |

The importer and the page run on different Pythons on purpose: Streamlit brings numpy 2.x, and
ROS's Python messages (their fixed-size arrays, such as the covariances, are numpy arrays) are
built against Ubuntu's numpy 1.26. The venv, once:

    python3 -m venv ~/venvs/telemetry && ~/venvs/telemetry/bin/pip install -r tools/requirements-dashboard.txt

## Recording

`ros2 bag record` of `/odom` (50 Hz) and `/rover/battery` (1 Hz, latched), plus
`/odometry/ground_truth` in the sim. It's one more subscriber, so the bridge and the EKF run
exactly as without it. MCAP by default, `--storage sqlite3` if the Pi image lacks the plugin
(check: `docker exec rover-bridge /ros_entrypoint.sh ros2 bag record --help | grep -A3 storage`).
Keyboard controls are off, so a Space in the recording terminal doesn't pause it. When the bag
ends, the script writes `run.json` into it: name, route, source, start time (with its UTC offset)
and the git commit (`-dirty` with uncommitted changes to tracked files).

**Real run** (one half circle is the minimal bench), on the Pi from `~/autonomous-rover`:

1. Clean start: restart the EKF and leave the truck untouched until the run ends. (Teleop **R**
   arms the motor; it is not a reset.)

        docker compose -f ros_ws/docker/compose.yaml restart ekf && sleep 20 && \
            docker compose -f ros_ws/docker/compose.yaml logs --tail 3 ekf

   The log should show the EKF started (its bias line) after 2 s at rest.
2. Start the recording, before driving:

        tools/record_run.sh real half_circle            # or: --storage sqlite3

   For the gyro's own heading, also start `tools/gyro_turn_check.py` in another terminal (its
   docstring has the `docker exec` line); it integrates the gyro until Ctrl-C.
3. Drive the route with teleop (docs/ros-bridge.md). After **Space**, stop the recording with
   **Ctrl-C** in its terminal; the bag is in `~/rover-logs/<name>`. Stop `gyro_turn_check.py` too:
   its `gyro (header)` total is the gyro heading.
4. Measure, if wanted: tape the chord, the straight-line distance from the start mark to the end
   mark (about 1.29 m for the half circle). The chord and the gyro heading go in at import.
5. Copy it to WSL:

        scp -r armando@192.168.1.18:rover-logs/<name> ~/rover-runs/

The recording runs on the Pi, so the 50 Hz `/odom` never crosses Wi-Fi (gaps up to ~600 ms there).
Check that the bridge's `cpu=` status line looks the same during the recording.

**Sim run**, in WSL, with the sim and the EKF up as in docs/kalman-filter.md (domain 77): in a
fourth terminal with the same environment,

    tools/record_run.sh sim half_circle                 # bag in ~/rover-runs/<name>

then `python3 tools/sim_route.py half_circle` in another, and Ctrl-C the recording after the route
prints its summary. `sim_route.py` resets the EKF itself, so both runs start at (0, 0, 0).

## Importing

    source /opt/ros/jazzy/setup.bash
    python3 tools/import_run.py ~/rover-runs/<name> [--tape-chord 1.186] [--gyro-heading 2.969]

It applies `tools/telemetry_schema.sql` (only what's missing), reads `/odom` and `/rover/battery`
by their header stamps, and loads the run in one transaction. Importing the same recording again
replaces it, the measured values included (pass them again). It refuses, and writes nothing, when a stream
has two messages with the same header stamp, when there's no `/odom`, when `run.json` is missing or
wrong, or when the bag has more than one data file (a split bag).

Synthetic runs to try the page without a bag (`synthetic_*`, replaced on every run):

    python3 tools/telemetry_synthetic.py
    psql -d rover -c "DELETE FROM runs WHERE name LIKE 'synthetic_%'"     # to remove them

## The page

    tools/dashboard.sh                                  # http://127.0.0.1:8501, Ctrl-C to stop

It opens on the latest real run (the latest run when there is no real one). For a real run, the
latest sim run of the same route is overlaid by default; pick another or none. Plots:

- the path at true shape: one metre is the same number of pixels on x and y, and 0 is on both axes;
- speed (the EKF's `twist.linear.x`) and, for a real run with a pack, battery voltage, on one time
  axis: from 0, or from the latched battery sample before it, to the last sample shown.

The summary has, for each run's EKF, the end point, the chord (straight-line distance from the
start to the end point), heading, distance (path length) and duration. Next to them, when given at
import: the taped chord beside the EKF's chord, and the gyro heading beside the EKF's heading.
`ROVER_DB` overrides the connection string (default `dbname=rover`).

### Viewing from Windows

Open **http://127.0.0.1:8501**, not `localhost`. Windows resolves `localhost` to `::1` first,
Streamlit listens on IPv4 127.0.0.1 only, and PowerShell 5.1 hangs on `::1` instead of falling
back (a browser may fall back, 127.0.0.1 always works). Check from PowerShell:

    Invoke-WebRequest -UseBasicParsing http://127.0.0.1:8501/_stcore/health     # 200, ok

WSL here runs mirrored networking (`.wslconfig`), and its Hyper-V firewall blocks inbound by
default (`Get-NetFirewallHyperVVMSetting`: DefaultInboundAction Block). One rule lets port 8501 in
(added 2026-10-25): `RoverStreamlit8501`, inbound TCP 8501, for WSL's VMCreatorId
`{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}`, remote addresses Any. That's safe while
`dashboard.sh` binds localhost; bound to all interfaces, the LAN could reach the page. To see it
or remove it (admin PowerShell for the removal):

    Get-NetFirewallHyperVRule -Name RoverStreamlit8501
    Remove-NetFirewallHyperVRule -Name RoverStreamlit8501

## First recorded runs

The first real and sim recordings of the half circle (2026-10-25, runs 5 and 4) and what they
show: docs/kalman-filter.md, "Recorded half circle (2026-10-25, telemetry)".

## Schema (tools/telemetry_schema.sql)

    runs(run_id PK identity, name, route, source, started_at timestamptz, git_commit,
         recording_sha256 UNIQUE, tape_chord_m >= 0, gyro_heading_rad, imported_at)
    odom_samples(run_id FK, t_ns bigint, x_m, y_m, heading_rad, speed_mps,  PK (run_id, t_ns))
    battery_samples(run_id FK, t_ns bigint, voltage_v NULL, present,         PK (run_id, t_ns))

The decisions, and what each costs:

- **A table per stream, not one long samples table.** The signals are fixed by the ROS message
  types, and each stream has its own rate. One row per message keeps the queries plain selects
  and gives each column its type, unit and NULL meaning ("no pack" is `present false`, voltage
  `NULL`, not a fake 0 V). The cost: a new signal is a migration, not a new `signal` value.
- **Time is `t_ns bigint`**: integer nanoseconds since the run's first `/odom` header stamp. It
  is exact, so it can be the key. A double of epoch seconds keeps only ~0.2 µs, and two stamps
  could round to one. Seconds for plotting are `t_ns / 1e9`. Two messages of one stream with the
  same stamp fail the import loudly instead of one being dropped: either the stamping or the
  recording is wrong, and that has to be seen. The latched battery sample can come from before
  the first `/odom`, so `t_ns` can be negative.
- **Relative time, plus `runs.started_at`.** Real stamps are the Pi's clock (the board-clock fit);
  sim stamps are sim time, which starts near zero. Relative time puts both on one axis; the wall
  clock of the recording is kept once per run.
- **The run's identity is the sha256 of the bag's data file**, `UNIQUE`. The same recording can't
  become two runs under two names. Re-import is *replace*: delete the run (its samples cascade) and
  insert it again, in one transaction, so a fixed importer can refresh an old run and a failure
  leaves the old one in place. The cost: `run_id` changes on every re-import.
- **The primary key `(run_id, t_ns)` is the only index.** Every read is "one run, in time order",
  which is exactly that index's order. A half circle is ~2,200 odom rows; nothing is downsampled.
- **Pairing by route**: the page picks the latest sim run of the same route, and you can override
  it. There is no pairing column to keep up to date; the cost is that "which sim" is a choice on
  the page, not a fact in the database.
- **What was measured is two nullable columns on `runs`**: `tape_chord_m`, the taped straight-line
  distance from the start mark to the end mark, and `gyro_heading_rad`, the turn integrated by
  `tools/gyro_turn_check.py`. These are what's actually measured at the bench: a chord is one tape
  pull, while an end point in x and y needs the start frame laid out on the floor. Two numbers
  don't need their own table. The cost: a new kind of measurement is a migration.

The page's queries are in `tools/telemetry_queries.py`. The distance is the path length: each
step's length first (`lag()` over `t_ns`, in a CTE), then the sum, because a window function
can't sit inside an aggregate.

## Tests

- `tools/test_import_run.py`: 23 tests without ROS or Postgres (fake messages): the stamps,
  yaw and unwrap, relative time, NaN to NULL, duplicate stamps, `run.json`, the data file and its
  hash. In the `python-tests` job like the other tools.
- `tools/test_telemetry_db.py`: 7 tests against a real Postgres, each in a schema of its own that
  is dropped afterwards, so a real database is safe: load, replace on re-import, a negative taped
  chord is refused, a failed import changes nothing, the latest sim of a route, seconds from
  `t_ns`, and the summary (path length, end point, chord) on a known path.
  Skipped unless `TELEMETRY_TEST_DB` is set. CI job `telemetry-db` sets it, against a postgres:16
  service. Locally:

        TELEMETRY_TEST_DB="dbname=rover" python3 -m pytest -q tools/test_telemetry_db.py

- **Failure control** (2026-10-25, before the `telemetry-db` job was committed), on a copy of the
  code outside the tree: with re-import changed to insert without deleting,
  `test_importing_the_same_recording_again_replaces_it` failed and pytest exited 1. With the
  `UNIQUE` on `recording_sha256` also removed, so the duplicate run really lands, the same test
  failed on finding two runs. A re-import that stops replacing fails the job.
