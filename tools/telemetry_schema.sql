-- Run telemetry (docs/telemetry.md): one row per recorded run, one row per message of each
-- stream. Applied by tools/import_run.py before every import, so it only ever adds what is
-- missing (IF NOT EXISTS). No schema-qualified names: the integration test applies it in a
-- schema of its own (search_path).

CREATE TABLE IF NOT EXISTS runs (
  run_id           bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
  name             text NOT NULL,
  route            text NOT NULL CHECK (route IN ('straight', 'half_circle')),
  source           text NOT NULL CHECK (source IN ('real', 'sim')),
  started_at       timestamptz NOT NULL,          -- wall clock when the recording started (run.json)
  git_commit       text,
  -- sha256 of the recording's data file (.mcap or .db3): the same recording is one run, and
  -- importing it again replaces that run (tools/import_run.py).
  recording_sha256 text NOT NULL UNIQUE CHECK (recording_sha256 ~ '^[0-9a-f]{64}$'),
  -- Measured at the end, NULL when not measured: the taped straight-line distance from the start
  -- mark to the end mark, and the turn from the gyro (tools/gyro_turn_check.py).
  tape_chord_m     double precision CHECK (tape_chord_m >= 0),
  gyro_heading_rad double precision,
  imported_at      timestamptz NOT NULL DEFAULT now()
);

-- Time is t_ns: integer nanoseconds since the run's first /odom header stamp, exact (a double
-- of epoch seconds keeps only ~0.2 us). Seconds for plotting are t_ns / 1e9.
CREATE TABLE IF NOT EXISTS odom_samples (
  run_id      bigint NOT NULL REFERENCES runs ON DELETE CASCADE,
  t_ns        bigint NOT NULL,
  x_m         double precision NOT NULL,
  y_m         double precision NOT NULL,
  heading_rad double precision NOT NULL,         -- unwrapped: a half circle may cross +-pi
  speed_mps   double precision NOT NULL,         -- the EKF's twist.linear.x
  PRIMARY KEY (run_id, t_ns)
);

-- /rover/battery at 1 Hz. It is latched (transient local), so the first sample can come from
-- before the first /odom: a negative t_ns. No pack (wall power): present false, voltage NULL.
CREATE TABLE IF NOT EXISTS battery_samples (
  run_id    bigint NOT NULL REFERENCES runs ON DELETE CASCADE,
  t_ns      bigint NOT NULL,
  voltage_v real,
  present   boolean NOT NULL,
  PRIMARY KEY (run_id, t_ns)
);
