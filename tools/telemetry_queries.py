"""The run telemetry page's queries (tools/dashboard.py), against tools/telemetry_schema.sql.

Plain functions over a psycopg connection, so the integration test (tools/test_telemetry_db.py)
runs the same SQL as the page. Seconds are derived from t_ns as t_ns / 1e9 (as float8, not
numeric, so they come back as floats).
"""

RUNS = """
SELECT run_id, name, route, source, started_at, git_commit,
       tape_chord_m, gyro_heading_rad
FROM runs
ORDER BY started_at DESC, run_id DESC
"""

# D4: the sim runs of a route, latest first; the page defaults to the first.
SIM_RUNS = """
SELECT run_id, name, started_at
FROM runs
WHERE source = 'sim' AND route = %(route)s
ORDER BY started_at DESC, run_id DESC
"""

ODOM = """
SELECT run_id, t_ns / 1e9::float8 AS t_s, x_m, y_m, heading_rad, speed_mps
FROM odom_samples
WHERE run_id = ANY(%(run_ids)s)
ORDER BY run_id, t_ns
"""

BATTERY = """
SELECT t_ns / 1e9::float8 AS t_s, voltage_v, present
FROM battery_samples
WHERE run_id = %(run_id)s
ORDER BY t_ns
"""

# Distance is the path length: each step's length first (lag() over time), then the sum; a
# window function can't sit inside an aggregate. The end point and heading are the last sample;
# the chord is the straight line from the start (0, 0) to it, what the tape measures.
SUMMARY = """
WITH steps AS (
  SELECT t_ns, x_m, y_m, heading_rad,
         sqrt((x_m - lag(x_m) OVER w) ^ 2 + (y_m - lag(y_m) OVER w) ^ 2) AS step_m
  FROM odom_samples
  WHERE run_id = %(run_id)s
  WINDOW w AS (ORDER BY t_ns)
),
last AS (
  SELECT x_m, y_m, heading_rad, t_ns FROM steps ORDER BY t_ns DESC LIMIT 1
)
SELECT last.x_m AS end_x_m, last.y_m AS end_y_m, last.heading_rad AS end_heading_rad,
       sqrt(last.x_m ^ 2 + last.y_m ^ 2) AS chord_m,
       last.t_ns / 1e9::float8 AS duration_s,
       (SELECT coalesce(sum(step_m), 0) FROM steps) AS distance_m,
       (SELECT count(*) FROM steps) AS samples
FROM last
"""


def _dicts(connection, query, params=None):
    with connection.cursor() as cursor:
        cursor.execute(query, params)
        names = [column.name for column in cursor.description]
        return [dict(zip(names, row)) for row in cursor.fetchall()]


def runs(connection):
    return _dicts(connection, RUNS)


def sim_runs(connection, route):
    return _dicts(connection, SIM_RUNS, {'route': route})


def odom(connection, run_ids):
    return _dicts(connection, ODOM, {'run_ids': list(run_ids)})


def battery(connection, run_id):
    return _dicts(connection, BATTERY, {'run_id': run_id})


def summary(connection, run_id):
    """End point, heading, duration and distance of one run; None if it has no samples."""
    rows = _dicts(connection, SUMMARY, {'run_id': run_id})
    return rows[0] if rows else None
