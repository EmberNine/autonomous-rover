"""The run telemetry page (docs/telemetry.md): one run from Postgres, its path overlaid with the
sim's run of the same route, battery voltage, speed, and a summary. Start it with
tools/dashboard.sh (Streamlit, in the ~/venvs/telemetry venv); ROVER_DB is the libpq connection
string (default dbname=rover).
"""
import os

import altair as alt
import pandas as pd
import psycopg
import streamlit as st

import telemetry_queries as queries

st.set_page_config(page_title='Rover runs', layout='wide')


@st.cache_resource
def connect():
    return psycopg.connect(os.environ.get('ROVER_DB', 'dbname=rover'), autocommit=True)


def label(run):
    return f"{run['name']} ({run['source']}, {run['started_at']:%Y-%m-%d %H:%M})"


connection = connect()
runs = queries.runs(connection)
st.title('Rover runs')
if not runs:
    st.info('No runs yet: record one with tools/record_run.sh, import it with tools/import_run.py.')
    st.stop()

PATH_PIXELS = 460            # the path plot's longer side
PATH_PAD_M = 0.1
PATH_MIN_ASPECT = 0.4        # the shorter range is padded to at least this share of the longer
TIME_PLOT_HEIGHT = 200


def path_domain(values, span):
    """[low, high] with 0 inside, padded, and centred out to at least span."""
    low, high = min(0.0, values.min()) - PATH_PAD_M, max(0.0, values.max()) + PATH_PAD_M
    extra = max(0.0, span - (high - low)) / 2
    return [low - extra, high + extra]


# The widgets hold run_ids (plain ints keep their identity across reruns), shown by label.
# The default is the latest real run (runs come latest first), else the latest run.
by_id = {run['run_id']: run for run in runs}
ids = list(by_id)
default = next((index for index, run_id in enumerate(ids) if by_id[run_id]['source'] == 'real'), 0)
left, right = st.columns(2)
run = by_id[left.selectbox('Run', ids, index=default, format_func=lambda run_id: label(by_id[run_id]))]
sims = [sim['run_id'] for sim in queries.sim_runs(connection, run['route']) if sim['run_id'] != run['run_id']]
compare = None
if run['source'] == 'real':
    # D4: the latest sim run of the same route by default; any other, or none, from the list.
    # 0 is "none": run_ids start at 1.
    compare_id = right.selectbox(f"Sim of {run['route']} to overlay", [0] + sims, index=1 if sims else 0,
                                 format_func=lambda run_id: 'none' if run_id == 0 else label(by_id[run_id]))
    compare = by_id.get(compare_id)

names = {run['run_id']: f"{run['source']}: {run['name']}"}
if compare:
    names[compare['run_id']] = f"sim: {compare['name']}"
odom = pd.DataFrame(queries.odom(connection, list(names)))
odom['run'] = odom['run_id'].map(names)
battery = pd.DataFrame(queries.battery(connection, run['run_id']), columns=['t_s', 'voltage_v', 'present'])
battery = battery.dropna(subset=['voltage_v'])
legend = alt.Legend(orient='bottom', direction='vertical', labelLimit=400)
# Streamlit's default autosize is "fit": axes and legend are squeezed into the size, which shrinks
# the plot and clips its left edge. "pad" keeps the size as the plot area; "fit-x" only fits the
# width to the column.
PAD = alt.AutoSizeParams(type='pad')
FIT_X = alt.AutoSizeParams(type='fit-x', contains='padding')

# The path at true shape: one metre is the same number of pixels on both axes. Numeric
# width/height are the plot area, and nice=False keeps the domains exactly as computed.
longer = max(high - low for low, high in (path_domain(odom['x_m'], 0), path_domain(odom['y_m'], 0)))
x_domain = path_domain(odom['x_m'], PATH_MIN_ASPECT * longer)
y_domain = path_domain(odom['y_m'], PATH_MIN_ASPECT * longer)
pixels_per_meter = PATH_PIXELS / longer
path = alt.Chart(odom).mark_line().encode(
    x=alt.X('x_m', title='x (m)', axis=alt.Axis(labelOverlap=True, labelSeparation=8),
            scale=alt.Scale(domain=x_domain, nice=False, zero=False)),
    y=alt.Y('y_m', title='y (m)', axis=alt.Axis(labelOverlap=True, labelSeparation=8),
            scale=alt.Scale(domain=y_domain, nice=False, zero=False)),
    color=alt.Color('run', title=None, legend=legend), order='t_s').properties(autosize=PAD,
        width=round((x_domain[1] - x_domain[0]) * pixels_per_meter),
        height=round((y_domain[1] - y_domain[0]) * pixels_per_meter))

# Speed and battery on one time axis: from 0 (or the latched battery sample before it) to the
# last sample of the runs shown.
t_start = min([0.0] + battery['t_s'].tolist())
t_end = max(odom['t_s'].max(), battery['t_s'].max() if len(battery) else 0.0)
time_axis = alt.X('t_s', title='t (s)', scale=alt.Scale(domain=[t_start, t_end], nice=False))
speed = alt.Chart(odom).mark_line().encode(
    x=time_axis, y=alt.Y('speed_mps', title='speed (m/s)'),
    color=alt.Color('run', title=None, legend=legend)).properties(height=TIME_PLOT_HEIGHT, autosize=FIT_X)

# Titles as text above the charts (a Vega title is clipped on the left in Streamlit), and no
# Streamlit theme, so the sizes above are the sizes drawn.
plots, numbers = st.columns([3, 2])
plots.markdown('**Path** (EKF /odom)')
plots.altair_chart(path, theme=None)
plots.markdown('**Speed** (EKF)')
plots.altair_chart(speed, theme=None, use_container_width=True)
plots.markdown('**Battery** (/rover/battery)')
if len(battery):
    plots.altair_chart(alt.Chart(battery).mark_line(point=True).encode(
        x=time_axis, y=alt.Y('voltage_v', title='voltage (V)', scale=alt.Scale(zero=False))).properties(
            height=TIME_PLOT_HEIGHT, autosize=FIT_X), theme=None, use_container_width=True)
else:
    plots.info('No battery voltage in this run (a sim run, or no pack: wall power).')

# The summary: each run's EKF numbers, and what was measured beside them: the taped start-to-end
# chord next to the EKF's chord, the gyro's heading (tools/gyro_turn_check.py) next to the EKF's.
numbers.subheader('Summary')
columns = {}
for run_id in names:                 # short column names (the runs are named in the pickers)
    summary = queries.summary(connection, run_id)
    columns[f"{by_id[run_id]['source']} EKF"] = {
        'end x (m)': summary['end_x_m'], 'end y (m)': summary['end_y_m'], 'chord (m)': summary['chord_m'],
        'heading (rad)': summary['end_heading_rad'], 'distance (m)': summary['distance_m'],
        'duration (s)': summary['duration_s']}
if run['tape_chord_m'] is not None or run['gyro_heading_rad'] is not None:
    columns['tape / gyro'] = {'chord (m)': run['tape_chord_m'], 'heading (rad)': run['gyro_heading_rad']}
metrics = list(next(iter(columns.values())))
# st.table, not st.dataframe: the grid shows a missing value as "None" whatever the Styler says.
numbers.table(pd.DataFrame(columns, dtype=float).reindex(metrics).style.format(precision=3, na_rep='-'))
numbers.caption(f"route {run['route']}, commit {run['git_commit'] or '-'}, run_id {run['run_id']}")
