#!/usr/bin/env bash
# Record one run as a ros2 bag, plus run.json (name, route, source, start time, git commit) for
# tools/import_run.py (docs/telemetry.md). Records /odom and /rover/battery (and, in the sim,
# /odometry/ground_truth); it only subscribes, so the bridge and the EKF run as they do without it.
#
#   tools/record_run.sh real|sim straight|half_circle [--storage mcap|sqlite3] [--out DIR (sim only)]
#
# real: on the Pi, from ~/autonomous-rover, with the compose services up. The bag is recorded
#   inside the rover-bridge container (on the Pi, so the 50 Hz /odom never crosses Wi-Fi), into
#   ~/rover-logs/<name> (its /logs). Clean start for a measured run: restart the EKF, then don't
#   touch the truck until the run ends (teleop R arms the motor; it is not a reset):
#       docker compose -f ros_ws/docker/compose.yaml restart ekf && sleep 20 && \
#           docker compose -f ros_ws/docker/compose.yaml logs --tail 3 ekf
#   Then start this, drive, and after Space stop it with Ctrl-C. Copy it to WSL:
#       scp -r armando@192.168.1.18:rover-logs/<name> ~/rover-runs/
#   What's measured goes in at import (tools/import_run.py --tape-chord --gyro-heading): the
#   taped chord, start mark to end mark, and the gyro heading from tools/gyro_turn_check.py,
#   started next to this and stopped after Space.
# sim: in WSL, with the sim and the EKF up (docs/kalman-filter.md, domain 77) and ROS sourced.
#   Start this before tools/sim_route.py, stop it with Ctrl-C when the route has printed its
#   summary. The bag goes into ~/rover-runs/<name>.
#
# --storage: mcap (default) or sqlite3, if the image has no MCAP plugin
#   (check: docker exec rover-bridge /ros_entrypoint.sh ros2 bag record --help | grep -A3 storage).
set -euo pipefail

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 2; }

[[ $# -ge 2 ]] || usage
source_kind=$1 route=$2
shift 2
storage=mcap out=
while [[ $# -gt 0 ]]; do
  case $1 in
    --storage) storage=${2:?}; shift 2 ;;
    --out) out=${2:?}; shift 2 ;;
    *) usage ;;
  esac
done
[[ $source_kind == real || $source_kind == sim ]] || usage
[[ $route == straight || $route == half_circle ]] || usage
[[ $storage == mcap || $storage == sqlite3 ]] || usage

repo=$(cd "$(dirname "$0")/.." && pwd)
commit=$(git -C "$repo" rev-parse --short HEAD)
[[ -z $(git -C "$repo" status --porcelain --untracked-files=no) ]] || commit+=-dirty
started_at=$(date --iso-8601=seconds)
name=${source_kind}_${route}_$(date +%Y%m%d_%H%M%S)

# Ctrl-C is for ros2 bag (it closes the bag cleanly); this script carries on to write run.json.
trap ':' INT
if [[ $source_kind == real ]]; then
  [[ -z $out ]] || { echo "--out is for sim runs: the container writes ~/rover-logs" >&2; exit 2; }
  out=$HOME/rover-logs
  echo "recording $name into $out (Ctrl-C to stop)"
  docker exec -it rover-bridge /ros_entrypoint.sh \
    ros2 bag record -s "$storage" --disable-keyboard-controls -o "/logs/$name" \
    --topics /odom /rover/battery || true
else
  out=${out:-$HOME/rover-runs}
  mkdir -p "$out"
  echo "recording $name into $out (Ctrl-C to stop)"
  ros2 bag record -s "$storage" --disable-keyboard-controls -o "$out/$name" \
    --topics /odom /rover/battery /odometry/ground_truth || true
fi
trap - INT

[[ -d $out/$name ]] || { echo "no bag at $out/$name: nothing recorded" >&2; exit 1; }
cat > "$out/$name/run.json" <<EOF
{"name": "$name", "route": "$route", "source": "$source_kind", "started_at": "$started_at", "git_commit": "$commit"}
EOF
echo "$out/$name: $(ls "$out/$name" | tr '\n' ' ')"
