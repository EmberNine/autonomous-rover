#!/usr/bin/env bash
# The run telemetry page (tools/dashboard.py, docs/telemetry.md), in WSL, on localhost:8501;
# Ctrl-C stops it. With WSL's mirrored networking the Windows browser can't reach it until the
# Hyper-V firewall lets it in (docs/telemetry.md, "Viewing from Windows").
#   tools/dashboard.sh [streamlit options, e.g. --server.port 8502]
# The venv once: python3 -m venv ~/venvs/telemetry && ~/venvs/telemetry/bin/pip install -r tools/requirements-dashboard.txt
set -euo pipefail
exec "$HOME/venvs/telemetry/bin/streamlit" run "$(dirname "$0")/dashboard.py" \
  --server.address localhost --server.headless true --browser.gatherUsageStats false "$@"
