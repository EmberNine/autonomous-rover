"""Makes the repo's tools/ importable (rover_link.py, serial_logger.py: stdlib only).

The bridge reuses them as they are instead of a copy. In the container,
PYTHONPATH has /repo/tools (ros_ws/docker/compose.yaml). Otherwise
ROVER_TOOLS_DIR, or tools/ next to ros_ws/ when running from the source tree.
"""
import os
import sys
from pathlib import Path


def add_tools_to_path():
    try:
        import rover_link  # noqa: F401
        return
    except ImportError:
        pass
    candidates = [os.environ.get('ROVER_TOOLS_DIR'), Path(__file__).resolve().parents[4] / 'tools']
    for candidate in candidates:
        if candidate and (Path(candidate) / 'rover_link.py').is_file():
            sys.path.insert(0, str(candidate))
            return
    raise ImportError('rover_link.py not found: put the repo\'s tools/ on PYTHONPATH or set ROVER_TOOLS_DIR')


add_tools_to_path()
