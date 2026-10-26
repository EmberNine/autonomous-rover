"""The package (ros_ws/src/rover_bridge) on sys.path, so the tests run from the repo root
without a colcon build: python3 -m pytest ros_ws/src/rover_bridge/test (rover_bridge._tools
finds tools/ next to ros_ws/)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
