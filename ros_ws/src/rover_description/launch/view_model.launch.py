#!/usr/bin/env python3
"""Display rover.urdf.xacro in RViz, with dimensions from a vehicle.cfg file.

Starts robot_state_publisher (publishing /robot_description and TF),
joint_state_publisher_gui (sliders to move every joint), and rviz2.

No vehicle dimension is hardcoded here: everything the URDF needs comes
from vehicle_cfg.py parsing the config file named by the `vehicle_config`
launch argument.
"""

import sys
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# vehicle_cfg.py is a sibling file in this same launch/ directory, not part
# of an importable Python package (rover_description is a plain
# ament_cmake package, with no setup.py or __init__.py to put it on
# PYTHONPATH). `ros2 launch` loads this file directly by path rather than
# importing it the way `python3 script.py` would, so nothing adds this
# directory to sys.path automatically -- we do it ourselves.
#
# .resolve() follows symlinks before taking the parent directory. That
# matters because `colcon build --symlink-install` installs this file (and
# vehicle_cfg.py) as symlinks back into ros_ws/src/rover_description/launch/;
# resolving first means sys.path always ends up pointing at the real
# directory that contains both files, whether this launch file is run from
# the install tree or straight out of the source tree.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vehicle_cfg import VehicleConfigError, load_vehicle_urdf_dimensions  # noqa: E402


def _default_vehicle_config_path() -> str:
    """The vehicle.cfg installed with this package, used unless overridden.

    This is the symlinked copy installed by rover_description's
    CMakeLists.txt (`install(FILES .../config/vehicle.cfg ...)`), which
    with --symlink-install points back at the repo's single source of
    truth rather than a stale copy.
    """
    return str(
        Path(get_package_share_directory("rover_description")) / "config" / "vehicle.cfg"
    )


def robot_description_xml(vehicle_config_path: str) -> str:
    """The URDF for a vehicle.cfg file (also used by pose.launch.py)."""
    # A missing/malformed/invalid cfg raises VehicleConfigError here, which
    # aborts the launch with a message naming the bad key and the file --
    # the whole point of validating before handing values to xacro.
    try:
        dimensions = load_vehicle_urdf_dimensions(vehicle_config_path)
    except VehicleConfigError as config_error:
        raise RuntimeError(f"Failed to load vehicle config: {config_error}") from config_error

    xacro_file_path = str(
        Path(get_package_share_directory("rover_description")) / "urdf" / "rover.urdf.xacro"
    )
    # xacro mappings are plain string substitutions for <xacro:arg> values,
    # so every float must be stringified.
    xacro_mappings = {
        "wheelbase_meters": str(dimensions.wheelbase_meters),
        "front_track_width_meters": str(dimensions.front_track_width_meters),
        "rear_track_width_meters": str(dimensions.rear_track_width_meters),
        "tire_diameter_meters": str(dimensions.tire_diameter_meters),
        "max_steering_angle_radians": str(dimensions.max_steering_angle_radians),
        "vehicle_mass_kilograms": str(dimensions.vehicle_mass_kilograms),
        "front_wheel_mass_kilograms": str(dimensions.front_wheel_mass_kilograms),
        "rear_wheel_mass_kilograms": str(dimensions.rear_wheel_mass_kilograms),
    }
    return xacro.process_file(xacro_file_path, mappings=xacro_mappings).toxml()


def _launch_setup(context, *args, **kwargs):
    """Build the model and nodes once the `vehicle_config` argument is resolved.

    This runs inside an OpaqueFunction because LaunchConfiguration values
    (including a user-supplied `vehicle_config:=...`) are only available
    once the launch system has resolved substitutions against `context`;
    they cannot be read at LaunchDescription-construction time.
    """
    vehicle_config_path = LaunchConfiguration("vehicle_config").perform(context)
    robot_description_xml_text = robot_description_xml(vehicle_config_path)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml_text}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz_config_path = str(
        Path(get_package_share_directory("rover_description")) / "rviz" / "view_model.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return [robot_state_publisher_node, joint_state_publisher_gui_node, rviz_node]


def generate_launch_description() -> LaunchDescription:
    """Declare the `vehicle_config` argument and defer the rest to OpaqueFunction."""
    vehicle_config_argument = DeclareLaunchArgument(
        "vehicle_config",
        default_value=_default_vehicle_config_path(),
        description="Path to a vehicle.cfg-formatted file supplying the URDF dimensions.",
    )

    return LaunchDescription([vehicle_config_argument, OpaqueFunction(function=_launch_setup)])
