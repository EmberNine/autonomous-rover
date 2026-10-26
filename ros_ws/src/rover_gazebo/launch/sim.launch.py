#!/usr/bin/env python3
"""Spawn the SCX24 rover in Gazebo Harmonic and bridge it to ROS 2.

Dimensions come from vehicle.cfg via rover_description's vehicle_cfg.py -
the same strict parser view_model.launch.py already uses - so this file
never hardcodes a vehicle dimension, mass, or steering limit.
"""

import sys
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# vehicle_cfg.py belongs to rover_description (it's the single shared
# parser view_model.launch.py already uses) rather than being copied into
# this package. Same reasoning as view_model.launch.py: rover_description
# is a plain ament_cmake package with no setup.py/__init__.py, so nothing
# puts its launch/ directory on sys.path automatically - this does it
# explicitly, resolving through any --symlink-install symlink first.
_rover_description_launch_dir = str(
    Path(get_package_share_directory("rover_description")).resolve() / "launch"
)
if _rover_description_launch_dir not in sys.path:
    sys.path.insert(0, _rover_description_launch_dir)
from vehicle_cfg import VehicleConfigError, load_vehicle_urdf_dimensions  # noqa: E402

# The fixed model name given to `ros2 run ros_gz_sim create` below and
# baked into config/bridge.yaml's GZ topic names. This is an identifier,
# not a vehicle.cfg dimension, and it must match rover_description's
# <robot name="..."> for the spawned entity and the URDF root to agree.
_GAZEBO_MODEL_NAME = "scx24_rover"

# Vertical gap between the tire's resting radius and the spawn height, so
# the rover drops a few millimeters onto the ground plane instead of
# spawning exactly tangent to it (which can register as an initial
# contact penetration). A simulation safety margin, not a vehicle
# dimension, so it stays a constant here rather than moving into
# vehicle.cfg.
_SPAWN_HEIGHT_CLEARANCE_METERS = 0.005


def _default_vehicle_config_path() -> str:
    """The vehicle.cfg installed with rover_description, unless overridden."""
    return str(
        Path(get_package_share_directory("rover_description")) / "config" / "vehicle.cfg"
    )


def _launch_setup(context, *args, **kwargs):
    """Parse vehicle.cfg, build robot_description, and start every node.

    Runs inside an OpaqueFunction so the `vehicle_config` launch argument
    is resolved first, and so a bad cfg raises before Gazebo,
    robot_state_publisher, the spawner, or the bridge have been started.
    """
    vehicle_config_path = LaunchConfiguration("vehicle_config").perform(context)

    try:
        dimensions = load_vehicle_urdf_dimensions(vehicle_config_path)
    except VehicleConfigError as config_error:
        raise RuntimeError(f"Failed to load vehicle config: {config_error}") from config_error

    xacro_file_path = str(
        Path(get_package_share_directory("rover_gazebo")) / "urdf" / "rover.gazebo.xacro"
    )
    # Same 8 keys view_model.launch.py passes: rover.gazebo.xacro declares
    # no xacro args of its own, since <xacro:include>-ing
    # rover_description's rover.urdf.xacro also includes its <xacro:arg>
    # declarations - these mappings flow straight through to them.
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
    robot_description_xml = xacro.process_file(
        xacro_file_path, mappings=xacro_mappings
    ).toxml()

    spawn_height_meters = (
        dimensions.tire_diameter_meters / 2 + _SPAWN_HEIGHT_CLEARANCE_METERS
    )

    world_path = str(Path(get_package_share_directory("rover_gazebo")) / "worlds" / "flat.sdf")
    server_only = "-s " if LaunchConfiguration("headless").perform(context).lower() == "true" else ""
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
        ),
        # headless:=true runs the server only (-s): no GUI window, e.g. for scripted routes.
        launch_arguments={"gz_args": f"-r {server_only}{world_path}"}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml, "use_sim_time": True}],
    )

    # Parameter names (topic/name/z) match ros_gz_sim's own
    # gz_spawn_model.launch.py, which passes them to this same executable
    # as ROS parameters rather than raw CLI flags.
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_scx24_rover",
        output="screen",
        parameters=[{
            "topic": "robot_description",
            "name": _GAZEBO_MODEL_NAME,
            "z": spawn_height_meters,
        }],
    )

    bridge_config_path = str(
        Path(get_package_share_directory("rover_gazebo")) / "config" / "bridge.yaml"
    )
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="bridge_node",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config_path, "use_sim_time": True}],
    )

    return [gazebo, robot_state_publisher_node, spawn_entity_node, bridge_node]


def generate_launch_description() -> LaunchDescription:
    vehicle_config_argument = DeclareLaunchArgument(
        "vehicle_config",
        default_value=_default_vehicle_config_path(),
        description="Path to a vehicle.cfg-formatted file supplying the URDF dimensions.",
    )

    headless_argument = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="true: the Gazebo server only, no GUI window.",
    )

    return LaunchDescription([vehicle_config_argument, headless_argument,
                              OpaqueFunction(function=_launch_setup)])
