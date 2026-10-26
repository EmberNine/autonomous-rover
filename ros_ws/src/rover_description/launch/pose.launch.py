"""The rover's live pose in RViz (group 6), on the laptop: the EKF on the Pi publishes /odom and
TF odom -> base_link; this adds the model on top.

Starts robot_state_publisher (the URDF from vehicle.cfg, as view_model.launch.py builds it),
joint_state_publisher (the wheel and steering joints at 0: only the pose is live) and rviz2
with rviz/pose.rviz (fixed frame odom, top-down: the model, TF, and /odom with its covariance).

    ros2 daemon start; source ~/rover_net.sh
    ros2 launch rover_description pose.launch.py
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_VIEW_MODEL_SPEC = importlib.util.spec_from_file_location(
    "view_model_launch", Path(__file__).resolve().parent / "view_model.launch.py")
view_model = importlib.util.module_from_spec(_VIEW_MODEL_SPEC)
_VIEW_MODEL_SPEC.loader.exec_module(view_model)


def _launch_setup(context, *args, **kwargs):
    vehicle_config_path = LaunchConfiguration("vehicle_config").perform(context)
    rviz_config_path = str(Path(get_package_share_directory("rover_description")) / "rviz" / "pose.rviz")
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": view_model.robot_description_xml(vehicle_config_path)}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             name="joint_state_publisher", output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_config_path]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_config", default_value=view_model._default_vehicle_config_path(),
                              description="Path to a vehicle.cfg-formatted file supplying the URDF dimensions."),
        OpaqueFunction(function=_launch_setup),
    ])
