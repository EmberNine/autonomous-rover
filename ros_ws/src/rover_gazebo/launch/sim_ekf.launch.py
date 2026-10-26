"""The sim plus the rover's EKF, for the real-vs-sim routes (tools/sim_route.py).

sim.launch.py (Gazebo, the bridge to ROS, the IMU), then the bridge's sim adapter
(rover_bridge.sim_adapter: /rover/joint_states and /rover/steering as the real bridge sends
them) and the EKF node, both on sim time. The EKF runs with rear_axle_locked false: the
model's AckermannSteering drives its wheels like an ideal differential, so the open formula
is the right one here, while the truck's vehicle.cfg says locked.

  ros2 launch rover_gazebo sim_ekf.launch.py [headless:=true] [vehicle_config:=...]
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("rover_gazebo"))
    default_cfg = str(Path(get_package_share_directory("rover_description")) / "config" / "vehicle.cfg")
    vehicle_config = LaunchConfiguration("vehicle_config")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "sim.launch.py")),
        launch_arguments={"vehicle_config": vehicle_config,
                          "headless": LaunchConfiguration("headless")}.items(),
    )
    adapter = Node(
        package="rover_bridge",
        executable="sim_adapter",
        name="rover_sim_adapter",
        output="screen",
        parameters=[{"vehicle_cfg": vehicle_config, "use_sim_time": True}],
    )
    ekf = Node(
        package="rover_ekf",
        executable="ekf_node",
        name="rover_ekf",
        output="screen",
        parameters=[{"vehicle_cfg": vehicle_config, "rear_axle_locked": False, "use_sim_time": True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_config", default_value=default_cfg,
                              description="vehicle.cfg for the model, the adapter and the EKF."),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true: the Gazebo server only, no GUI window."),
        sim, adapter, ekf,
    ])
