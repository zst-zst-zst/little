from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):
    pkg_dir = get_package_share_directory("solver")
    default_params = os.path.join(pkg_dir, "config", "solver_params.yaml")
    gimbal_params = os.path.join(pkg_dir, "config", "solver_gimbal_params.yaml")
    cam_params = os.path.join(pkg_dir, "config", "camera_driver_params.yaml")
    serial_bridge_params = os.path.join(pkg_dir, "config", "dji_c_serial_bridge.yaml")
    gimbal_serial_compat_params = os.path.join(pkg_dir, "config", "gimbal_serial_compat.yaml")

    profile = LaunchConfiguration("profile").perform(context).strip().lower()
    if profile == "default":
        solver_params = default_params
    else:
        solver_params = gimbal_params

    camera_node = Node(
        package="camera_driver",
        executable="camera_driver_node",
        name="camera_driver",
        output="screen",
        parameters=[cam_params],
    )

    tracker_node = Node(
        package="solver",
        executable="solver_node",
        name="solver",
        output="screen",
        parameters=[solver_params],
    )

    serial_bridge_node = Node(
        package="solver",
        executable="dji_c_serial_bridge_node",
        name="dji_c_serial_bridge",
        output="screen",
        parameters=[serial_bridge_params],
        condition=IfCondition(LaunchConfiguration("enable_dji_c_bridge")),
    )

    gimbal_serial_compat_node = Node(
        package="solver",
        executable="gimbal_serial_compat_node",
        name="gimbal_serial_compat",
        output="screen",
        parameters=[gimbal_serial_compat_params],
        condition=IfCondition(LaunchConfiguration("enable_gimbal_serial_compat")),
    )

    return [camera_node, tracker_node, serial_bridge_node, gimbal_serial_compat_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="gimbal",
            description="Parameter profile: gimbal or default",
        ),
        DeclareLaunchArgument(
            "enable_dji_c_bridge",
            default_value="false",
            description="Whether to run DJI C UART bridge node",
        ),
        DeclareLaunchArgument(
            "enable_gimbal_serial_compat",
            default_value="false",
            description="Whether to run migrated LaserTracking gimbal_serial module",
        ),
        OpaqueFunction(function=launch_setup),
    ])
