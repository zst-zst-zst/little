from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("solver")
    tracker_params = os.path.join(pkg_dir, "config", "solver_gimbal_params.yaml")
    cam_params = os.path.join(pkg_dir, "config", "camera_driver_params.yaml")

    camera_node = Node(
        package="camera_driver",
        executable="camera_driver_node",
        name="camera_driver",
        output="screen",
        parameters=[cam_params],
    )

    solver_node = Node(
        package="solver",
        executable="solver_node",
        name="solver",
        output="screen",
        parameters=[tracker_params],
    )

    return LaunchDescription([camera_node, solver_node])
