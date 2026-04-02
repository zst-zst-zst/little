from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("solver"),
        "config",
        "solver_params.yaml",
    )

    tracker_node = Node(
        package="solver",
        executable="solver_node",
        name="solver",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([tracker_node])
