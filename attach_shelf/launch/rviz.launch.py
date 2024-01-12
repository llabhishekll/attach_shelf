from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # package launch description
    path_root = Path(get_package_share_directory("attach_shelf"))
    path_rviz = path_root / "rviz" / "default.config.rviz"

    # return launch
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=["-d", path_rviz.as_posix()],
            ),
        ]
    )
