from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # launch parameters
    obstacle = LaunchConfiguration("obstacle", default="0.0")
    degrees = LaunchConfiguration("degrees", default="0")
    final_approach = LaunchConfiguration("final_approach", default="false")

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="obstacle"),
            DeclareLaunchArgument(name="degrees"),
            DeclareLaunchArgument(name="final_approach"),
            Node(
                package="attach_shelf",
                executable="approach_service_node",
                output="screen",
            ),
            Node(
                package="attach_shelf",
                executable="pre_approach_v2_node",
                output="screen",
                parameters=[
                    {
                        "obstacle": obstacle,
                        "degrees": degrees,
                        "final_approach": final_approach,
                    }
                ],
            )
        ]
    )
