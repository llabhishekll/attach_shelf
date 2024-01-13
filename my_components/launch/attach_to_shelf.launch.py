from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # return launch
    return LaunchDescription(
        [
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                name="my_container",
                namespace="",
                composable_node_descriptions=[
                    ComposableNode(
                        package="my_components",
                        plugin="my_components::PreApproach",
                        name="pre_approach",
                    ),
                    ComposableNode(
                        package="my_components",
                        plugin="my_components::AttachServer",
                        name="attach_server",
                    ),
                ],
                output="screen",
            )
        ]
    )
