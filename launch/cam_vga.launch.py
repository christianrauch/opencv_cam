import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    composable_node = ComposableNode(node_name='camera',
    package='opencv_cam', node_plugin='CameraNode',
    parameters=[{"width": 640, "height": 480}])
    container = ComposableNodeContainer(
            node_name='camera_container',
            node_namespace='camera',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[composable_node],
            output='screen',
    )

    return launch.LaunchDescription([container])
