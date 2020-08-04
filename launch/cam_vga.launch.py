import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    composable_node = ComposableNode(name='camera',
    package='opencv_cam', plugin='CameraNode',
    parameters=[{"width": 640, "height": 480}])
    container = ComposableNodeContainer(
            name='camera_container',
            namespace='camera',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[composable_node],
            output='screen',
    )

    return launch.LaunchDescription([container])
