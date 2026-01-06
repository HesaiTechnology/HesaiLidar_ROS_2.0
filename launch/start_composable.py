from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = get_package_share_directory('hesai_ros_driver') + '/rviz/rviz2.rviz'
    
    # Create a container to hold the composable nodes
    container = ComposableNodeContainer(
        name='hesai_container',
        namespace='hesai_ros_driver',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hesai_ros_driver',
                plugin='HesaiComposableNode',
                name='hesai_ros_driver_node',
            ),
        ],
        output='screen',
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )
    
    return LaunchDescription([
        container,
        rviz_node,
    ])
