from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('hesai_ros_driver'), 
        'config',
        'ros_config.yaml'
        )
    
    return LaunchDescription([
        Node(
            name='hesai_driver_node',
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            output='screen',
            parameters=[config_dir]),
    ])
