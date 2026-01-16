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
    
    hesai_driver_node = Node(
        name='hesai_driver_node',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': config_dir}]
    )

    with open(config_dir, 'r') as f:
        rviz2 = f.readline().strip()
    if 'rviz2: true' in rviz2:
        rviz_config=get_package_share_directory('hesai_ros_driver')+'/rviz/rviz2.rviz'
        rviz_node = Node(
            namespace='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d',rviz_config]
        )
    
        return LaunchDescription([
            rviz_node,
            hesai_driver_node
        ])
    else:
        return LaunchDescription([
            hesai_driver_node
        ])