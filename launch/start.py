from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('hesai_ros_driver')+'/rviz/rviz2.rviz'
    # yaml_config=get_package_share_directory('hesai_ros_driver')+'/config/config.yaml'
    return LaunchDescription([
        Node(
            namespace='hesai_ros_driver', 
            package='hesai_ros_driver', 
            executable='hesai_ros_driver_node', 
            output='screen',
            # parameters=[{'config_path': yaml_config}]
        ),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
