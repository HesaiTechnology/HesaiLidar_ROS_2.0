"""Run the Hesai driver as a composable node inside a component container.

Arguments:
  config_path         Driver YAML config. Empty -> config/config.yaml of the package.
  container_name      Name of the container to create, or of an existing one when
                      use_existing_container:=true.
  use_existing_container  Load into an already running container instead of starting one.
  use_intra_process   Enable intra-process (zero-copy) comms with other components.
  rviz                Start rviz2.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rviz_config = get_package_share_directory('hesai_ros_driver') + '/rviz/rviz2.rviz'

    container_name = LaunchConfiguration('container_name')
    use_existing = LaunchConfiguration('use_existing_container')

    driver = ComposableNode(
        package='hesai_ros_driver',
        plugin='hesai_ros_driver::HesaiRosDriverNode',
        name='hesai_ros_driver_node',
        namespace='hesai_ros_driver',
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
        extra_arguments=[{'use_intra_process_comms': LaunchConfiguration('use_intra_process')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=''),
        DeclareLaunchArgument('container_name', default_value='hesai_container'),
        DeclareLaunchArgument('use_existing_container', default_value='false'),
        DeclareLaunchArgument('use_intra_process', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[driver],
            output='screen',
            condition=UnlessCondition(use_existing),
        ),
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[driver],
            condition=IfCondition(use_existing),
        ),
        Node(
            namespace='rviz2', package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
