from launch import LaunchDescription
from launch_ros.actions import Node
# Launch Args
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# Load Params Yaml
import os
from ament_index_python.packages import get_package_share_directory
# Shell Commands
from launch.actions import ExecuteProcess

def generate_launch_description():

  plugin_loader_node = Node(
    package='ros2-message-filters',
    executable='rclcpp_node_mf',
    name='rclcpp_node_mf',
    # parameters=[os.path.join(get_package_share_directory('base_plugin'), 'config', 'params.yaml')],
  )

  start_map_baselink_tf_cmd = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_baselink_tf',
    output='screen',
    arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]
  )

  play_rosbag = ExecuteProcess(
    cmd=["ros2", "bag", "play", "-l", os.path.join(get_package_share_directory("ros2-message-filters"), "bags", "velodyne_points")], shell=True,
    output="screen"
  )

  # Define launch description
  ld = LaunchDescription()
  ld.add_action(plugin_loader_node)
  ld.add_action(start_map_baselink_tf_cmd)
  ld.add_action(play_rosbag)

  # Return launch description
  return ld
