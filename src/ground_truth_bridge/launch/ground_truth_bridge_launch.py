import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动 ground_truth_bridge 节点。

    用法示例:
      ros2 launch ground_truth_bridge ground_truth_bridge_launch.py \
          log_file:=/home/db2412/Lidar_nav2_ws/data/trajs/gt.txt
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'gt_odom_topic', default_value='/gt_odom',
            description='Gazebo P3D 真值里程计话题'),
        DeclareLaunchArgument(
            'gt_path_topic', default_value='/gt_path',
            description='聚合后的真值轨迹话题 (nav_msgs/Path)'),
        DeclareLaunchArgument(
            'frame_id', default_value='map',
            description='路径消息的坐标系 (Gazebo 中 world 与 map 原点重合)'),
        DeclareLaunchArgument(
            'timeout_sec', default_value='5.0',
            description='真值消息超时告警阈值 (秒)'),
        DeclareLaunchArgument(
            'log_file', default_value='',
            description='TUM 格式轨迹日志路径；留空则不写日志'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='是否使用仿真时钟 (仿真必须为 true)'),

        Node(
            package='ground_truth_bridge',
            executable='ground_truth_bridge_node',
            name='ground_truth_bridge',
            output='screen',
            parameters=[{
                'gt_odom_topic': LaunchConfiguration('gt_odom_topic'),
                'gt_path_topic': LaunchConfiguration('gt_path_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'timeout_sec': LaunchConfiguration('timeout_sec'),
                'log_file': LaunchConfiguration('log_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])