import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_pcd_file(pcd: str) -> str:
    """将 prior_pcd_file 参数解析为绝对路径：
    - 绝对路径直接使用
    - 否则视为 me_nav2_bringup/pcd/ 目录下的文件名（.pcd 后缀可省略）
    """
    if os.path.isabs(pcd):
        return pcd
    if not pcd.endswith('.pcd'):
        pcd = pcd + '.pcd'
    return os.path.join(get_package_share_directory('me_nav2_bringup'), 'pcd', pcd)


def _default_pcd_file() -> str:
    return os.path.join(
        get_package_share_directory('me_nav2_bringup'), 'pcd', 'nav_test_4_27.pcd')


def _launch_node(context, *args, **kwargs):
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in \
        ('true', '1', 'yes')
    prior_pcd_file = _resolve_pcd_file(
        LaunchConfiguration('prior_pcd_file').perform(context))

    node = Node(
        package="global_relocalization_kiss_matcher",
        executable="global_kiss_matcher_relocalization_exec",
        namespace="",
        output="screen",
        emulate_tty=True,
        remappings=remappings,
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
                "voxel_resolution": 0.25,
                "use_global_initialization": True,
                "use_kiss_recovery": True,
                "gicp_max_consecutive_failures": 2,
                "recovery_min_points": 1000,
                "recovery_cooldown_sec": 2.0,
                "verify_kiss_with_gicp": True,
                "kiss_min_inliers": 3,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "lidar_frame": "livox_frame",
                "robot_base_frame": "base_footprint",
                "prior_pcd_file": prior_pcd_file,
                "input_cloud_topic": "/registered_scan",
                "use_sim_time": use_sim_time,
            }
        ],
    )
    return [node]


def generate_launch_description():
    """KISS-Matcher 全局重定位（支持多场景先验 PCD 切换）。

    用法示例:
      ros2 launch global_relocalization_kiss_matcher \
          global_kiss_matcher_relocalization_launch.py
      ros2 launch global_relocalization_kiss_matcher \
          global_kiss_matcher_relocalization_launch.py \
          prior_pcd_file:=normal_indoor use_sim_time:=true
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'prior_pcd_file', default_value=_default_pcd_file(),
            description='先验 PCD 地图: 绝对路径，或 me_nav2_bringup/pcd/ 下的文件名(可省略 .pcd)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟 (仿真建议 true)'),
        OpaqueFunction(function=_launch_node),
    ])
