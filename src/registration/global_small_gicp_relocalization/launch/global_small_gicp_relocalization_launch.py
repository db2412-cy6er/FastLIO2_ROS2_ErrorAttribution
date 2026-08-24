import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_pcd_file(pcd: str) -> str:
    if os.path.isabs(pcd):
        return pcd
    if not pcd.endswith('.pcd'):
        pcd = pcd + '.pcd'
    return os.path.join(get_package_share_directory('me_nav2_bringup'), 'pcd', pcd)


def _default_pcd_file() -> str:
    return os.path.join(
        get_package_share_directory('me_nav2_bringup'), 'pcd', 'nav_test_4_27.pcd')


def _launch_node(context, *args, **kwargs):
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in \
        ('true', '1', 'yes')
    prior_pcd_file = _resolve_pcd_file(
        LaunchConfiguration('prior_pcd_file').perform(context))

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        emulate_tty=True,  # 开启提示颜色
        remappings=remappings,
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
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
    """small_gicp 局部重定位（支持多场景先验 PCD 切换）。"""
    return LaunchDescription([
        DeclareLaunchArgument(
            'prior_pcd_file', default_value=_default_pcd_file(),
            description='先验 PCD 地图: 绝对路径，或 me_nav2_bringup/pcd/ 下的文件名(可省略 .pcd)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟 (仿真建议 true)'),
        OpaqueFunction(function=_launch_node),
    ])
