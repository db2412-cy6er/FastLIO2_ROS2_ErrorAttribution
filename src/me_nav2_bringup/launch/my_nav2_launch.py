import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_map_file(map_arg: str) -> str:
    """将 map 参数解析为绝对路径：
    - 绝对路径直接使用
    - 否则视为 me_nav2_bringup/map/ 目录下的文件名（.yaml 后缀可省略）
    """
    if os.path.isabs(map_arg):
        return map_arg
    if not map_arg.endswith('.yaml'):
        map_arg = map_arg + '.yaml'
    return os.path.join(get_package_share_directory('me_nav2_bringup'), 'map', map_arg)


def _setup_nav(context, *args, **kwargs):
    # 获取共享目录路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    me_share_path = get_package_share_directory('me_nav2_bringup')

    # 配置文件与地图路径
    params_file = os.path.join(me_share_path, 'config', 'nav2_params.yaml')
    map_yaml_file = _resolve_map_file(LaunchConfiguration('map').perform(context))
    rviz_file = os.path.join(me_share_path, 'rviz', 'nav2.rviz')

    # 是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1', 'yes')

    # 启动纯导航组件，不使用AMCL
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': str(use_sim_time),
            'autostart': 'True'
        }.items()
    )

    # 独立启动 Map Server：加载静态地图，供 Global Costmap 使用
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}]
    )

    # 启动 Map Server 的生命周期管理器
    lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # rviz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
    )

    return [navigation_cmd, map_server_cmd, lifecycle_manager_map_cmd, rviz_cmd]


def generate_launch_description():
    """Nav2 纯导航启动（支持多地图切换）。

    用法示例:
      ros2 launch me_nav2_bringup my_nav2_launch.py                       # 默认 nav_test_4_27
      ros2 launch me_nav2_bringup my_nav2_launch.py map:=normal_indoor
      ros2 launch me_nav2_bringup my_nav2_launch.py map:=/abs/path/to/map.yaml
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='nav_test_4_27',
            description='导航地图: me_nav2_bringup/map/ 下的文件名(可省略 .yaml)或绝对路径。'
                        '内置: nav_test_4_27, test_4_27'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Nav2 是否使用仿真时钟'),
        OpaqueFunction(function=_setup_nav),
    ])