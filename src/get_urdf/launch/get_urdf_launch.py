import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    """Gazebo 仿真环境启动（支持多世界切换）。

    用法示例:
      ros2 launch get_urdf get_urdf_launch.py            # 默认 test_world
      ros2 launch get_urdf get_urdf_launch.py world:=normal_indoor
      ros2 launch get_urdf get_urdf_launch.py world:=/abs/path/to/xx.world
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='test_world',
            description='Gazebo 世界: worlds/ 下的文件名(可省略 .world)或绝对路径。'
                        '内置: test_world, normal_indoor'),
        OpaqueFunction(function=_launch_simulation),
    ])


def _resolve_world_file(world: str) -> str:
    """将 world 参数解析为绝对路径：
    - 绝对路径直接使用
    - 否则视为 get_urdf/worlds/ 目录下的文件名（.world 后缀可省略）
    """
    if os.path.isabs(world):
        return world
    if not world.endswith('.world'):
        world = world + '.world'
    return os.path.join(get_package_share_directory('get_urdf'), 'worlds', world)


def _launch_simulation(context, *args, **kwargs):
    world_file_path = _resolve_world_file(LaunchConfiguration('world').perform(context))

    # 动态获取 get_urdf 包在 install 目录下的绝对路径
    pkg_share_path = get_package_share_directory('get_urdf')

    rviz_config_path = os.path.join(pkg_share_path, 'rviz', 'nav2_new.rviz')

    # 拼接出准确的 URDF 路径
    urdf_file_path = os.path.join(pkg_share_path, 'model', 'simple_car.urdf')

    # 2. 读取 URDF 文件内容
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return [
        # 3. 启动 Gazebo 仿真引擎
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_file_path],
            output='screen'),

        # 4. 调用官方的 robot_state_publisher 节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # 这里的键名必须是 'robot_description'
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),

        # 5. 在 Gazebo 中生成机器人，数据源指向 robot_description 话题
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-entity', 'simple_car', '-topic', 'robot_description']),

        # 6. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]),
    ]