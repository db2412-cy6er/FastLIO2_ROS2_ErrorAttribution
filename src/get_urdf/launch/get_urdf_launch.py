#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""get_urdf_launch.py — Gazebo 仿真启动（环境地图与机器人出生位姿均可配置）。

设计约束（与工程约定保持一致）:
  * 环境地图可配置  : world:=xxx.world        （位于 <get_urdf>/worlds/ 下）
  * 机器人位姿可配置 : x:=... y:=... z:=... yaw:=...
  * Gazebo 资源路径可配置: gazebo_resource_root:=<资源根目录>（含 models/ 子目录，默认内置在 get_urdf 包）
  * 机器人模型固定   : model/simple_car.urdf （由 spawn_entity.py 从 /robot_description 动态生成）
  * 传感器配置固定   : URDF 内 Livox MID-360 / IMU / 底盘
  * Gazebo ROS 接口 : libgazebo_ros_init.so / libgazebo_ros_factory.so（必须保留）

内置地图（均已验证，白名单见 run_experiment.py）:
  test_world / normal_indoor / bookstore / hospital

用法示例:
  ros2 launch get_urdf get_urdf_launch.py
  ros2 launch get_urdf get_urdf_launch.py world:=normal_indoor.world
  ros2 launch get_urdf get_urdf_launch.py world:=bookstore.world
  ros2 launch get_urdf get_urdf_launch.py world:=hospital.world
  ros2 launch get_urdf get_urdf_launch.py world:=normal_indoor.world \
      x:=2.0 y:=-1.5 z:=0.30 yaw:=1.57
  # 受控退化场景夹具（实验期生成，如 run_experiment --scenario corridor）:
  ros2 launch get_urdf get_urdf_launch.py world:=/abs/path/to/corridor.world

> world 文件缺失时启动前直接报错退出（不再由 Gazebo 静默回退 empty.world，
> 避免"无地面 → 机器人无限坠落"这一类无声故障）。
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# 机器人模型名（spawn_entity 实体名，数据源为 /robot_description 话题）
ROBOT_ENTITY_NAME = 'simple_car'


def _world_file_substitution(pkg_share, world_arg):
    """构造 Gazebo 世界文件路径 substitution：
    - world 为绝对路径（实验期生成夹具）→ 原样使用（自动补 .world）
    - 否则 → <pkg_share>/worlds/<name>.world
    说明: launch 的 PathJoinSubstitution 对绝对路径组件不保证"后者覆盖前者"，
    故用 PythonExpression 显式分支。
    """
    return PythonExpression([
        "('", world_arg, "' if '", world_arg, "'.endswith('.world') "
        "else ('", world_arg, "' + '.world')) "
        "if '", world_arg, "'.startswith('/') else "
        "('", pkg_share, "' + '/worlds/' + ('", world_arg,
        "' if '", world_arg, "'.endswith('.world') else ('", world_arg, "' + '.world')))",
    ])


def _find_gazebo_media_path():
    """查找 Gazebo Classic 自带的 media 资源目录（含 RTSS 着色器库）。

    /usr/share/gazebo/setup.sh 会把该路径写入 GAZEBO_RESOURCE_PATH，
    而 source /opt/ros/humble/setup.bash 不会。缺失会导致 gzclient 渲染
    初始化失败（Failed to initialize scene / GLWidget could not create a scene），
    表现为 gazebo 进程崩溃。返回第一个存在 media/ 子目录的候选路径。
    """
    for base in ('/usr/share/gazebo-11', '/usr/share/gazebo'):
        if os.path.isdir(os.path.join(base, 'media')):
            return base
    return ''


def _check_and_log(context, *args, **kwargs):
    """启动前路径检查与关键参数日志（在 SetEnvironmentVariable 之后执行）。"""
    root = LaunchConfiguration('gazebo_resource_root').perform(context)
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    # 读取 launch context 环境（SetEnvironmentVariable 已写入新值）
    model_path = context.environment.get('GAZEBO_MODEL_PATH', '')
    resource_path = context.environment.get('GAZEBO_RESOURCE_PATH', '')

    msgs = ['\n========== get_urdf_launch ==========']
    msgs.append('World           : {}'.format(world))
    msgs.append('Resource root   : {}'.format(root))
    msgs.append('Spawn pose      : x={}, y={}, z={}, yaw={}'.format(x, y, z, yaw))

    # world 文件前置校验：缺失时 Gazebo 会静默回退 empty.world（无地面 →
    # 机器人无限坠落），必须在启动前 fail-fast（raise 使 launch 终止）。
    world_file = world if world.startswith('/') else os.path.join(root, 'worlds', world)
    if not world_file.endswith('.world'):
        world_file += '.world'
    if not os.path.isfile(world_file):
        raise RuntimeError(
            '[FATAL] world 文件不存在: {}\n'
            '        Gazebo 将回退 empty.world（无地面 → 机器人无限坠落）。\n'
            '        请先重新构建 get_urdf: '
            'colcon build --packages-select get_urdf --symlink-install\n'
            '        （包内地图文件位于 src/get_urdf/worlds/，需与 install 目录一致）'
            .format(world_file))
    msgs.append('World file      : {} [OK]'.format(world_file))

    if not os.path.isdir(root):
        msgs.append('[WARNING] gazebo_resource_root 目录不存在: {}'.format(root))
    else:
        models_dir = os.path.join(root, 'models')
        if not os.path.isdir(models_dir):
            msgs.append('[WARNING] 找不到子目录: {}'.format(models_dir))
        else:
            msgs.append('[OK] 资源根目录与 models/ 子目录均已找到')

    media_path = _find_gazebo_media_path()
    if media_path:
        msgs.append('Gazebo media path   : {}'.format(media_path))
    else:
        msgs.append('[WARNING] 未找到 Gazebo media 目录(/usr/share/gazebo-11)，'
                    'gzclient 渲染可能失败')

    msgs.append('GAZEBO_MODEL_PATH    : {}'.format(model_path))
    msgs.append('GAZEBO_RESOURCE_PATH : {}'.format(resource_path))
    msgs.append('=====================================')
    return [LogInfo(msg=m) for m in msgs]


def generate_launch_description():
    # ---- 包 share 目录（自动解析，不硬编码绝对路径） ----
    pkg_share = FindPackageShare('get_urdf')

    # ---- launch 参数 ----
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='test_world.world',
        description='Gazebo 世界文件：包内地图名（位于 <get_urdf>/worlds/，.world 可省略），'
                    '或绝对路径（受控退化场景夹具）。'
                    '内置：test_world / normal_indoor / bookstore / hospital',
    )
    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0', description='机器人初始 x 坐标（m）')
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0', description='机器人初始 y 坐标（m）')
    z_arg = DeclareLaunchArgument(
        'z', default_value='0.30',
        description='机器人初始 z 坐标（m），默认略高于地面，由物理引擎自然落到地面')
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='机器人初始朝向 yaw（rad）：1.57≈90°，3.14≈180°，-1.57≈-90°')
    gazebo_resource_root_arg = DeclareLaunchArgument(
        'gazebo_resource_root',
        default_value=pkg_share,
        description='Gazebo 资源根目录（含 models/ 子目录）。'
                    '默认指向 get_urdf 包的 share 目录（内置 Small House/Bookstore/Hospital 模型），'
                    '用于设置 GAZEBO_MODEL_PATH=<root>/models 与 '
                    'GAZEBO_RESOURCE_PATH=<root>，可通过命令行覆盖为外部路径。')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true: 只起 gzserver（无 gzclient GUI），降低渲染负载，'
                    '适合自动化实验/服务器')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='false: 不起 rviz2（无 GUI 环境/自动化实验时建议关闭）')

    # ---- 资源路径（全部基于包 share 目录自动拼接） ----
    world_file = _world_file_substitution(pkg_share, LaunchConfiguration('world'))
    urdf_file = PathJoinSubstitution([pkg_share, 'model', 'simple_car.urdf'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'nav2_new.rviz'])

    # ---- 机器人描述（运行时读取 URDF） ----
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]), value_type=str)

    # ---- Gazebo 环境变量：新路径 prepend，保留原有路径 ----
    # GAZEBO_MODEL_PATH    = <root>/models:<原有>
    # GAZEBO_RESOURCE_PATH = <root>:<gazebo media>:<原有>
    #   其中 gazebo media(如 /usr/share/gazebo-11) 提供 RTSS 着色器库，
    #   缺失会导致 gzclient "Failed to initialize scene" 崩溃。
    gazebo_models_path = PathJoinSubstitution([
        LaunchConfiguration('gazebo_resource_root'), 'models'])
    gazebo_media_path = _find_gazebo_media_path()
    gazebo_resource_parts = [LaunchConfiguration('gazebo_resource_root')]
    if gazebo_media_path:
        gazebo_resource_parts += [':', gazebo_media_path]
    gazebo_resource_parts += [
        ':', EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='')]

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gazebo_resource_root_arg,
        headless_arg,
        rviz_arg,

        # 0. 设置 Gazebo 模型/资源搜索路径（必须在 gazebo 启动之前生效）
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                gazebo_models_path,
                ':',
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=gazebo_resource_parts,
        ),

        # 0.1 启动前路径检查 + 关键参数日志（此时环境变量已写入 context.environment）
        OpaqueFunction(function=_check_and_log),

        # 1. Gazebo 仿真引擎（gzserver）与客户端（gzclient）
        #    保留 libgazebo_ros_init.so / libgazebo_ros_factory.so：
        #    spawn_entity.py 依赖 factory 服务向 Gazebo 插入机器人。
        #    headless:=true 时只起 gzserver（无 gzclient），降低渲染负载。
        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file,
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('headless')),
        ),
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file,
            ],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('headless')),
        ),

        # 2. robot_state_publisher：发布 /robot_description 与 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description, 'use_sim_time': True},
            ],
        ),

        # 3. spawn_entity.py：从 /robot_description 动态生成机器人到 Gazebo。
        #    初始位姿由 -x -y -z -Y 控制（注意 yaw 使用 -Y，不是 -yaw）。
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-entity', ROBOT_ENTITY_NAME,
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ),

        # 4. RViz 可视化（rviz:=false 时关闭，自动化实验建议关闭以减少软渲染负载）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])