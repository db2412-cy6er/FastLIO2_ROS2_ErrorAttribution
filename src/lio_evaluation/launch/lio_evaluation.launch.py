import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动 lio_evaluation 三个在线节点:
    - degeneracy_proxy_node (M3 外部几何代理检测器)
    - recorder_node        (TUM + degen CSV 落盘)
    - monitor_node         (CPU/内存/帧耗时)
    """
    output_dir = LaunchConfiguration('output_dir')
    traj_map = LaunchConfiguration('traj_map')
    enable_degen = LaunchConfiguration('enable_degen')
    enable_iter = LaunchConfiguration('enable_iter')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='',
                              description='数据落盘目录 (experiment_XXX)'),
        DeclareLaunchArgument(
            'traj_map',
            default_value='/gt_odom:groundtruth.txt,/Odometry:fastlio.txt',
            description='TUM 轨迹 topic:file 映射 (逗号分隔)'),
        DeclareLaunchArgument('enable_degen', default_value='true'),
        DeclareLaunchArgument('enable_iter', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ---- M3: External Geometry Proxy Detector ----
        Node(
            package='lio_evaluation',
            executable='degeneracy_proxy_node',
            name='degeneracy_proxy',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cloud_topic': '/cloud_registered',
                'odom_topic': '/Odometry',
                'score_topic': '/lio/degeneracy_score_proxy',
            }],
        ),
        # ---- M4: 在线采集 ----
        Node(
            package='lio_evaluation',
            executable='recorder_node',
            name='lio_recorder',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'output_dir': output_dir,
                'traj_map': traj_map,
                'enable_degen': enable_degen,
                'enable_iter': enable_iter,
            }],
        ),
        # ---- M4: 资源监控 ----
        Node(
            package='lio_evaluation',
            executable='monitor_node',
            name='lio_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'output_dir': output_dir,
                'proc_name': 'fastlio_mapping',
            }],
        ),
        # ---- P2: Error Attribution (rule-based) ----
        Node(
            package='lio_evaluation',
            executable='attribution_node',
            name='error_attribution',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(os.path.dirname(
                    os.path.abspath(__file__)), '..', 'config', 'attribution_params.yaml'),
            }],
        ),
    ])
