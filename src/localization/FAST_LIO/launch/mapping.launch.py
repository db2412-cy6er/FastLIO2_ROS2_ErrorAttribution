import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    # P2: health 参数经 launch 传入 (绕开 rcl_yaml_param_parser 对 yaml 嵌套块
    # 的解析缺陷; 与 use_sim_time 同机制)。默认 false = 零侵入。
    health_enable = LaunchConfiguration('health_enable')
    health_imu_window_sec = LaunchConfiguration('health_imu_window_sec')
    # P5.0: degeneracy.debug_iterations 暴露 (P5.0 first-iteration 统计采集用;
    # 默认 false = 零侵入, 与 mid360.yaml 默认一致)。
    degeneracy_debug_iterations = LaunchConfiguration('degeneracy_debug_iterations')
    # P3: adaptive LiDAR weighting 参数 (唯一参数源, 不写 mid360.yaml; 默认全关 = 零侵入)。
    adaptive_enable = LaunchConfiguration('adaptive_enable')
    adaptive_max_geom_scale = LaunchConfiguration('adaptive_max_geom_scale')
    adaptive_max_match_scale = LaunchConfiguration('adaptive_max_match_scale')
    adaptive_attack_alpha = LaunchConfiguration('adaptive_attack_alpha')
    adaptive_release_alpha = LaunchConfiguration('adaptive_release_alpha')
    adaptive_high_scale_warn_th = LaunchConfiguration('adaptive_high_scale_warn_th')
    adaptive_max_downweight_duration = LaunchConfiguration('adaptive_max_downweight_duration_s')
    # P4: directional update 参数 (唯一参数源, 不写 mid360.yaml; 默认全关 = 零侵入)。
    directional_enable = LaunchConfiguration('directional_enable')
    directional_beta_t = LaunchConfiguration('directional_beta_t')
    directional_beta_r = LaunchConfiguration('directional_beta_r')
    directional_ht_accum_enable = LaunchConfiguration('directional_ht_accum_enable')
    directional_ht_accum_alpha = LaunchConfiguration('directional_ht_accum_alpha')
    directional_imu_safety_limiter_enable = LaunchConfiguration('directional_imu_safety_limiter_enable')
    directional_imu_beta_floor = LaunchConfiguration('directional_imu_beta_floor')
    # P5: current-frame robust measurement gate 参数 (唯一参数源, 默认全关 = 零侵入)。
    robust_gate_enable = LaunchConfiguration('robust_gate_enable')
    robust_gate_num_drop_th = LaunchConfiguration('robust_gate_num_drop_th')
    robust_gate_ratio_drop_th = LaunchConfiguration('robust_gate_ratio_drop_th')
    robust_gate_p90_rise_th = LaunchConfiguration('robust_gate_p90_rise_th')
    robust_gate_recover_num_th = LaunchConfiguration('robust_gate_recover_num_th')
    robust_gate_recover_ratio_th = LaunchConfiguration('robust_gate_recover_ratio_th')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_health_enable_cmd = DeclareLaunchArgument(
        'health_enable', default_value='false',
        description='P2: 发布 /lio/health (须同时 degeneracy.enable=true)'
    )
    declare_health_window_cmd = DeclareLaunchArgument(
        'health_imu_window_sec', default_value='0.5',
        description='P2: rolling IMU excitation 统计窗口 (s)'
    )
    declare_degen_debug_iter_cmd = DeclareLaunchArgument(
        'degeneracy_debug_iterations', default_value='false',
        description='P5.0: 发布 /lio/degeneracy_iterations (IEKF 每迭代观测, first-iteration 统计采集)'
    )
    declare_adaptive_enable_cmd = DeclareLaunchArgument(
        'adaptive_enable', default_value='false',
        description='P3: 自适应 LiDAR 权重 (须同时 degeneracy.enable=true)'
    )
    declare_adaptive_geom_cmd = DeclareLaunchArgument(
        'adaptive_max_geom_scale', default_value='2.0',
        description='P3: geometry channel 最大 covariance 放大倍数 (mild)'
    )
    declare_adaptive_match_cmd = DeclareLaunchArgument(
        'adaptive_max_match_scale', default_value='50.0',
        description='P3: matching channel 最大 covariance 放大倍数 (aggressive)'
    )
    declare_adaptive_attack_cmd = DeclareLaunchArgument(
        'adaptive_attack_alpha', default_value='0.5',
        description='P3: 进入降权 EMA 系数 (fast attack)'
    )
    declare_adaptive_release_cmd = DeclareLaunchArgument(
        'adaptive_release_alpha', default_value='0.1',
        description='P3: 恢复 EMA 系数 (slow release)'
    )
    declare_adaptive_warn_cmd = DeclareLaunchArgument(
        'adaptive_high_scale_warn_th', default_value='3.0',
        description='P3: 连续高 scale 告警阈值 (防正反馈循环; 默认仅 matching 通道可触发)'
    )
    declare_adaptive_downweight_cmd = DeclareLaunchArgument(
        'adaptive_max_downweight_duration_s', default_value='3.0',
        description='P3: 连续降权时长上限 (s); 超时强制回 baseline, 打破'
                    '"降权→漂移→失配→误判→继续降权"正反馈循环'
    )
    declare_directional_enable_cmd = DeclareLaunchArgument(
        'directional_enable', default_value='false',
        description='P4: 方向性状态 correction 抑制 (须同时 degeneracy.enable=true)'
    )
    declare_directional_beta_t_cmd = DeclareLaunchArgument(
        'directional_beta_t', default_value='0.5',
        description='P4: 平动弱方向 correction 保留系数 (0=完全抑制, 1=不抑制)'
    )
    declare_directional_beta_r_cmd = DeclareLaunchArgument(
        'directional_beta_r', default_value='1.0',
        description='P4: 转动弱方向保留系数 (v1 恒 1 = 不抑制转动)'
    )
    declare_directional_ht_accum_cmd = DeclareLaunchArgument(
        'directional_ht_accum_enable', default_value='false',
        description='P4: Ht 时间累积弱方向 (独立开关; 先累积 Ht 再特征分解, 防 sign ambiguity)'
    )
    declare_directional_ht_accum_alpha_cmd = DeclareLaunchArgument(
        'directional_ht_accum_alpha', default_value='0.5',
        description='P4: Ht_cum = alpha*Ht + (1-alpha)*Ht_cum 的 alpha'
    )
    declare_directional_imu_safety_cmd = DeclareLaunchArgument(
        'directional_imu_safety_limiter_enable', default_value='false',
        description='P4: IMU 低激励时限制 beta 下限 (v1 默认只记录不联动)'
    )
    declare_directional_imu_floor_cmd = DeclareLaunchArgument(
        'directional_imu_beta_floor', default_value='0.5',
        description='P4: IMU 低激励时的 beta 下限'
    )
    declare_robust_gate_enable_cmd = DeclareLaunchArgument(
        'robust_gate_enable', default_value='false',
        description='P5: 当前帧鲁棒测量拒绝 (current-frame matching gate; 须 degeneracy.enable=true)'
    )
    declare_robust_gate_num_cmd = DeclareLaunchArgument(
        'robust_gate_num_drop_th', default_value='0.40',
        description='P5: 有效特征数相对滚动基线骤降阈值 (rel_num < 此值 → reject)'
    )
    declare_robust_gate_ratio_cmd = DeclareLaunchArgument(
        'robust_gate_ratio_drop_th', default_value='0.30',
        description='P5: 有效占比相对滚动基线骤降阈值 (rel_ratio < 此值 → reject)'
    )
    declare_robust_gate_p90_cmd = DeclareLaunchArgument(
        'robust_gate_p90_rise_th', default_value='2.0',
        description='P5: residual_p90 相对滚动基线飙升倍数阈值 (> 此值 → reject)'
    )
    declare_robust_gate_rec_num_cmd = DeclareLaunchArgument(
        'robust_gate_recover_num_th', default_value='0.55',
        description='P5: 恢复判定 rel_num 阈值 (hysteresis, 比 reject 宽松)'
    )
    declare_robust_gate_rec_ratio_cmd = DeclareLaunchArgument(
        'robust_gate_recover_ratio_th', default_value='0.45',
        description='P5: 恢复判定 rel_ratio 阈值 (hysteresis)'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time,
                     'health.enable': health_enable,
                     'health.imu_window_sec': health_imu_window_sec,
                     'degeneracy.debug_iterations': degeneracy_debug_iterations,
                     'adaptive.enable': adaptive_enable,
                     'adaptive.max_geom_scale': adaptive_max_geom_scale,
                     'adaptive.max_match_scale': adaptive_max_match_scale,
                     'adaptive.attack_alpha': adaptive_attack_alpha,
                     'adaptive.release_alpha': adaptive_release_alpha,
                     'adaptive.high_scale_warn_th': adaptive_high_scale_warn_th,
                     'adaptive.max_downweight_duration_s': adaptive_max_downweight_duration,
                     'directional.enable': directional_enable,
                     'directional.beta_t': directional_beta_t,
                     'directional.beta_r': directional_beta_r,
                     'directional.ht_accum_enable': directional_ht_accum_enable,
                     'directional.ht_accum_alpha': directional_ht_accum_alpha,
                     'directional.imu_safety_limiter_enable': directional_imu_safety_limiter_enable,
                     'directional.imu_beta_floor': directional_imu_beta_floor,
                     'robust_gate.enable': robust_gate_enable,
                     'robust_gate.num_drop_th': robust_gate_num_drop_th,
                     'robust_gate.ratio_drop_th': robust_gate_ratio_drop_th,
                     'robust_gate.p90_rise_th': robust_gate_p90_rise_th,
                     'robust_gate.recover_num_th': robust_gate_recover_num_th,
                     'robust_gate.recover_ratio_th': robust_gate_recover_ratio_th}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_health_enable_cmd)
    ld.add_action(declare_health_window_cmd)
    ld.add_action(declare_degen_debug_iter_cmd)
    ld.add_action(declare_adaptive_enable_cmd)
    ld.add_action(declare_adaptive_geom_cmd)
    ld.add_action(declare_adaptive_match_cmd)
    ld.add_action(declare_adaptive_attack_cmd)
    ld.add_action(declare_adaptive_release_cmd)
    ld.add_action(declare_adaptive_warn_cmd)
    ld.add_action(declare_adaptive_downweight_cmd)
    ld.add_action(declare_directional_enable_cmd)
    ld.add_action(declare_directional_beta_t_cmd)
    ld.add_action(declare_directional_beta_r_cmd)
    ld.add_action(declare_directional_ht_accum_cmd)
    ld.add_action(declare_directional_ht_accum_alpha_cmd)
    ld.add_action(declare_directional_imu_safety_cmd)
    ld.add_action(declare_directional_imu_floor_cmd)
    ld.add_action(declare_robust_gate_enable_cmd)
    ld.add_action(declare_robust_gate_num_cmd)
    ld.add_action(declare_robust_gate_ratio_cmd)
    ld.add_action(declare_robust_gate_p90_cmd)
    ld.add_action(declare_robust_gate_rec_num_cmd)
    ld.add_action(declare_robust_gate_rec_ratio_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
