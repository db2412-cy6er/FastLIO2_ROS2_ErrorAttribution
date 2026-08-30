# LIO 健康状态 (P2: Error Attribution 的统一输入信号聚合)
#
# 发布: fast_lio 内嵌 (degeneracy.enable && health.enable 门控), topic: /lio/health
# 发布时间语义: 每次 LiDAR measurement update 收敛后发布一次,
#   header.stamp = get_ros_time(lidar_end_time), 与 /Odometry /lio/degeneracy_score 同帧。
#
# 坐标系说明 (重要):
#   - weak_direction (H6 最小特征向量) 前 3 分量 = 平动 (世界系 camera_init,
#     h_x 列0-2 是世界系法向量 n); 后 3 分量 = 转动 (body 系, h_x 列3-5 = p×n)。
#   - weak_translation_direction (Ht 3x3 最小特征向量) 为纯平动方向, 世界系;
#     与 Gazebo GT 位置误差比较前须用 Umeyama 对齐旋转 R_align 旋转到 GT 系。
#   - weak_rotation_direction (Hr 3x3 最小特征向量) 为纯转动方向, body 系。
#
# filter covariance DOF 索引 (state_ikfom, 23 维):
#   pos 0-2 / rot 3-5 / offset_R 6-8 / offset_T 9-11 / vel 12-14 / bg 15-17 / ba 18-20 / grav 21-22

std_msgs/Header header

# ── geometry (LiDAR geometric Hessian) ──
float64 score                # 0~1, EMA + 滞后后输出
bool   is_degenerate
float64 trans_ratio          # Ht λmin/λmax
float64 rot_ratio            # Hr λmin/λmax
float64 condition_number     # H6 条件数 (辅助)
float64 normal_concentration # 法向量集中度 μ1/Σμ (辅助)
float64[6] eigenvalues       # H6 特征值 (升序)
float64[6] weak_direction    # H6 最小特征向量 (trans=世界系, rot=body 系; P3 方向抑制用)
uint8  weak_mode_index       # 最小特征值的模态下标 0-5 (信息性, 不直接对应物理轴)
uint8  dominant_weak_axis    # argmax(|weak_direction|) 的状态维 0-5 (粗略主弱轴)
float64[3] weak_translation_direction  # Ht 最小特征向量, 世界系 (P2 方向验证用)
float64[3] weak_rotation_direction     # Hr 最小特征向量, body 系 (姿态误差验证用)

# ── matching quality ──
uint32 effective_feature_num # 有效匹配点数量
uint32 candidate_feature_num # 候选匹配点数量 (= feats_down_size)
float64 effective_feature_ratio  # effective / candidate (相对特征充足度)
float64 mean_residual        # point-to-plane 平均残差 (m)
float64 residual_p90         # 有效点残差 90 分位 (m, 对局部匹配恶化更敏感)
float64 total_residual       # 残差总和 (m)

# ── filter covariance (kf.get_P()) ──
float64[6] pose_cov_diag     # pos(0-2) + rot(3-5) 对角
float64 pos_cov_trace
float64 rot_cov_trace
float64[3] bg_cov_diag        # gyro bias 对角
float64[3] ba_cov_diag        # acc bias 对角
float64 bg_cov_trace
float64 ba_cov_trace

# ── IMU excitation (rolling window = health.imu_window_sec) ──
uint32 imu_sample_num         # 窗口内 IMU 样本数
float64[3] imu_gyr_rms        # 三轴角速度 RMS (rad/s)
float64[3] imu_gyr_var        # 三轴角速度方差
float64[3] imu_acc_var        # 三轴加速度方差 (m/s^2)^2
float64 imu_gyr_excitation    # RMS(|ω|) (rad/s)
float64 imu_acc_excitation    # sqrt(var_ax+var_ay+var_az) (m/s^2), 动态激励水平

# ── 便于日志/回放对齐 ──
float64 pos_x
float64 pos_y
float64 pos_z
