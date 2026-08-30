# 单次 IEKF 迭代过程观测（debug 模式, 用于分析退化环境下迭代收敛行为）。
# 仅在 degeneracy.debug_iterations 开启时发布, topic: /lio/degeneracy_iterations。
# 最终 /lio/degeneracy_score 仍只发布收敛后的 final result。
#
# P5.0: 增加 matching 扩展字段 (candidate_feature_num / effective_feature_ratio /
#   residual_p90), 使 iteration==1 的行可作为 current-frame first-iteration 统计
#   (P5 gate 标定用; P2 阈值是在 final-iteration 上标定的, 不可直接复用)。

std_msgs/Header header
uint32 iteration             # IEKF 迭代序号 (本次 measurement update 内, 从 1 起)
uint32 frame_index           # LiDAR 帧序号
float64 total_residual       # 当前迭代总残差 (Σ|point-to-plane distance|)
float64 mean_residual        # 平均残差 = total_residual / effective_feature_num
uint32 effective_feature_num # 有效匹配点数量
uint32 candidate_feature_num # 候选匹配点数量 (= feats_down_size, P5.0)
float64 effective_feature_ratio  # effective / candidate (P5.0)
float64 residual_p90         # 有效点残差 90 分位 (m, P5.0)
float64 trans_ratio          # Ht 平动块 λmin/λmax
float64 rot_ratio            # Hr 转动块 λmin/λmax
float64 lambda_max           # 完整 6DoF Hessian 最大特征值
float64 lambda_min           # 完整 6DoF Hessian 最小特征值
