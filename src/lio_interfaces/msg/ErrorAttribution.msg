# 误差归因输出 (P2: rule-based Error Attribution)
#
# 发布: lio_evaluation error_attribution_node, topic: /lio/error_attribution
# 发布时间语义: 每次 LiDAR measurement update 后随 /lio/health 同步发布 (同 header.stamp)。
#
# 语义说明:
#   - 本消息的"核心结果"是多标签 flags (is_geometry_degen / is_matching_failure /
#     is_imu_low_excitation), 它们可同时成立 (真实 LIO 故障中几何退化与低 IMU
#     激励完全可能共存)。
#   - label 仅作为显示摘要 = argmax(severity_*), tie-break 由可配置 priority 决定。
#   - margin 是 heuristic 裕度 (距规则判定边界的归一化距离), 不是概率意义上的
#     置信度; P3 若用于连续权重缩放必须先归一化 + clip。
#   - is_imu_low_excitation 只由 IMU 信号判定 (不含几何/cov);
#     imu_risk_score 才表示"低激励 × 依赖几何"的定位风险。

std_msgs/Header header

uint8 label                    # 0=NORMAL 1=GEOMETRY_DEGENERATION
                               # 2=CORRESPONDENCE_FAILURE 3=IMU_WEAK_EXCITATION
string label_str
float64 margin                 # 主标签距判定边界的归一化裕度 (heuristic, 非概率)

# ── 逐类 severity score (0~1, heuristic, 非概率) ──
float64 severity_normal
float64 severity_geometry
float64 severity_correspondence
float64 severity_imu_weak

# ── 信号级 flags (核心结果, 可同时成立) ──
bool is_geometry_degen
bool is_matching_failure
bool is_imu_low_excitation

# ── 风险分 (0~1) ──
float64 geom_risk_score
float64 imu_risk_score

# ── 触发主因快照 (便于日志/CSV 落盘自包含) ──
float64 trans_ratio
float64 rot_ratio
float64 score
uint32 effective_feature_num
float64 effective_feature_ratio
float64 mean_residual
float64 residual_p90
float64 imu_gyr_excitation
float64 imu_acc_excitation
float64 pos_cov_trace
float64[3] weak_translation_direction
uint8 weak_mode_index
uint8 dominant_weak_axis
