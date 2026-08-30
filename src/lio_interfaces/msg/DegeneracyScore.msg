# FAST-LIO 退化检测分数（P1: Degeneracy Detection and Evaluation）
#
# 理论口径说明：
#   本消息中的 trans_ratio / rot_ratio / eigenvalues / weak_direction 均来自
#   FAST-LIO 求解过程中真实参与 IEKF 更新的 LiDAR 测量 Jacobian (h_x) 累积得到的
#   "LiDAR geometric Hessian" / "measurement Hessian"。
#   它只反映当前 scan-to-map 匹配提供的几何约束，不包含 measurement noise、
#   state covariance 等滤波器信息，因此不应被称为"滤波器完整信息矩阵"。
#
#   - Ht = Σ n nᵀ              (3x3, 平动块, 世界系法向量 n)
#   - Hr = Σ (p×n)(p×n)ᵀ       (3x3, 转动块, body 系)
#   - H6 = Σ j jᵀ              (6x6, 完整 pose geometric Hessian, j = h_x[i, 0:6])
#
#   trans_ratio / rot_ratio 是 P1 主要判定依据；完整 6DoF 的 eigenvalues /
#   weak_direction 仅保存，供 P2 (Error Attribution) / P3 (Adaptive Update) 使用。
#
# 发布时间语义：每次 LiDAR measurement update 收敛后发布一次（自然跟随 scan rate）。

std_msgs/Header header

# ── 综合状态 ──
float64 score                # 0~1, 1=严重退化; raw_score 经 EMA + 滞后后输出
bool   is_degenerate         # 平滑后的稳定退化标志 (P3 自适应更新的控制信号)

# ── geometry observability (LiDAR geometric Hessian) ──
float64 trans_ratio          # Ht 平动块 λmin/λmax    【P1 主判据】
float64 rot_ratio            # Hr 转动块 λmin/λmax    【P1 主判据】
float64 condition_number     # 完整 6DoF Hessian 条件数 (辅助记录, 不作唯一判据)
float64 normal_concentration # 法向量方向集中度 μ1/Σμ (辅助解释指标)
float64[6] eigenvalues       # 完整 6DoF Hessian 特征值 (升序, 保存供 P2/P3)
float64[6] weak_direction    # 最小特征值对应特征向量 (weak direction, P3 方向性抑制)

# ── matching quality (LIO Health Monitor matching 路扩展点) ──
uint32 effective_feature_num # 有效匹配点数量 (区分真正退化 vs 匹配失败)
float64 mean_residual        # point-to-plane 平均残差 (m)

# ── 便于日志/回放对齐 ──
float64 pos_x
float64 pos_y
float64 pos_z
