# lio_evaluation

退化感知 LIO 改进项目（P1: Degeneracy Detection and Evaluation）的**评测与退化检测模块**。

## 双检测器架构

| 检测器 | 位置 | 数据来源 | 话题 | 说明 |
|---|---|---|---|---|
| **内嵌 Exact（主）** | `fast_lio/src/laserMapping.cpp` | IEKF 真实测量 Jacobian `h_x` | `/lio/degeneracy_score` | 精确 LiDAR geometric Hessian (Ht/Hr/H6)，零近似 |
| **外部几何代理 Proxy（旁路）** | `src/degeneracy_proxy.cpp` | `/cloud_registered` + `/Odometry` | `/lio/degeneracy_score_proxy` | 自维护局部地图 + kNN/PCA 重建约束，验证几何趋势一致性 |

> 口径：`JᵀJ` 只反映 scan-to-map 匹配提供的几何约束（**LiDAR geometric Hessian / measurement Hessian**），
> 不是滤波器完整信息状态（不含 measurement noise / state covariance）。

## 节点 / 脚本

| 入口 | 作用 |
|---|---|
| `degeneracy_proxy_node` | M3 外部几何代理检测器（C++） |
| `recorder_node` | 在线采集：TUM 轨迹 + degen CSVs（fast_lio 不写盘，全部落盘统一在此） |
| `monitor_node` | CPU/内存（psutil）+ 帧间隔采样 |
| `compute_metrics` | 离线：时间对齐 → SE(3) Umeyama → ATE/RPE（evo）→ runtime 合并 → result.yaml |
| `directional_analysis` | M6：weak direction vs GT 各方向误差对照 |
| `run_experiment` | 编排：prepare / run-online / replay / eval / all（含世界白名单校验 + 运行时健康门禁 + 孤儿 gzserver 清理 + 脚本化运动驱动 + 健康看门狗） |
| `generate_worlds` | 生成受控退化场景夹具（实验期生成到实验目录，不驻留包内 worlds/，生成后自动自检） |
| `drive_node` | 脚本化运动驱动 + 反应式避障（漫游：前进腿→每次只转 30° 小角，覆盖更多新区域；前向锥检测 `/livox/lidar`，<stop_dist 时**步进式**避障：每次只转 30° 重新评估，>clear_dist/stop_dist 恢复前进），默认开启（`--no-drive` 关闭） |
| `health_watch` | 运行期健康看门狗：翻车(FLIP)/卡死(STUCK)/RTF<0.9(RTF_LOW) 事件记录到 `<exp>/health.log`；`--health-abort` 时翻车即写 ABORT 哨兵提前中止 |

## 实验流程

```bash
# 1. 健康基线实验（包内确认地图，白名单自动校验）
#    默认按"仿真时间"采集 args.duration 秒（RTF<1 时墙钟自动延长，保证有效轨迹达标）
python3 scripts/run_experiment.py all --world normal_indoor --algo degen --duration 90
#    可用确认地图（P0/P0.1 已验证）:
#    test_world / normal_indoor / bookstore / hospital
#    出生位姿可用 --x --y --z --yaw 覆盖（如 bookstore 推荐 --x -2.3 --y 6.2 --yaw -1.72）
#    可选: --headless(只起 gzserver) --no-rviz(关 rviz2) --health-abort(翻车即中止)
#          --no-drive(禁用运动驱动)

# 1b. 手动遥控录数据（--teleop: 启动 gui_teleop_node, 在 Teleop 窗口内 WASD 驾驶,
#      w/s 前后, a/d 左右转, Shift 加速, 空格急停, 关窗口即提前停止采集）
python3 scripts/run_experiment.py all --world normal_indoor --algo degen \
    --duration 90 --teleop --no-rviz

# 2. 受控退化场景 (M5) — 实验期生成夹具到实验目录，生成后自检 + 运行时健康门禁
python3 scripts/run_experiment.py all --scenario corridor     --algo degen --duration 90
python3 scripts/run_experiment.py all --scenario single_plane --algo degen --duration 90
python3 scripts/run_experiment.py all --scenario open_ground  --algo degen --duration 60
#    需要时单独生成夹具:
python3 scripts/generate_worlds.py --out-dir data/experiments/experiment_001 --scenario corridor

# 3. 同一 bag 对比 baseline vs degen (可复现):
#    先录 bag: scripts/record_bag.sh   (录制 /livox/lidar /livox/imu /gt_odom 等)
python3 scripts/run_experiment.py all --algo baseline --bag data/bags/<bag> --duration <len>
python3 scripts/run_experiment.py all --algo degen    --bag data/bags/<bag> --duration <len>

# 4. 单独重算指标
python3 scripts/run_experiment.py eval data/experiments/experiment_001
```

> **采集窗口**：`run-online` 以**仿真时间**（`/clock`）推进 `--duration` 秒为完成判据，
> 墙钟上限为 `duration×4` 兜底。因此即使虚拟机 RTF<1，落盘轨迹的真实跨度也达到请求时长。

> **关于地图**：包内 `get_urdf/worlds/` 只保留 5 个确认地图
> （test_world / normal_indoor / bookstore / hospital）。
> 受控退化场景是"实验夹具"，由 `generate_worlds.py` 在实验期生成到实验目录
> （不驻留包内），并在 `run-online` 启动后通过健康门禁验证
> （世界真正加载 / 机器人落在地面 / 雷达有点云），避免"world 缺失 → Gazebo 静默
> 回退 empty.world → 机器人无限坠落"这类无声故障。
>
> **出生位姿**：`--x/--y/--z/--yaw` 可覆盖；已内置推荐位姿
> （normal_indoor: x=5.0 y=-1.5 yaw=1.57；bookstore: x=-2.3 y=6.2 yaw=-1.72）。
> 若小车在出生点被家具卡住（GT 位移很小），请调整出生位姿或缩短驱动距离。
>
> **运动驱动**：在线实验默认开启 `drive_node`（前进→掉头→返回），否则静态机器人
> 的 ATE/RPE 无意义；`--no-drive` 可关闭。

## 实验目录产物

```
experiment_001/
├── config.yaml            # 元信息 + fast_lio 配置快照
├── groundtruth.txt        # TUM (/gt_odom)
├── fastlio.txt            # TUM (/Odometry)
├── liosam.txt             # P1 占位
├── runtime.csv            # 帧耗时 + CPU% + RSS/M
├── degen_exact.csv        # 内嵌检测器分数序列
├── degen_proxy.csv        # 旁路检测器分数序列
├── degen_iterations.csv   # IEKF 迭代观测 (debug 模式)
├── directional_error.png  # weak direction vs GT 误差曲线
├── ate_result.zip         # evo_ape 结果
├── rpe_result.zip         # evo_rpe 结果
└── result.yaml            # 汇总指标 (ATE/RPE/runtime/退化统计/一致性/directional)
```

## 退化判定参数（`fast_lio/config/mid360.yaml` → `degeneracy:`）

- `threshold_t / threshold_r`：Ht/Hr 的 λmin/λmax 退化阈值（默认 `threshold_t=0.10`、`threshold_r=0.02`；corridor 实测标定，见 `report/P1.3退化检测评测·收尾报告.md`）
- `ema_alpha / score_enter / score_exit / min_enter_frames / min_exit_frames`：EMA + 滞后状态机
- `debug_iterations`：开启后额外发布 `/lio/degeneracy_iterations`

`degeneracy.enable` 默认 **false**：不计算不发布，行为与原始 FAST-LIO 数值等价（零侵入）。

## P2/P3 预留

- `DegeneracyScore` 已含 matching quality 字段（effective_feature_num / mean_residual），是 LIO Health Monitor 的 matching 路。
- `weak_direction`（完整 6DoF Hessian 最小特征值特征向量）已保存，P3 方向性退化抑制直接用。
- 消息接口统一在 `src/lio_interfaces/`，后续 `LioHealth` / `ErrorAttribution` 消息在此扩展。
