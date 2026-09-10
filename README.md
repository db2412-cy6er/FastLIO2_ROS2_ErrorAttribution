# FastLIO2_ROS2_ErrorAttribution

> **基于 FAST-LIO2 的退化感知 LiDAR-惯性 SLAM 系统**
> 可观测性检测 → 误差归因 → 自适应更新 全链路研究，并落地为带 3D 全局重定位的 Nav2 自主导航平台

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22313F?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](https://releases.ubuntu.com/22.04/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-orange)](https://classic.gazebosim.org/)
[![LiDAR](https://img.shields.io/badge/LiDAR-Livox%20MID--360-00A6D6)](https://www.livoxtech.com/mid-360)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](./LICENSE)

---

## Fork 说明 / Attribution

本仓库 **fork 自 [`Ikunio/Lidar_nav2_ws`](https://github.com/Ikunio/Lidar_nav2_ws)**（MIT License，© 2025 Ikunio），
上游提供了一套 ROS 2 + Gazebo 的 3D LiDAR 自主导航底座：双 LIO 里程计（FAST-LIO2 / Point-LIO）、
KISS-Matcher + small_gicp 无初值全局重定位、Nav2 导航栈、仿真/实机一键切换的脚本体系。

**本仓库在上游底座之上新增的工作（P0–P5）**：

| 阶段 | 内容 |
|---|---|
| **P0 / P0.1** | Gazebo 真值评测基建（P3D 插件 → `/gt_odom` → `ground_truth_bridge` → `/gt_path`）；4 张高保真室内地图 + **182 个 RoboMaker 模型包内自包含** |
| **P1** | FAST-LIO2 滤波器内核的 **几何 Hessian 退化检测**（内嵌 exact 检测器 + 外部几何 proxy 检测器 双架构）与首轮评测 |
| **P2** | **三层误差归因框架**（Health → Attribution → Consequence）+ 4 类故障注入（dropout / fov_crop / outlier / noise）验证 |
| **P3 / P4 / P5** | 自适应降权 / 方向性抑制 / 帧级拒绝 **三条直觉路线的 median-of-3 严格 A/B 与量化证伪** |

> 上游原有的构建、运行与实机部署文档**完整保留**在 [`README_upstream.md`](./README_upstream.md) 与
> [`环境搭建.md`](./环境搭建.md) / [`Environment_Setup_EN.md`](./Environment_Setup_EN.md)。
> 上游的公开接口与包名**未做重命名**，本仓库仅替换了 README 并追加 P 系列模块。

---

## 目录

- [1. 项目简介](#1-项目简介)
- [2. 我的贡献（P0–P5）](#2-我的贡献p0p5)
- [3. 快速开始](#3-快速开始)
- [4. P 系列工作总览](#4-p-系列工作总览)
- [5. 最终成果与量化指标](#5-最终成果与量化指标)
- [6. 指标参数字典](#6-指标参数字典)
- [7. 模块成果清单](#7-模块成果清单)
- [8. 结论](#8-结论)
- [9. 可复现性](#9-可复现性)
- [10. 目录结构](#10-目录结构)
- [11. 许可证与致谢](#11-许可证与致谢)

---

## 1. 项目简介

本项目讲成 **"两条主线、一个闭环"**：

- **主线一 · 3D LiDAR 自主导航系统（工程底座）**
  LiDAR/IMU → LIO（FAST-LIO2 或 Point-LIO，可切换）→ TF 桥接 `lio_interface` → `/registered_scan`
  （`sensor_scan_generation`）→ 3D→2D 切片 → 全局重定位（KISS-Matcher + small_gicp）→ Nav2（NavFn + DWB）。
  TF 树：**`map → odom → base_footprint → chassis → livox_frame`**。

- **主线二 · 退化感知 LIO 研究平台（算法纵深）**
  在主线一的 LIO 里程计**内核**里，围绕"几何退化"这一 SLAM 经典难题，打通
  **检测（P1）→ 归因（P2）→ 自适应控制（P3/P4/P5）→ 真值评测**的完整研究闭环，
  并用受控实验系统证伪三条主流"自适应"路线，收敛出唯一被验证有效的方向。

- **一个闭环 · Gazebo + 真值的可复现实验底座**
  受控退化场景生成（走廊 / 单面墙 / 开阔地）+ 4 类故障注入 + 反应式避障漫游 + 翻车/卡死看门狗，
  每个实验目录自包含 `config.yaml`（含 git commit 与 resolved 参数快照）+ 轨迹 + 多路 CSV + `result.yaml`，
  使"每个结论都能一键回退复现"。
---

## 2. 我的贡献（P0–P5）

### 2.1 一句话（30 秒电梯陈述）

> 在 ROS 2 + Gazebo + Livox MID-360 的仿真/实机双模环境上，先搭出一套完整 **3D LiDAR 自主导航系统**
> （FAST-LIO2/Point-LIO 双后端里程计、SLAM Toolbox 建图、PCD 先验地图 + KISS-Matcher/small_gicp 无初值全局重定位、Nav2）；
> 随后深入 **FAST-LIO2 滤波器的 ESKF/IEKF 内核**，自研 **"退化检测 → 误差归因 → 自适应更新"全链路**与配套评测闭环：
> 从真实参与迭代更新的测量 Jacobian 构造几何 Hessian 量化可观测性，用三层框架区分
> "几何退化 / 匹配失败 / IMU 激励不足"三类风险源，再通过 fault injection、hold-out、median-of-3 严格 A/B，
> 逐条**证伪**"整体降权 / 方向性抑制 / 帧级拒绝"三种直觉方案，最终把有效方向收敛到
> **"检测 + 归因 + 门控"**，并沉淀出一套可复现的 LIO 算法评测基础设施。

### 2.2 三大突出点

**① 在滤波器"内部"构建可观测性量化器（P1）**
不靠点云外观"猜"退化，而是在 FAST-LIO2 每次 LiDAR update 收敛后，取**真实参与状态更新的逐点
point-to-plane 测量 Jacobian 行** `j = h_x[i, 0:6]`，累加成 6×6 **LiDAR geometric Hessian**：
`Ht = Σ n·nᵀ`（平动块，世界系法向量）、`Hr = Σ (p×n)(p×n)ᵀ`（转动块）、`H6 = Σ j·jᵀ`。
随后特征分解得到 `trans_ratio = λmin(Ht)/λmax(Ht)`、`rot_ratio = λmin(Hr)/λmax(Hr)` 与
**弱方向向量 `weak_direction`**（H6 最小特征值对应特征向量）。
坐标约定（`h_x` 列 0-2 世界系 / 列 3-5 body 系）在 `.msg` 注释里显式固化，避免 P2–P5 用错坐标系。

**② 从"检测"到"归因"——三层误差归因框架（P2）**
把 LIO 故障按**风险源**拆成可同时成立的多标签 flags：
`is_geometry_degen / is_matching_failure / is_imu_low_excitation`，各自输出 0~1 heuristic severity。
FAST-LIO 内嵌发布 `/lio/health`（聚合 geometry / matching / covariance / IMU 激励 四路原始信号），
外挂 rule-based `attribution_node` 输出 `/lio/error_attribution`。

**③ 用严格实验方法学系统证伪三条直觉"自适应"路线（P3/P4/P5）**
`P3 整体降权`（运行时 R 缩放 + fast-attack/slow-release EMA + 3s 上限）→
`P4 方向性抑制`（ESKF state-correction 层沿弱方向投影 `Π_β(v)=v−(1−β)(v·w_t)w_t`，同步投影 K 行保持一致）
→ `P5 当前帧测量拒绝`（IEKF 首次迭代后按相对基线骤降 + 残差异常拒绝整帧，恢复 x/P 为 propagation 态并跳过地图插入）。
三条路线全部经 median-of-3 A/B 量化证伪，并定位到"**拒绝→漂移→拒绝**"正反馈。

### 2.3 工程纪律（为什么这些结论可信）

| 纪律 | 做法 | 证据 |
|---|---|---|
| 零侵入 | 所有新机制默认 `false`，仅经 launch 单一入口开启 | 同 bag 回放 baseline/degen 轨迹 Welch t 检验 **p=0.177** |
| 数值正确性 | 每个机制配 synthetic test + **C++/Python parity check** | P3 `max\|err\|=5e-6`；P4 **535/535 帧 100% 一致** |
| 数据划分 | 只用 `corridor_run1` 标定，新走廊/室内/单面墙/held-out fault 做验证 | 见 `report/P2*` |
| 统计口径 | 正式 A/B 只信 **median-of-3**，只承认超过运行噪声的差异 | FAST-LIO 同 bag 复跑 ATE 0.104~0.204 m 浮动实测 |
| 可回退 | `p2-final / p3-final / p4-final / p5-final` 四个 git tag 冻结 | `git tag` |
| 参数单一事实源 | 参数只从 launch 下发，实验目录锁定 resolved 快照 | `config.yaml` 中的 `resolved_args` |

---

## 3. 快速开始

```bash
# 0) 依赖（完整步骤见 环境搭建.md / README_upstream.md）
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

# 1) 构建
cd scripts && ./build.sh            # = colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source ../install/setup.bash

# 2) 仿真导航（含 KISS-Matcher 全局重定位 + Nav2）
./nav2_sim.sh                                  # 默认 test_world + nav_test_4_27
./nav2_sim.sh normal_indoor normal_indoor normal_indoor   # 指定 world / 2D 地图 / 先验 PCD

# 3) 实验编排（P 系列评测入口）
python3 src/lio_evaluation/scripts/run_experiment.py all --world normal_indoor --duration 90
python3 src/lio_evaluation/scripts/run_experiment.py replay <exp_dir> --bag data/bags/corridor_run2 --algo degen
python3 src/lio_evaluation/scripts/run_experiment.py eval <exp_dir>       # → result.yaml 成绩单
```

> 常用脚本一览表见 [§7.5](#75-脚本scripts)；上游完整用法（实机模式、地图保存、重定位调参）见
> [`README_upstream.md`](./README_upstream.md)。
---

## 4. P 系列工作总览

| 阶段 | 目标 | 关键方法 | 关键结论 | 证据文件 |
|---|---|---|---|---|
| **P0** | 真值评测基建 + 场景参数化 | Gazebo P3D 真值插件 → `/gt_odom`；`ground_truth_bridge` → `/gt_path`/TUM；`world` 与出生位姿 launch 参数化 | 没有 GT 基建，后面所有"厘米级结论"都不成立 | `report/P0增加室内地图报告.md` |
| **P0.1** | 多室内地图 + 模型自包含 | 3 套 RoboMaker 室内地图（normal_indoor / bookstore / hospital）；**182 个模型移入包内**；`gazebo_resource_root` 改用 `FindPackageShare()`；修 Linux mesh 大小写 bug | 从"依赖外部绝对路径、启动即崩"→"包内自包含、四个地图一键切换" | `report/P0.1增加室内地图报告.md` |
| **P1** | 退化检测（双检测器） | 内嵌 **exact**（几何 Hessian，`H6/Ht/Hr`，看滤波器内部真实约束）+ 外部 **proxy**（`degeneracy_proxy.cpp`，kNN/PCA 从 `/cloud_registered` 独立估计）；受控场景 `corridor/single_plane/open_ground` | 双检测器**灵敏度互补**：open_ground 100% overlap；corridor 只有 proxy 先检出；single_plane 只有 exact 检出 → 单检测器必漏检一类 | `report/P1模块构建.md`、`P1.1`、`P1.2`、`P1.3` |
| **P1.3** | 阈值标定 + 弱方向分析 | 同 bag baseline/degen 各 3 次回放做**数值等价验证**；扫描 `threshold_t`；z 轴弱方向成因分析 | 数值等价（Welch t 检验 **p=0.177**）；走廊检出率 **0% → 99.6%**（`threshold_t` 0.02→0.10~0.15）；z 弱是 MID-360 视场（−7°~+52°、89% 光束朝上、0.8 m 走廊地面点仅 6%）导致，非 bug | `report/P1.3退化检测评测·收尾报告.md` |
| **P2** | 三层误差归因 | `LioHealth` 聚合四路信号 → rule-based `attribution_node` → `ErrorAttribution`；4 类故障原语注入；标定/验证集分离；precision/recall 冻结阈值 | 干净走廊 recall **1.00**、正常室内**零误报**、单面墙 recall **1.00**；fault 注入 **precision 1.00 / recall 0.65~0.77**；**核心反直觉结论：检出退化 ≠ 误差变大**（门槛型退化 ATE 差异可达两个数量级） | `report/P2误差归因模块.md` |
| **P3** | 自适应降权（证伪） | `LASER_POINT_COV` 编译期常量 → 运行时 `lidar_meas_cov_`；双通道 severity → R 放大倍数；fast-attack/slow-release EMA；3 s 降权时长上限 | normal 零影响、corridor 无显著差异；**single_plane 变差 4.5×**；holdout/stress fault **变差 14× / 8.7×**；标定期发现"降权→IMU 漂移→失配→误判→继续降权"**正反馈循环** | `report/P3自适应权重.md` |
| **P4** | 方向性抑制（证伪 + gate 有效） | ESKF state-correction 层投影 `Π_β`；同步投影 K 行保持协方差一致；`Ht` 时间累积解决特征向量符号歧义；仅 `geometry-bad ∧ matching-ok` 触发 | normal/corridor 安全；**single_plane holdout 变差 3.7×**；fault 场景 **15~50% 改善倾向**（方差大、机制未明）；**matching gate 有效性被确认** | `report/P4方向性抑制.md` |
| **P5** | 当前帧鲁棒测量拒绝（证伪） | IEKF 首次迭代后按"相对滚动基线骤降 + 残差异常"拒绝整帧；恢复 `x/P` 为 propagation 态并跳过地图插入；恢复状态机 + 地图门控 | clean / geometry 场景**零误杀、零侵入**；但持续 fault **变差 5.8~26×**；定位到"**拒绝→漂移→拒绝**"负反馈 | `report/P5当前帧鲁棒测量拒绝.md`、`P5.0` |

---

## 5. 最终成果与量化指标

### 5.1 检测能力（P1）

| 指标 | 结果 | 说明 |
|---|---|---|
| 干净走廊召回率 | **1.00** | 8 张受控场景 |
| 单面墙召回率 | **1.00** | single_plane |
| 4 类故障注入 precision | **全部 1.00** | dropout / fov_crop / outlier / noise |
| 走廊退化帧检出率 | **0% → 99.6%** | 阈值 `threshold_t` 0.02 → 0.10~0.15 |
| 健康室内误报 | **degraded_ratio = 0.0%** | normal_indoor / bookstore / hospital |
| 双检测器互补 | open_ground **100% overlap**；corridor 仅 proxy 先检出；single_plane 仅 exact 检出 | 单检测器必漏检一类 |
| 零侵入验证 | 同 bag 各 3 次回放，轨迹**数值等价** | Welch t 检验 **p=0.177** |

### 5.2 归因能力（P2）

| 指标 | 结果 |
|---|---|
| 干净走廊 recall | **1.00** |
| 正常室内 flag 误报 | **0**（无 flag 触发） |
| 单面墙 recall | **1.00** |
| fault 注入 precision / recall | **1.00 / 0.65~0.77** |
| held-out fault recall | **0.77**（时间基准统一 + ±3 s 缓冲后） |

### 5.3 三条自适应路线的 A/B 结论（P3/P4/P5，median-of-3）

| 路线 | normal | corridor | single_plane | fault 场景 | 判定 |
|---|---|---|---|---|---|
| **P3 整体降权** | 零影响 | 无显著差异 | **变差 4.5×** | holdout **变差 14×** / stress **变差 8.7×** | ❌ 证伪（严重场景确定性有害） |
| **P4 方向性抑制** | 安全 | 安全 | **变差 3.7×** | **改善 15~50%**（方差大、机制未明） | ❌ 主体证伪；✅ **matching gate 有效** |
| **P5 帧级拒绝** | 零侵入 | 零拒帧 | 零误杀 | **变差 5.8~26×** | ❌ 证伪（"拒绝→漂移→拒绝"负反馈） |

> P4.5 标定补充（`single_plane_run1`）：β=0.5 与 degen 持平（1.561 vs 1.576），β=0.3 / β=0.7 变差（2.128 / 2.033）；
> corridor A/B 持平（0.0396 vs 0.0372）——即**不存在稳定的最优 β**。

### 5.4 定位精度与"解耦"证据

| 场景 | ATE (m) | 口径 |
|---|---|---|
| 健康室内（4 张地图） | **0.010 ~ 0.153** | `report/汇总报告.md` §5.1 |
| corridor / single_plane / open_ground | **~0.10 / 1.75 / 1.79** | 受控退化场景（§5.1） |
| 走廊几何退化 vs 单面墙 vs 人为强故障 | **~0.034 / 7.4 / 4.08** | 解耦论证（§2.2，与上一行不同实验批次） |

> **结论：几何退化程度与误差后果并非单调对应**——"检测到退化"报告的是**可观测性风险**，
> 误差后果必须由 GT 单独量化。这解释了"很多退化检测工作检出了却看不到误差上涨"的现象。
> （绝对数值随实验批次/场景不同，引用时请以对应 `experiment_XXX/result.yaml` 为准。）

### 5.5 工程与可复现性证据

| 项目 | 数值 |
|---|---|
| 自包含实验目录 | **175 个**（每个含 `config.yaml` 参数快照 + 轨迹 + 多路 CSV + `result.yaml`） |
| canonical rosbag | **5 套**（其中 P2 冻结 3 套：`corridor_run2` 113.7 s、`normal_indoor_run1`、`single_plane_run1`） |
| 故障原语 | **4 类**（dropout / fov_crop / outlier / noise），固定 seed |
| 室内地图 / 模型 | **4 张** / **182 个**（包内自包含） |
| 里程碑 tag | **4 个**（`p2-final` / `p3-final` / `p4-final` / `p5-final`） |
| parity check 精度 | P3 `max\|err\| = 5e-6`；P4 **535/535 帧 100%** |
| 主循环实时性 | fast_lio ~10 Hz / 100 ms（退化极端帧 900 ms 尖峰属场景特性） |
| 有效实验稳定性 | normal_indoor 稳定 **94 s**、轨迹 22.4 m、覆盖面积 **×4**、零翻车 |
---

## 6. 指标参数字典

### 6.1 `result.yaml` —— 每个实验的"成绩单"

每个 `data/experiments/experiment_XXX/result.yaml` 由 `src/lio_evaluation/scripts/compute_metrics.py` 生成。
下面逐字段说明含义、单位与计算口径。

#### 6.1.1 `trajectory` —— 轨迹与关联规模

| 字段 | 含义 | 单位 / 取值 |
|---|---|---|
| `n_gt` | 真值轨迹帧数（Gazebo P3D `/gt_odom`） | 帧 |
| `n_est` | 估计轨迹帧数（FAST-LIO `/Odometry`） | 帧 |
| `n_used` | 时间关联后**实际参与评测**的帧数（`max_diff = 0.05 s`） | 帧 |
| `duration_s` | 评测时长 = 估计轨迹首末时间戳之差 | s |
| `length_m` | 估计轨迹的**累计路径长度**（逐帧位移和） | m |
| `dt_median_s` | 估计轨迹相邻帧时间间隔的**中位数**（≈0.1 → 10 Hz） | s |

#### 6.1.2 `ate` —— 绝对轨迹误差（evo APE, translation）

| 字段 | 含义 | 单位 |
|---|---|---|
| `rmse_m` / `mean_m` / `median_m` / `max_m` / `std_m` | APE 的 RMSE / 均值 / 中位数 / 最大值 / 标准差 | m |
| `align` | 对齐方式，固定为 `umeyama_se3`（SE(3) Umeyama，**不估尺度**） | — |

> 时间对齐：GT 按估计时间戳**线性插值位置 + slerp 姿态**后再关联；
> 坐标对齐：用 SE(3) Umeyama 把估计轨迹对齐到 GT，以吸收 `camera_init` 与 `map` 的原点差异。

#### 6.1.3 `rpe` —— 相对位姿误差（evo RPE，1 m 间隔）

| 字段 | 含义 | 单位 |
|---|---|---|
| `delta_m` | RPE 的路径间隔，固定 `1.0` | m |
| `rmse_m` / `mean_m` / `median_m` / `max_m` | RPE 统计量 | m |
| `align` | `umeyama_se3` | — |
| （整体为 `null`） | 轨迹过短（机器人几乎未移动/出生点卡住）时 evo 抛 `FilterException`，**优雅降级为 null**，ATE 与 directional 仍保留 | — |

#### 6.1.4 `degeneracy` —— 退化检测统计（双检测器）

`degeneracy.exact` = **内嵌精确检测器**（FAST-LIO 内部，基于几何 Hessian）；
`degeneracy.proxy` = **外部几何代理检测器**（`degeneracy_proxy.cpp`，kNN/PCA 从 `/cloud_registered` 独立估计）。
**两者字段完全同构**：

| 字段 | 含义 | 单位 / 取值 |
|---|---|---|
| `frames` | 参与统计的帧数 | 帧 |
| `mean_score` | 退化分数均值（**越高越退化**） | 0~1 |
| `degraded_ratio` | 判定为退化（`is_degenerate=1`）的**帧占比** | 0~1 |
| `mean_trans_ratio` | `λmin(Ht)/λmax(Ht)` 均值 → **平动可观测性**（越小越弱） | 0~1 |
| `mean_rot_ratio` | `λmin(Hr)/λmax(Hr)` 均值 → **转动可观测性** | 0~1 |
| `mean_normal_concentration` | 局部法向**集中度**（点云平面朝向一致性） | 0~1 |
| `mean_effective_feature_num` | **有效特征数**均值（参与更新且残差合格的点数） | 个 |

`degeneracy.consistency` —— **双检测器一致性**：

| 字段 | 含义 | 单位 / 取值 |
|---|---|---|
| `pearson` | 两路 `score` 的**皮尔逊线性相关** | −1~1 |
| `spearman` | 两路 `score` 的**斯皮尔曼秩相关** | −1~1 |
| `rmse` | 两路 `score` 的 RMSE（两路 score 量纲/口径不同，**仅供趋势参考**） | 0~1 |
| `event_overlap` | 二值退化事件的 **IoU**（交集/并集） | 0~1 |
| `detection_delay_s` | proxy 相对 exact 的**首次检出延迟**（各自首次进入退化的时刻之差） | s |

#### 6.1.5 `runtime` —— 运行时开销

| 字段 | 含义 | 单位 |
|---|---|---|
| `loop_time_ms.mean / max / p99` | 滤波器主循环**单帧耗时** 均值/最大/P99 | ms |
| `cpu_percent.mean / max` | 进程 CPU 占用（`psutil`） | % |
| `rss_mb.mean / max` | 常驻内存 RSS | MB |
| `vms_mb.mean / max` | 虚拟内存 VMS | MB |

> 数据来源：`recorder_node.py` 落 `frametime.csv`，`monitor_node.py` 落 `resource.csv`，
> `compute_metrics.py` 的 `merge_runtime()` 按时间戳就近合并为 `runtime.csv` 后统计。
#### 6.1.6 `directional` —— 误差的方向性分解（弱方向分析）

| 字段 | 含义 | 单位 / 取值 |
|---|---|---|
| `pos_err_rms_m.x / .y / .z` | 位置误差在**各轴**的 RMS | m |
| `pos_err_total_rms_m` | 位置误差总 RMS（= ATE RMSE，用于自校验） | m |
| `rot_err_mean_deg` / `rot_err_max_deg` | 姿态误差 均值 / 最大 | ° |
| `weak_dir_err_rms_m` | **沿"弱方向"投影后的误差 RMS** ← 判断误差是否真被弱方向主导的核心指标 | m |
| `axis_error_growth_m.x / .y / .z` | 各轴误差的**增长量**（末段 − 首段） | m |
| `max_growth_axis` | 增长最大的轴（`x` / `y` / `z`） | 枚举 |

#### 6.1.7 `attribution` —— 三层归因结果

| 字段 | 含义 | 单位 / 取值 |
|---|---|---|
| `n_frames` | 参与归因统计的帧数 | 帧 |
| `params_file` | 使用的阈值文件（`attribution_params.yaml`） | 文件名 |
| `tune` | 是否开启在线调参（正式结果固定 `false`） | bool |
| `error_bands_m.low` / `.mid` | 误差分带阈值（决定 `band_distribution` 的 LOW / MID / HIGH） | m |
| `label_distribution.<LABEL>` | 四类标签的帧占比：`NORMAL` / `GEOMETRY_DEGENERATION` / `CORRESPONDENCE_FAILURE` / `IMU_LOW_EXCITATION` | 0~1 |
| `label_error_condition.<LABEL>` | **每个标签**下的误差后果统计（见下） | — |
| `flag_error_condition.<FLAG>` | **每个 flag 为真**时的误差后果统计（`is_geometry_degen` / `is_matching_failure` / `is_imu_low_excitation`） | — |
| `classification.annotated_frames` | 有故障注入标注的帧数 | 帧 |
| `classification.per_annotation[]` | 每个注入段：`name` / `ground_truth` / `n_frames` / `predicted_majority` / `flag_detection_rate` | — |
| `classification.per_class{}` | 逐类：`n_true` / `n_pred` / `precision` / `recall` | — |

`label_error_condition.<X>` 与 `flag_error_condition.<X>` 使用**同一套子字段**：

| 子字段 | 含义 | 单位 |
|---|---|---|
| `mean_m` / `median_m` / `p90_m` / `max_m` / `rmse_m` | 该条件下的位置误差统计 | m |
| `n_frames` / `n_ratio` | 命中帧数 / 占比 | 帧 / 0~1 |
| `band_distribution.LOW / .MID / .HIGH` | 误差落在 低/中/高 带的**比例**（带边界由 `error_bands_m` 定义） | 0~1 |
| `weak_dir_err_fraction_mean` | 误差中沿**弱方向**的占比均值 ← 验证"弱方向主导"假设 | 0~1 |

> 该区块是"**检出退化 ≠ 误差变大**"论断的直接证据来源：
> 对比 `label_error_condition` 各标签的 `rmse_m` 与 `band_distribution`，
> 可以看到几何退化标签下误差可能仍在 MID 带，而匹配失败标签下 HIGH 带占比显著抬升。

#### 6.1.8 示例（节选，真实数据）

```yaml
experiment: experiment_129
trajectory: { n_gt: 8479, n_est: 848, n_used: 848, duration_s: 84.702, length_m: 16.360, dt_median_s: 0.1 }
ate:  { rmse_m: 0.2109, mean_m: 0.1703, median_m: 0.1245, max_m: 0.4765, std_m: 0.1245, align: umeyama_se3 }
rpe:  { delta_m: 1.0, rmse_m: 0.1974, mean_m: 0.1578, median_m: 0.1540, max_m: 0.4982, align: umeyama_se3 }
degeneracy:
  exact: { frames: 848, mean_score: 0.4461, degraded_ratio: 0.7441, mean_trans_ratio: 0.0977, mean_rot_ratio: 0.0502 }
  proxy: { frames: 846, mean_score: 0.0803, degraded_ratio: 0.0579, mean_trans_ratio: 0.2547, mean_rot_ratio: 0.0255 }
  consistency: { pearson: 0.4999, spearman: 0.6108, rmse: 0.4274, event_overlap: 0.0777, detection_delay_s: 0.4 }
attribution:
  label_distribution: { NORMAL: 0.219, GEOMETRY_DEGENERATION: 0.547, CORRESPONDENCE_FAILURE: 0.233, IMU_LOW_EXCITATION: 0.0 }
  classification:
    per_class:
      correspondence_failure: { n_true: 479, n_pred: 316, precision: 1.0, recall: 0.6597 }
```
（完整文件见任意 `data/experiments/experiment_*/result.yaml`）

### 6.2 `attribution_params.yaml` —— 归因阈值表（冻结版 v1.1）

文件位置：`src/lio_evaluation/config/attribution_params.yaml`
（规则的**单一事实源**是 `scripts/attribution_rules.py`，online 节点与离线 `--tune` 共用）。

#### 6.2.1 geometry 路（几何可观测性，P1.3 标定，冻结）

| 参数 | 冻结值 | 含义 | 单位 | 判定方向 |
|---|---|---|---|---|
| `score_geom_th` | `0.50` | `/lio/health` 几何分高于此 → 可观测性弱 | 0~1 | 高于触发 |
| `ratio_t_th` | `0.10` | `trans_ratio` 低于此 → **平动弱约束** | 0~1 | 低于触发 |
| `ratio_r_th` | `0.02` | `rot_ratio` 低于此 → **转动弱约束** | 0~1 | 低于触发 |

#### 6.2.2 matching 路（匹配/对应失败，P2.3 基于 exp_029 的 C3 强化 fault 标定，冻结）

| 参数 | 冻结值 | 含义 | 单位 | 判定方向 |
|---|---|---|---|---|
| `feats_low_th` | `150` | `effective_feature_num` **绝对下限**（数量退化独立触发） | 个 | 低于触发 |
| `feats_critical_th` | `30` | 低于此**直接判定匹配失败** | 个 | 低于触发 |
| `ratio_low_th` | `0.20` | `effective_feature_ratio` 低于此 → 有效占比不足 | 0~1 | 低于触发 |
| `residual_high_th` | `0.05` | `mean_residual` 高于此 → 残差异常 | m | 高于触发 |
| `residual_p90_th` | `0.08` | `residual_p90` 高于此 → 局部匹配恶化（更敏感） | m | 高于触发 |
| `ratio_drop_th` | `0.30` | 相对**滚动基线**的有效占比**骤降**阈值 | 比例 | 跌幅超过触发 |
| `num_drop_th` | `0.40` | 相对滚动基线的有效特征数**骤降**阈值 | 比例 | 跌幅超过触发 |
| `ratio_baseline_window_s` | `30.0` | 滚动基线窗口（长 fault 段防基线被污染） | s | — |

> 标定记录：`residual_p90_th` 0.20→0.08（更敏感）、`feats_low_th` 100→150（corridor 数量下限）、
> `ratio_drop_th` 0.50→0.30、`num_drop_th` 0.50→0.40（0.7 会误伤干净场景正常波动）、
> `ratio_baseline_window_s` 10→30。验证口径：注入段 ±3 s 缓冲。

#### 6.2.3 IMU 路（激励不足，P2.3 motion-state 标定，冻结）

| 参数 | 冻结值 | 含义 | 单位 | 判定方向 |
|---|---|---|---|---|
| `gyr_exc_th` | `0.03` | `imu_gyr_excitation` 低于此 → **转动激励不足** | rad/s | 低于触发 |
| `acc_exc_th` | `0.30` | `imu_acc_excitation` 低于此 → **平动动态激励不足** | m/s² | 低于触发 |

> 标定依据：exp_037（`const_vel`）+ exp_038（`stop_go`）的 motion-state 标定，不用 GT 误差。
> stationary 命中 1.00、turning 误报 0.00；`const_vel` 命中 0.30（匀速直行 acc 抖动方差 ~0.37，物理上仍存在平动激励，属预期）。

#### 6.2.4 filter 路（v1 仅辅助确认，不单独触发）

| 参数 | 冻结值 | 含义 | 单位 | 判定方向 |
|---|---|---|---|---|
| `cov_pos_high_th` | `1.0e-2` | `pos_cov_trace` 高于此 → 位置不确定度高 | m² | 高于触发 |

#### 6.2.5 主标签 tie-break 优先级

| 参数 | 值 | 含义 |
|---|---|---|
| `priority[]` | `CORRESPONDENCE_FAILURE` → `GEOMETRY_DEGENERATION` → `IMU_LOW_EXCITATION` → `NORMAL` | 主标签 = `argmax(severity)`；并列时按此顺序取 |

调参流程：`python3 scripts/calibrate_thresholds.py --path matching --exps <calib_exp>`
→ `python3 scripts/attribution_validation.py <exp_dir> --tune --params <本文件>`
---

## 7. 模块成果清单

### 7.1 `src/lio_interfaces/` —— 自定义消息接口包（纯接口，ament_cmake + rosidl）

单独成包的好处：`fast_lio`（生产者）与 `lio_evaluation`（消费者）共同依赖它，**依赖方向不倒置**
（核心算法包不依赖评测工具包）；字段注释里写清理论口径与坐标系约定 —— **接口即文档**。
采用"向后追加字段"策略，保证 `p2-final` 等冻结 tag 的旧脚本仍兼容。

| 消息 | 用途 |
|---|---|
| `DegeneracyScore.msg` | P1 退化分数、`trans_ratio` / `rot_ratio`、弱方向向量、`is_degenerate` |
| `DegeneracyIteration.msg` | IEKF 迭代过程观测（`iteration` / `frame_index` / 残差 / 特征数 / ratio / p90；`iteration==1` 即 first-iteration 口径） |
| `LioHealth.msg` | P2 健康信号聚合（geometry / matching / covariance / IMU 激励四路），并含 P3/P4/P5 扩展字段 |
| `ErrorAttribution.msg` | P2 归因输出：多标签 flags + 各类 severity + 触发主因快照 |

### 7.2 `src/lio_evaluation/` —— 自研评测与编排包（C++/Python 混合）

| 类别 | 文件 | 说明 |
|---|---|---|
| C++ 节点 | `src/degeneracy_proxy.cpp` | **外部几何代理检测器**（kNN/PCA 从 `/cloud_registered` 独立估计退化） |
| 运行编排 | `scripts/run_experiment.py` | `prepare / run-online / replay / eval / all` 五段式；参数 `--world --scenario --drive(wander\|stop_go\|const_vel) --fault --teleop --headless --adaptive --directional --robust-gate` |
| 数据采集 | `scripts/recorder_node.py`、`monitor_node.py`、`health_watch.py`、`drive_node.py` | TUM 轨迹 + 多路 CSV 落盘；`psutil` 资源监控；翻车/卡死/RTF 看门狗；反应式避障漫游 |
| 场景与故障 | `scripts/generate_worlds.py`、`lidar_fault_injector.py` | 受控退化场景生成（corridor / single_plane / open_ground）；4 类故障原语 |
| 归因 | `scripts/attribution_rules.py`（单一事实源）、`attribution_node.py`、`attribution_validation.py`、`config/attribution_params.yaml` | rule-based 归因 + 两层验证 + 阈值表（可 `--tune`） |
| 评测 | `scripts/compute_metrics.py`、`directional_analysis.py`、`aggregate_experiments.py`、`calibrate_thresholds.py` | 时间对齐 + evo ATE/RPE；弱方向误差分解；跨实验聚合；阈值标定 |
| 机制验证 | `scripts/p3_ab_compare.py`、`p4_sanity_1d2d.py`、`p4_synthetic_test.py`、`p4_parity_check.py`、`p4_batch_replay.py`、`p5_analyze_first_iter.py`、`p5_synthetic_test.py`、`p5_parity_check.py`、`p5_batch_replay.py`、`parity_check.py` | sanity / synthetic / **parity** / 批量 A-B |

### 7.3 FAST-LIO2 内核改造点（最小侵入，全部默认关闭）

| 文件 | 改造 | 对应阶段 |
|---|---|---|
| `localization/FAST_LIO/include/.../esekfom.hpp` → `update_iterated_dyn_share_modified()` | ① 写死的测量协方差 → 运行时变量 `lidar_meas_cov_` | P3 |
| 同上（插入点 A） | `dx_` 计算后、`boxplus` 前，投影其**平动分量**到弱方向 | P4 |
| 同上（插入点 B） | `P_ = L_ − K_x P_` 前投影 `K_x` 的 pos 三行，**保证协方差不自相矛盾** | P4 |
| 同上（插入点 C） | 第一次 `h_dyn_share()` 之后、K 计算之前的 **first-iteration reject hook**；拒绝时 `x_ = x_propagated; P_ = P_propagated` | P5 |
| `localization/FAST_LIO/src/laserMapping.cpp` | `compute_geometric_hessian()`（读 `h_x` 累加 `H6/Ht/Hr`）、`publish_degeneracy_score()`、P2 health 聚合、P3 adaptive controller、P4 `update_directional_cache()/compute_directional_trigger()/compute_directional_matching_gate()`（`Ht` 时间累积解决特征向量符号歧义）、P5 robust gate 状态机 | P1–P5 |

### 7.4 导航 / 仿真主线包

| 包 | 作用 |
|---|---|
| `src/get_urdf/` | 机器人 URDF（`simple_car.urdf`：`base_footprint/chassis/4 轮/livox_frame`）+ **4 张 world** + **182 模型自包含** + `launch/get_urdf_launch.py`（world / 出生位姿 / 资源路径参数化） |
| `src/livox_ros_driver2/`、`src/livox_laser_simulation_RO2/` | 实机 / 仿真 LiDAR 驱动（**同话题名** `/livox/lidar`） |
| `src/localization/FAST_LIO`、`point_lio`、`Sophus` | LIO 后端与数学依赖（可切换） |
| `src/lio_interface/` | LIO 内部坐标系 → 标准 `odom`（`fastlio_*_launch.py` 订阅 `/Odometry`；`pointlio_*` 订阅 `/aft_mapped_to_init`） |
| `src/sensor_scan_generation/` | 发布 `/odom`、`odom→base_footprint` TF、`/registered_scan` |
| `src/registration/global_relocalization_kiss_matcher/` | KISS-Matcher coarse-to-fine 无初值初始化 → small_gicp 连续跟踪；失败自动回退；支持 `/initialpose` 修正 |
| `src/me_nav2_bringup/` | Nav2 launch / 参数 / 2D 地图 / PCD（`nav2_params.yaml`、`slam_toolbox_params.yaml`、`Pointcloud2d_3d.yaml`） |
| `src/ground_truth_bridge/` | Gazebo P3D 真值 → `/gt_path` + TUM 落盘 |
| `src/gui_teleop/` | WASD GUI 遥控（速度调节 + 急停） |
| `src/pcd2pgm-master/` | PCD → 2D 栅格离线工具 |
### 7.5 脚本（`scripts/`）

| 命令 | 作用 |
|---|---|
| `./build.sh` | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` |
| `./mapping_sim.sh` / `./mapping_real.sh` | 仿真 / 实机建图 |
| `./nav2_sim.sh [world] [map] [pcd]` / `./nav2_real.sh` | 仿真 / 实机导航 |
| `./save_map.sh` / `./save_pcd.sh` | 保存 2D `pgm/yaml` 与 3D `PCD` |
| `./record_bag.sh` | 录制评测 bag（含真值） |
| `./show_tf_tree.sh` | TF 树排障 |
| `./mapping3d_sim.sh` / `./mapping3d_headless.sh` / `./RUN.sh` | 3D 建图与组合启动 |

### 7.6 核心话题（速查）

| 话题 | 类型 | 发布者 | 用途 |
|---|---|---|---|
| `/livox/lidar` | `PointCloud2` / `CustomMsg` | Livox 驱动 / 仿真 | 原始点云 |
| `/livox/imu` | `sensor_msgs/Imu` | Livox 驱动 / 仿真 | 200 Hz IMU |
| `/cloud_registered` | `PointCloud2` | FAST-LIO / Point-LIO | LIO 内部注册帧 |
| `/registered_scan` | `PointCloud2` | `sensor_scan_generation` | 重定位/切片输入 |
| `/odom` | `nav_msgs/Odometry` | `sensor_scan_generation` | 里程计 |
| `/gt_odom` / `/gt_path` | `Odometry` / `Path` | Gazebo P3D + `ground_truth_bridge` | 真值 |
| `/scan` | `LaserScan` | `pointcloud_to_laserscan` | 3D→2D 切片 |
| **`/lio/degeneracy_score`** | `DegeneracyScore` | FAST-LIO（P1） | **退化分数** |
| **`/lio/health`** | `LioHealth` | FAST-LIO（P2） | **健康信号** |
| **`/lio/error_attribution`** | `ErrorAttribution` | `attribution_node`（P2） | **误差归因** |

> 误差状态 `state_ikfom` 为 23 维：`pos 0-2 / rot 3-5 / offset_R 6-8 / offset_T 9-11 / vel 12-14 / bg 15-17 / ba 18-20 / grav 21-22`；
> 几何 Hessian 中 `h_x` 列 0-2 对应平动（世界系法向量 `n`），列 3-5 对应转动（body 系 `p×n`）；
> `weak_translation_direction` 为**世界系**纯平动方向，与 GT 误差比较前需先乘 Umeyama 对齐旋转 `R_align`。

### 7.7 数据资产

| 资产 | 内容 |
|---|---|
| 实验目录 | **175 个** `data/experiments/experiment_XXX/`（`config.yaml` 快照 + `mid360.yaml` + `groundtruth.txt` + `fastlio.txt` + `degen_exact.csv` / `degen_proxy.csv` / `lio_health.csv` / `error_attribution.csv` + `result.yaml` + `logs/`） |
| canonical bags | `corridor_run2`（113.7 s，`/livox/lidar` 1029 帧 + `/livox/imu` 17095 条 + `/gt_odom` 10280 帧）、`normal_indoor_run1`、`single_plane_run1`（仅原始三话题，P2 冻结） |
| 补充数据 | `corridor_run1`（P1 标定）、`single_plane_run2`（P4 holdout，84 s） |
| 故障剧本 | `data/fault_configs/`：`corridor_run1_faults.yaml`、`corridor_holdout_faults.yaml`(seed=7) 等 |
| 聚合结果 | `data/experiments/aggregate_summary.yaml` |

> 说明：`data/` 体积较大（含 rosbag 二进制），**未纳入本仓库**；请按 `report/*.md` 中的命令自行生成/录制。
---

## 8. 结论

### 8.1 三条"直觉自适应"路线全部被证伪

| 路线 | 直觉 | 实测 | 机制解释 |
|---|---|---|---|
| P3 整体降权 | "退化时少信雷达" | ❌ 严重场景变差 4.5~14× | 降权 → IMU 独自积分漂移 → 失配 → 误判匹配失败 → **继续降权**（正反馈）；加 3 s 时长上限后发散被按住（10 m → 1.4 m），但**路线本身有缺陷** |
| P4 方向性抑制 | "只压弱方向" | ❌ 主体证伪（single_plane 变差 3.7×） | **弱约束 ≠ 有害约束**：把状态估计的弱方向分量交给 IMU，而 IMU 精度有限时是净损失；✅ 但 `matching gate`（匹配差时禁触发）被验证有效 |
| P5 帧级拒绝 | "可疑帧整帧丢弃" | ❌ 持续 fault 变差 5.8~26× | 拒绝 → 该帧只剩 IMU 递推 → 漂移 → 下一帧基线/残差更差 → **继续拒绝**（负反馈） |

### 8.2 唯一被验证有效的方向：**检测 + 归因 + 门控**

- **P1 检测**：双检测器（exact 几何 Hessian + proxy 几何代理）**灵敏度互补**，单检测器必漏检一类退化。
- **P2 归因**：三层框架（Health → Attribution → Consequence）用多标签区分三类风险源，干净场景零误报、故障场景 precision 1.00。
- **P4 matching gate**：在"匹配确实失败"时禁止方向性干预 —— 这是三条路线里**唯一产生正向收益**的控制逻辑。

> 对 FAST-LIO2 这类已经很强壮的滤波器，**介入式防护（改状态估计）难以超越原版**；
> 工业界更合理的做法是在 LIO **外面**做"健康监测 + 重定位兜底"，而不是改滤波核。

### 8.3 诚实的负面结论（为什么它们同样是成果）

1. **检测到退化 ≠ 误差变大**：几何退化报告的是**可观测性风险**，误差后果必须由 GT 单独量化；
   走廊类退化 ATE 可低至 ~0.03~0.10 m，而 single_plane / 人为强故障可达 1.75~7.4 m。
2. **走廊弱方向是 z 不是 bug**：MID-360 视场 −7°~+52°、89% 光束朝上、0.8 m 走廊地面点仅 6%，
   z 约束天然最弱（已用 bag 逐帧点云统计验证，见 `report/P1.3 §4`）。
3. **FAST-LIO 回放存在非确定性**：同 bag 三次回放 ATE 0.104~0.204 m 浮动；选择
   **"承认它并用 median-of-3 统计消化"**，而不是假装两次回放一致。
4. **三条路线的边界被量化**：即使结论是"无效"，也给出了明确的失效倍数与机制，为团队省下试错成本。

---

## 9. 可复现性

### 9.1 里程碑 tag

```bash
git tag                 # p2-final / p3-final / p4-final / p5-final
git checkout p3-final   # 回到 P3 冻结时刻的完整代码状态
```

| tag | 冻结内容 |
|---|---|
| `p2-final` | 基建入库 + 阈值标定 + 修复 + canonical bags |
| `p3-final` | Adaptive LiDAR Weighting 实现与证伪 |
| `p4-final` | state-layer 方向投影 + matching gate |
| `p5-final` | 当前帧鲁棒测量拒绝 + 恢复状态机 |

### 9.2 一次实验的完整流程

```bash
WS=~/FastLIO2_ROS2_ErrorAttribution ; cd $WS
python3 src/lio_evaluation/scripts/run_experiment.py all \
    --scenario corridor --drive stop_go \
    --fault data/fault_configs/corridor_run1_faults.yaml --duration 95
# → data/experiments/experiment_XXX/  (config.yaml 含 git commit 与 resolved 参数快照)
python3 src/lio_evaluation/scripts/aggregate_experiments.py   # → aggregate_summary.yaml
```

### 9.3 结论的可靠性链条

```
数值 sanity test  →  synthetic test  →  C++/Python parity check
      ↓                    ↓                      ↓
   零侵入验证       标定集/验证集分离       median-of-3 A/B
      ↓                    ↓                      ↓
  Welch t 检验             precision/recall        只承认超噪声差异
      (p=0.177)             (1.00 / 0.65~0.77)      → 可复现结论（含负面）
```

---

## 10. 目录结构

```text
FastLIO2_ROS2_ErrorAttribution/
├── src/
│   ├── lio_interfaces/                  # ① 自研：4 个自定义 msg（接口包）
│   ├── lio_evaluation/                  # ② 自研：C++ proxy 检测器 + 30+ 评测/编排脚本
│   ├── localization/FAST_LIO/           # ③ 内核改造（esekfom.hpp / laserMapping.cpp）
│   ├── localization/point_lio/          #    备用 LIO 后端
│   ├── get_urdf/                        #    仿真：URDF + 4 world + 182 模型（自包含）
│   ├── livox_ros_driver2/               #    实机驱动
│   ├── livox_laser_simulation_RO2/      #    仿真 LiDAR 驱动
│   ├── lio_interface/                   #    LIO → 标准 odom TF 桥接
│   ├── sensor_scan_generation/          #    /odom + /registered_scan
│   ├── registration/global_relocalization_kiss_matcher/   # KISS-Matcher + small_gicp
│   ├── me_nav2_bringup/                 #    Nav2 参数/地图/PCD
│   ├── ground_truth_bridge/             #    P3D 真值 → /gt_path
│   ├── gui_teleop/  pcd2pgm-master/     #    GUI 遥控 / PCD→PGM
│   └── gld_robot_description/           #    实机 URDF
├── scripts/                             # 一键构建/建图/导航/保存/录包（13 个脚本）
├── report/                              # P0–P5 全部实验报告（18 篇，含汇总报告.md）
├── docs/                                # 演示 gif / 实验记录模板
├── README.md  README_upstream.md        # 本文件 / 上游原始 README（归档）
├── 环境搭建.md  Environment_Setup_EN.md   # 环境搭建（中/英）
├── LICENSE                              # MIT（© 2025 Ikunio）
└── data/                                # 实验数据（未入库，需自行生成）
```

---

## 11. 许可证与致谢

- 本项目基于 **[Ikunio/Lidar_nav2_ws](https://github.com/Ikunio/Lidar_nav2_ws)**（**MIT License, © 2025 Ikunio**），
  仓库根 [`LICENSE`](./LICENSE) 保留其原始版权声明。
- 同时包含多个第三方开源组件（各自遵循其原始许可）：**FAST-LIO2**（GPL-2.0）、**Point-LIO**（GPL-2.0）、
  **ikd-Tree**、**livox_ros_driver2 / Livox-SDK2**、**KISS-Matcher**、**small_gicp**、**Sophia / Sophus**、
  **rtabmap_ros** 等。二次分发请遵循各组件的许可条款。
- 感谢上游作者提供的导航底座与仿真基建思路。

---

## 附录：reports 导航

| 报告 | 内容 |
|---|---|
| `report/汇总报告.md` | 项目总述 + 六大突出点 + 面试题库 + 贡献与踩坑（**建议先读**） |
| `report/P0*.md` | 仿真场景参数化、模型自包含、多室内地图集成 |
| `report/P1模块构建.md` | 双检测器架构设计与 P2–P5 蓝图 |
| `report/P1.1` / `P1.2` / `P1.3` | 首轮评测 / 避障漫游与看门狗 / 数值等价+阈值标定+弱方向分析 |
| `report/P2误差归因模块.md` | 三层框架、fault injection、两层验证、precision/recall |
| `report/P3自适应权重.md` | adaptive R 缩放实现与 median-of-3 证伪结论 |
| `report/P4方向性抑制.md` | state-layer projector、matching gate、证伪与 gate 正向结论 |
| `report/P5当前帧鲁棒测量拒绝.md`、`P5.0` | current-frame reject、恢复状态机、诚实负面结论 |

---

*本 README 由 P0–P5 阶段的源码、`result.yaml` 与 `report/*.md` 整理而成；
所有量化结论均可在对应 `data/experiments/experiment_XXX/result.yaml` 中追溯到原始数据。*
