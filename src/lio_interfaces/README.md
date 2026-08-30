# lio_interfaces

退化感知 LIO 改进项目的共享 ROS2 消息接口包。

## 消息

| 消息 | 用途 |
|---|---|
| `DegeneracyScore` | 每帧退化检测分数（P1 主消息） |
| `DegeneracyIteration` | 单次 IEKF 迭代观测（debug 模式，`degeneracy.debug_iterations=true` 时发布） |

## 设计要点

- 消息接口由本包统一持有，`fast_lio`（核心算法）与 `lio_evaluation`（评测/旁路检测）都只依赖本包，
  核心算法不依赖评测工具，避免依赖倒置。
- `DegeneracyScore` 的理论口径为 **LiDAR geometric Hessian / measurement Hessian**（见消息注释），
  仅反映 scan-to-map 匹配提供的几何约束，不是滤波器完整信息状态。
- 后续 `LioHealth`、`ErrorAttribution` 等消息预留在本包扩展。

## 构建

```bash
colcon build --packages-select lio_interfaces --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
