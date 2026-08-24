# 实验报告归档 (Experiments)

本目录是**误差归因闭环**的落地载体：每次"仿真评测 → 误差分析 → 定位根因 → 修复验证"
的过程都要在这里留下一份可复现的实验报告。

## 误差归因闭环怎么做

```
① 测 (Measure)  在仿真中录制 真值(/gt_odom) + 里程计(/odom) 数据，
                 用 evo 计算 ATE / RPE —— 把"感觉准不准"变成数字
② 归因 (Attribute)  误差大时不盲调，用控制变量法(消融实验)锁定根因，
                 例如: IMU 协方差? 点云降采样? 退化环境? 外参估计?
③ 修 (Fix)  针对根因做定向修改 (改参数 / 改算法 / 改预处理)
④ 验 (Verify)  同一测试路线重跑，对比前后指标，确认误差下降，
                 然后写报告，进入下一个循环
```

关键前置条件（本项目已具备）：
- 真值数据: `get_urdf` 的 URDF 内置 P3D 插件发布 `/gt_odom`
- 数据采集: `scripts/record_bag.sh` 一键录制 rosbag
- 固定测试路线: 手动遥操作时尽量走同一条路线，或用自动巡航脚本保证可复现

## 报告模板

每次实验复制 `template.md` 并使用 `exp_YYYYMMDD_NN.md` 命名。

## 评测命令速记

```bash
# 1. 录数据 (先启动 nav2_sim.sh，再执行)
cd scripts && ./record_bag.sh          # 或 BAG_NAME=xx ./record_bag.sh

# 2. 从 rosbag 导出轨迹 (TUM 格式)
#    真值: /gt_odom
#    里程计: /odom
#    示例(需要 evo 工具): 见 scripts/eval 说明

# 3. 误差分析
evo_ape tum gt.tum odom.tum -a --plot   # 绝对位姿误差
evo_rpe tum gt.tum odom.tum -a --plot   # 相对位姿误差
```