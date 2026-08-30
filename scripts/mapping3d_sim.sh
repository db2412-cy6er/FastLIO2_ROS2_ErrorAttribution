#!/usr/bin/env bash

# 仿真 3D 建图脚本（轻量版，不依赖 pointcloud_to_laserscan / slam_toolbox / Nav2）
#
# 用途:
#   1. 启动 Gazebo 世界 + FAST-LIO 3D 建图链路 + 真值桥接
#   2. 遥控/自动驾驶遍历环境后，关闭 FAST-LIO 自动产出 PCD 地图
#   3. 用 pcd2pgm 将 PCD 转为 2D 占用栅格地图
#
# 用法:
#   ./mapping3d_sim.sh [world]         # 默认 test_world，可传 normal_indoor

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1
export WORKSPACE_ROOT

WORLD="${1:-test_world}"

# 清掉旧的 map_file_path，确保本次产出的 PCD 干净可辨识
rm -f test.pcd test_cover.pcd 2>/dev/null

echo ">>> 启动 3D 建图场景: world=$WORLD"

# 1. 仿真真值桥接
gnome-terminal --title="真值桥接 GT" -- bash -c "
source install/setup.bash;
ros2 launch ground_truth_bridge ground_truth_bridge_launch.py use_sim_time:=true"

# 2. Gazebo 仿真环境
gnome-terminal --title="Gazebo 仿真" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch get_urdf get_urdf_launch.py world:=$WORLD"

# 3. FAST-LIO 里程计（仿真必须 use_sim_time:=true，否则与 /clock 失步）
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py use_sim_time:=true"

# 4. 里程计 TF 桥接
gnome-terminal --title="lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py"

# 5. odom/base_footprint TF 与 /registered_scan
gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# 6. 手动遥控（WASD；若要用自动巡航可注释本行，改用 scripts/patrol.py）
gnome-terminal --title="GUI 遥控" -- bash -c "
source install/setup.bash;
ros2 run gui_teleop gui_teleop_node"

echo ">>> 已开启全部 3D 建图节点。遥控遍历环境后，关闭 FAST-LIO 窗口即自动保存地图 PCD (test.pcd)"