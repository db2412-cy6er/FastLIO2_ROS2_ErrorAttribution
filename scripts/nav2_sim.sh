#!/usr/bin/env bash

# 仿真导航启动脚本（支持多场景切换）
#
# 用法:
#   ./nav2_sim.sh                                 # 默认 test_world + nav_test_4_27
#   ./nav2_sim.sh normal_indoor normal_indoor normal_indoor   # 指定 world/map/pcd
#   ./nav2_sim.sh normal_indoor                  # 仅覆盖 world，map/pcd 沿用默认
#
# 场景参数:
#   WORLD : get_urdf/worlds/ 下的世界文件名（可省略 .world）
#   MAP   : me_nav2_bringup/map/ 下的 2D 地图名（可省略 .yaml）
#   PCD   : me_nav2_bringup/pcd/ 下的先验点云名（可省略 .pcd）

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
export WORKSPACE_ROOT
cd "$WORKSPACE_ROOT" || exit 1

WORLD="${1:-test_world}"
MAP="${2:-nav_test_4_27}"
PCD="${3:-nav_test_4_27}"

echo ">>> 启动导航场景: world=$WORLD, map=$MAP, pcd=$PCD"

# -----------------------------------------------------------------------------------
# 使用 fast-lio 作为里程计
# -----------------------------------------------------------------------------------
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py"

# 里程计接口
gnome-terminal --title="lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py"

# -----------------------------------------------------------------------------------
# # 使用 point-lio 作为里程计（取消注释即可切换）
# gnome-terminal --title="点云格式转换器" -- bash -c "
# source install/setup.bash;
# ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args \\
#   -p pcd_topic:=/livox/lidar \\
#   -p n_scan:=50 \\
#   -p horizon_scan:=360 \\
#   -p ang_bottom:=7.22 \\
#   -p ang_res_y:=1.248"
# gnome-terminal --title="Point-LIO 里程计" -- bash -c "
# source install/setup.bash;
# ros2 launch point_lio point_lio.launch.py use_sim_time:=True"
# gnome-terminal --title="lio_interface" -- bash -c "
# source install/setup.bash;
# ros2 launch lio_interface pointlio_lio_interface_launch.py"
# -----------------------------------------------------------------------------------

# 仿真真值桥接（/gt_odom -> /gt_path，供评测与 RViz 显示）
gnome-terminal --title="真值桥接 GT" -- bash -c "
source install/setup.bash;
ros2 launch ground_truth_bridge ground_truth_bridge_launch.py"

# GUI 控制（可能占用 /cmd_vel，与 Nav2 冲突时注释掉即可）
gnome-terminal --title="GUI控制" -- bash -c "
source install/setup.bash;
ros2 run gui_teleop gui_teleop_node"

# -----------------------------------------------------------------------------------
# Gazebo 仿真环境
# -----------------------------------------------------------------------------------
gnome-terminal --title="Gazebo 仿真" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch get_urdf get_urdf_launch.py world:=$WORLD"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py"

# 重定位（二选一）：有初值用 small_gicp，无初值用 KISS-Matcher
# gnome-terminal --title="small_gicp 重定位" -- bash -c "
# source install/setup.bash;
# ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py \\
#   prior_pcd_file:=$PCD use_sim_time:=true"

gnome-terminal --title="KISS + GICP 重定位" -- bash -c "
source install/setup.bash;
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py \\
  prior_pcd_file:=$PCD use_sim_time:=true"

# Nav2 导航（map 与 2D 地图对应）
gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup my_nav2_launch.py map:=$MAP use_sim_time:=false"
