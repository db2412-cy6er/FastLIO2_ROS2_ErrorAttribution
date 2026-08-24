#!/usr/bin/env bash

# 仿真建图启动脚本（支持多场景切换）
#
# 用法:
#   ./mapping_sim.sh                 # 默认 test_world 建图
#   ./mapping_sim.sh normal_indoor  # 指定场景

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
export WORKSPACE_ROOT
cd "$WORKSPACE_ROOT" || exit 1

WORLD="${1:-test_world}"
MAP="${2:-nav_test_4_27}"

echo ">>> 启动建图场景: world=$WORLD, map=$MAP"

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

# GUI 控制
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

gnome-terminal --title="slam_toolbox 建图" -- bash -c "
source install/setup.bash;
ros2 launch slam_toolbox online_async_launch.py \\
    slam_params_file:=$WORKSPACE_ROOT/src/me_nav2_bringup/config/slam_toolbox_params.yaml"

gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup my_nav2_launch.py map:=$MAP use_sim_time:=false"
