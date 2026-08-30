#!/usr/bin/env bash
# 无界面 3D 建图启动（日志可监控，便于自动化/排障）
# 与 scripts/mapping3d_sim.sh（gnome-terminal 版）二选一使用。
#
# 用法: ./mapping3d_headless.sh [world]

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

WORLD="${1:-test_world}"
LOG_DIR="$WORKSPACE_ROOT/data/logs/mapping3d_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash

killall -9 gzserver gzclient 2>/dev/null
rm -f test.pcd 2>/dev/null

start_one() {
  local name="$1"; shift
  nohup "$@" > "$LOG_DIR/$name.log" 2>&1 &
  echo $! > "$LOG_DIR/$name.pid"
}

start_one gazebo  ros2 launch get_urdf get_urdf_launch.py world:=$WORLD
start_one fastlio ros2 launch fast_lio mapping.launch.py use_sim_time:=true
start_one lio_interface ros2 launch lio_interface lio_interface_launch.py
start_one ssg       ros2 launch sensor_scan_generation sensor_scan_generation_launch.py
start_one gt        ros2 launch ground_truth_bridge ground_truth_bridge_launch.py
start_one teleop    ros2 run gui_teleop gui_teleop_node

sleep 3
echo ">>> 3D 建图已启动, 世界: $WORLD"
echo ">>> 日志目录: $LOG_DIR"
echo "$LOG_DIR" > "$WORKSPACE_ROOT/.last_mapping_log"
