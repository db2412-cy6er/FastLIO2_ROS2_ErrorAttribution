#!/usr/bin/env bash
# 数据采集脚本：录制传感器 + 里程计 + 真值话题，
# 用于离线 LIO 评测（误差归因闭环的数据基础）。
#
# 用法:
#   ./record_bag.sh                     # 默认录制到 data/bags/<时间戳>
#   BAG_NAME=exp1 ./record_bag.sh       # 指定包名
#   BAG_DIR=/tmp/bags ./record_bag.sh   # 指定输出目录

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

OUT_DIR="${BAG_DIR:-$WORKSPACE_ROOT/data/bags}"
BAG_NAME="${BAG_NAME:-$(date +%Y%m%d_%H%M%S)}"
TOPICS_FILE="${TOPICS_FILE:-$SCRIPT_DIR/bag_topics.txt}"

mkdir -p "$OUT_DIR"

if [ -f "$TOPICS_FILE" ]; then
  TOPIC_ARGS="--topics-file $TOPICS_FILE"
else
  TOPIC_ARGS="--topics /livox/lidar /livox/imu /odom /gt_odom /gt_path /registered_scan /scan /cmd_vel /tf /tf_static"
fi

echo ">>> 开始录制数据包: $OUT_DIR/$BAG_NAME"
echo ">>> 按 Ctrl+C 停止并保存"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag record $TOPIC_ARGS -o "$OUT_DIR/$BAG_NAME"
echo ">>> 数据包已保存: $OUT_DIR/$BAG_NAME"