#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 LiDAR fault injector (可控输入故障注入, 提供 correspondence/matching 退化真值)。

订阅原始 /livox/lidar (sensor_msgs/PointCloud2), 按配置在指定时间段注入故障,
发布 /livox/lidar_faulty 供 FAST-LIO 订阅。注入参数 + 起止时间 + random seed
全部记录到实验 config, 保证可重复; 同一 rosbag 可复放不同 fault level。

故障类型:
  dropout : 随机丢弃 ratio 比例的点                          (点稀疏)
  fov_crop: 只保留指定方位/俯仰范围内的点 (模拟视场收缩)
  outlier : 将 ratio 比例的点替换为随机离群点                (离群注入)
  noise   : 对所有点叠加 std_m 高斯测距噪声                  (量测噪声)

用法:
  python3 lidar_fault_injector.py --ros-args -p config:=fault_config.yaml -p use_sim_time:=true
"""
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

FIELDS = [pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
          pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
          pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
          pc2.PointField(name="intensity", offset=12, datatype=pc2.PointField.FLOAT32, count=1)]


def apply_faults(points, segs, rng, t_rel):
    """points: (N,4) [x,y,z,intensity]; 返回处理后的数组。"""
    if len(points) == 0:
        return points
    for s in segs:
        if s["start"] <= t_rel <= s["end"]:
            ftype = s["type"]
            if ftype == "dropout":
                ratio = float(s.get("ratio", 0.5))
                keep = rng.random(len(points)) >= ratio
                points = points[keep]
            elif ftype == "fov_crop":
                az_min = np.radians(float(s.get("az_min_deg", -60)))
                az_max = np.radians(float(s.get("az_max_deg", 60)))
                el_min = np.radians(float(s.get("el_min_deg", -20)))
                el_max = np.radians(float(s.get("el_max_deg", 30)))
                x, y, z = points[:, 0], points[:, 1], points[:, 2]
                r = np.sqrt(x * x + y * y)
                az = np.arctan2(y, x)
                el = np.arctan2(z, r)
                mask = (az >= az_min) & (az <= az_max) & (el >= el_min) & (el <= el_max)
                points = points[mask]
            elif ftype == "outlier":
                ratio = float(s.get("ratio", 0.05))
                radius = float(s.get("radius_m", 3.0))
                n_out = int(ratio * len(points))
                idx = rng.choice(len(points), size=n_out, replace=False)
                if n_out > 0:
                    d = radius * (0.5 + rng.random(n_out))
                    theta = rng.uniform(-np.pi, np.pi, n_out)
                    phi = rng.uniform(-np.pi / 2, np.pi / 2, n_out)
                    points[idx, 0] = d * np.cos(phi) * np.cos(theta)
                    points[idx, 1] = d * np.cos(phi) * np.sin(theta)
                    points[idx, 2] = d * np.sin(phi)
                    points[idx, 3] = 0.0
            elif ftype == "noise":
                std = float(s.get("std_m", 0.05))
                n = len(points)
                points[:, 0] += rng.normal(0, std, n)
                points[:, 1] += rng.normal(0, std, n)
                points[:, 2] += rng.normal(0, std, n)
            break  # 同一时刻只执行一种故障 (分段设计, 避免叠加语义混淆)
    return points


class FaultInjectorNode(Node):
    def __init__(self):
        super().__init__("lidar_fault_injector")
        cfg_path = self.declare_parameter("config", "").value
        if not cfg_path:
            self.get_logger().error("必须指定 -p config:=<fault_config.yaml>")
            raise RuntimeError("config param required")
        with open(cfg_path) as f:
            self.cfg = (yaml.safe_load(f) or {}).get("fault", {})
        self.out_topic = self.cfg.get("output_topic", "/livox/lidar_faulty")
        self.seed = int(self.cfg.get("seed", 42))
        self.segs = self.cfg.get("segments", [])
        self.rng = np.random.default_rng(self.seed)
        self.t0 = None
        self.last_log = 0.0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(PointCloud2, "/livox/lidar", self._on_cloud, qos)
        self.pub = self.create_publisher(PointCloud2, self.out_topic, qos)
        self.get_logger().info(
            f"fault injector 启动: /livox/lidar -> {self.out_topic}, "
            f"seed={self.seed}, segments={len(self.segs)}")

    def _on_cloud(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t_rel = now - self.t0
        active = [s for s in self.segs if s["start"] <= t_rel <= s["end"]]

        try:
            arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z", "intensity"))
        except Exception as e:
            self.get_logger().warn(f"点云解析失败: {e}")
            return

        pts = np.ascontiguousarray(arr, dtype=np.float32)
        if active:
            pts = apply_faults(pts, active, self.rng, t_rel)
            if now - self.last_log > 2.0:
                self.get_logger().info(
                    f"注入中 t_rel={t_rel:.1f}s: {[s['type'] for s in active]} "
                    f"({len(pts)} pts)")
                self.last_log = now

        # 保留原始 header.stamp (不重写): 保证与 /livox/imu /gt_odom 时间对齐,
        # 回放时 lio_health.csv / error_attribution.csv 可按同帧对齐。
        out = pc2.create_cloud(msg.header, FIELDS, pts.tolist())
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
