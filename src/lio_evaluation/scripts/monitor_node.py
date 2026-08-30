#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lio_evaluation 资源监控节点 (M4)

- 以 1s 周期采样 fastlio_mapping 进程的 CPU / 内存 (psutil) -> resource.csv
- 订阅 /Odometry 统计帧间隔 (loop time) -> frametime.csv
- 离线阶段由 compute_metrics.py 合并为最终 runtime.csv
"""

import os
import time
import csv
import psutil
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class MonitorNode(Node):
    def __init__(self):
        super().__init__("lio_monitor")
        self.out_dir = self.declare_parameter("output_dir", "").value
        self.proc_name = self.declare_parameter("proc_name", "fastlio_mapping").value
        self.sample_period = self.declare_parameter("sample_period", 1.0).value

        if not self.out_dir:
            raise RuntimeError("output_dir must be set")
        os.makedirs(self.out_dir, exist_ok=True)

        self._res_file = open(os.path.join(self.out_dir, "resource.csv"), "w")
        self._res_writer = csv.writer(self._res_file)
        self._res_writer.writerow(["wall_time", "cpu_percent", "rss_mb", "vms_mb", "threads"])

        self._ft_file = open(os.path.join(self.out_dir, "frametime.csv"), "w")
        self._ft_writer = csv.writer(self._ft_file)
        self._ft_writer.writerow(["timestamp", "loop_time_ms"])
        self._last_stamp = None

        self.create_subscription(
            Odometry, "/Odometry",
            lambda msg: self._on_odom(msg), 10)

        self._timer = self.create_timer(
            self.sample_period, self._sample_resources)

        self._pid = None
        self._last_cpu_sample = None
        self.get_logger().info(f"lio_monitor started: proc={self.proc_name}, dir={self.out_dir}")

    def _find_pid(self):
        if self._pid is not None:
            return self._pid
        for p in psutil.process_iter(["name", "cmdline"]):
            try:
                if p.info["name"] and self.proc_name in p.info["name"]:
                    self._pid = p.pid
                    self.get_logger().info(f"找到进程 {self.proc_name}: pid={self._pid}")
                    return self._pid
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None

    def _on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_stamp is not None and t >= self._last_stamp:
            dt_ms = (t - self._last_stamp) * 1000.0
            self._ft_writer.writerow([f"{t:.9f}", f"{dt_ms:.3f}"])
            self._ft_file.flush()
        self._last_stamp = t

    def _sample_resources(self):
        pid = self._find_pid()
        row = [f"{time.time():.3f}"]
        if pid is None:
            row += ["nan", "nan", "nan", "nan"]
            self.get_logger().warn_throttle(5.0, f"未找到进程 {self.proc_name}")
        else:
            try:
                p = psutil.Process(pid)
                cpu = p.cpu_percent(interval=None)
                if self._last_cpu_sample is None:
                    cpu = 0.0
                self._last_cpu_sample = cpu
                mem = p.memory_info()
                row += [f"{cpu:.2f}", f"{mem.rss / 1048576.0:.2f}",
                        f"{mem.vms / 1048576.0:.2f}", f"{p.num_threads()}"]
            except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                self.get_logger().warn(f"采样失败: {e}")
                row += ["nan", "nan", "nan", "nan"]
        self._res_writer.writerow(row)
        self._res_file.flush()

    def shutdown(self):
        self._res_file.close()
        self._ft_file.close()
        self.get_logger().info("lio_monitor closed files")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
