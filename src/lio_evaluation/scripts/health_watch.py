#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运行期健康看门狗 — 量化并避免"实验提前结束 / 数据报废"。

订阅 /gt_odom（真值位姿）与 /cmd_vel（驱动指令），检测：
  - 翻车：|roll| 或 |pitch| > flip_rad(0.9 rad) → 记录 FLIP 事件；
    当 abort:=true 时写入 <output_dir>/ABORT 哨兵文件，run_experiment
    轮询到该文件即提前中止采集（避免继续收集垃圾数据）。
  - 卡死：收到非零 /cmd_vel 但连续 stuck_sec(15s) 位移 < stuck_min_m(0.1m)
    → STUCK 事件（避障正常时不应再出现）。
  - RTF：sim 时钟 vs 墙钟，每 rtf_window_s(10s) 计算一次，< rtf_warn(0.9)
    → RTF_LOW 事件（卡顿/仿真慢的量化依据）。
  - 心跳：每 heartbeat_s(10s) 记录一行 HEARTBEAT(t_sim, rtf, cmd_v)。

用法: python3 health_watch.py --ros-args -p output_dir:=EXP_DIR -p abort:=false
输出: <output_dir>/health.log  （可选中止: <output_dir>/ABORT）
"""

import math
import os
import time as _wall

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def roll_pitch_from_quat(x, y, z, w):
    """四元数(xyzw) → (roll, pitch) [rad]。"""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class HealthWatch(Node):
    def __init__(self):
        super().__init__("health_watch")
        self.out_dir = self.declare_parameter("output_dir", "").value
        self.abort = self.declare_parameter("abort", False).value
        self.flip_rad = self.declare_parameter("flip_rad", 0.9).value
        self.stuck_min_m = self.declare_parameter("stuck_min_m", 0.1).value
        self.stuck_sec = self.declare_parameter("stuck_sec", 15.0).value
        self.rtf_window_s = self.declare_parameter("rtf_window_s", 10.0).value
        self.rtf_warn = self.declare_parameter("rtf_warn", 0.9).value
        self.heartbeat_s = self.declare_parameter("heartbeat_s", 10.0).value

        if not self.out_dir:
            raise RuntimeError("output_dir 未设置，无法记录健康日志")
        os.makedirs(self.out_dir, exist_ok=True)
        self.log_path = os.path.join(self.out_dir, "health.log")
        self._fh = open(self.log_path, "w")
        self._fh.write("# health_watch 事件日志 (t_sim, kind, detail)\n")
        self._fh.flush()

        # ---- 状态（必须先于订阅/定时器创建，避免回调在 __init__ 未完成时触发） ----
        self.cur_pos = None       # 最近真值位置
        self.cmd_v = 0.0          # 最近 /cmd_vel 线速度幅值
        self.rtf_sim0 = None      # RTF 窗口起点 (sim, wall)
        self.rtf_wall0 = None
        self.rtf = float("nan")   # 最近一次 RTF 窗口估计
        self.flip_logged = False
        self.stuck_ep_start = None  # "有指令且几乎不动"时段的起点 (sim_ns, pos)
        self._last_hb = 0.0

        self.get_logger().info(
            f"health_watch 启动: out={self.out_dir}, abort={self.abort}, "
            f"flip>{math.degrees(self.flip_rad):.0f}°, "
            f"stuck<{self.stuck_min_m}m/{self.stuck_sec:.0f}s, "
            f"rtf_warn<{self.rtf_warn}")

        self.create_subscription(Odometry, "/gt_odom", self._on_gt, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.timer = self.create_timer(1.0, self._check)

    # ------------------------------------------------------------------
    def _event(self, kind, detail):
        t = self.get_clock().now().nanoseconds * 1e-9
        line = f"{t:.3f} {kind} {detail}"
        self._fh.write(line + "\n")
        self._fh.flush()
        self.get_logger().warn(line)

    # ------------------------------------------------------------------
    def _on_gt(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.cur_pos = (p.x, p.y, p.z)
        roll, pitch = roll_pitch_from_quat(q.x, q.y, q.z, q.w)
        if not self.flip_logged and \
                (abs(roll) > self.flip_rad or abs(pitch) > self.flip_rad):
            self.flip_logged = True
            self._event("FLIP", f"roll={math.degrees(roll):.1f}deg "
                        f"pitch={math.degrees(pitch):.1f}deg")
            if self.abort:
                ab = os.path.join(self.out_dir, "ABORT")
                with open(ab, "w") as f:
                    f.write("flip detected, requested early stop\n")
                self.get_logger().error("翻车已确认, 已写入 ABORT 哨兵请求中止")

    def _on_cmd(self, msg):
        self.cmd_v = abs(msg.linear.x)

    # ------------------------------------------------------------------
    def _check(self):
        if self.cur_pos is None:
            return
        now_sim = self.get_clock().now().nanoseconds * 1e-9
        now_wall = _wall.monotonic()

        # ---- RTF 窗口 ----
        if self.rtf_sim0 is None:
            self.rtf_sim0, self.rtf_wall0 = now_sim, now_wall
        else:
            dsim = now_sim - self.rtf_sim0
            dwall = now_wall - self.rtf_wall0
            if dwall >= self.rtf_window_s and dsim > 0:
                self.rtf = dsim / dwall
                if self.rtf < self.rtf_warn:
                    self._event("RTF_LOW",
                                f"rtf={self.rtf:.3f} (sim {dsim:.1f}s / wall "
                                f"{dwall:.1f}s), 仿真慢于实时")
                self.rtf_sim0, self.rtf_wall0 = now_sim, now_wall

        # ---- 卡死检测：有前进指令但长时间几乎不动 ----
        if self.cmd_v > 0.05:
            if self.stuck_ep_start is None:
                self.stuck_ep_start = (now_sim, self.cur_pos)
            else:
                t0, p0 = self.stuck_ep_start
                if now_sim - t0 >= self.stuck_sec and \
                        math.dist(p0, self.cur_pos) < self.stuck_min_m:
                    self._event("STUCK",
                                f"指令 vx={self.cmd_v:.2f} 但 "
                                f"{self.stuck_sec:.0f}s 位移 "
                                f"{math.dist(p0, self.cur_pos):.3f}m")
                    self.stuck_ep_start = (now_sim, self.cur_pos)  # 避免刷屏
        else:
            self.stuck_ep_start = None

        # ---- 心跳 ----
        if now_wall - self._last_hb >= self.heartbeat_s:
            self._last_hb = now_wall
            self._event("HEARTBEAT", f"rtf={self.rtf:.3f} cmd_v={self.cmd_v:.2f}")

    def stop(self):
        try:
            self._fh.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = HealthWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
