#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""脚本化运动驱动 + 反应式避障 — 让在线实验中机器人安全移动。

脚本模式（漫游 wander，默认）:
  前进 3.2m (0.4 m/s × 8s) → 原地转 30° → 前进 3.2m → 转 30° …
  每次转向只偏 30°，航向逐渐变化，让小车在实验期覆盖更多新区域
  （而非直线往返在同一路径上折返）。
  （距离/角度由指令积分得到，避免 RTF/斜坡导致的计时漂移。）

反应式避障层（优先级高于脚本模式）:
  订阅 /livox/lidar (sensor_msgs/PointCloud2, 雷达坐标系 livox_frame)。
  前向锥检测：x>0、|atan2(y,x)|<=wedge_deg、高度 z∈[height_min,height_max]
  （相对雷达，避开地面与天花板）。
  - FORWARD 期间锥内最近水平距离 < stop_dist → 进入 AVOID_TURN：
    **步进式小转角**，每转 avoid_turn_rad(默认30°) 后重新评估一次，
    前方 >= clear_dist(或转完一步后 >= stop_dist) 即恢复 FORWARD；
    仍被挡则再转一小步 —— 避免在原地大角度旋转"转来转去"；
  - TURN（脚本 30° 转向）期间本身就在原地旋转，锥内障碍不打断（旋转即避让）；
  - 激光数据超时(lidar_timeout) → 失效安全停车（防盲跑撞墙）。

驱动平滑：/cmd_vel 按 cmd_rate(20Hz) 发布 + 线性加减速斜坡(accel/ang_accel)，
避免指令突变在软渲染下被放大成"一顿一顿"。

用法: python3 drive_node.py [--ros-args -p use_sim_time:=true -p stop_dist:=0.8 ...]
发布: /cmd_vel (geometry_msgs/Twist, 20Hz)
"""

import math
import time as _wall

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def front_min_distance(xyz, wedge_deg, height_min, height_max):
    """前向锥内最近水平距离(m)。xyz: (N,3) 本体系点云(x前,y左,z上)。

    返回 float：无有效点 → math.inf。
    判定条件：x>0（前方）、|atan2(y,x)|<=wedge_deg（水平锥角）、
    height_min<=z<=height_max（高度带，剔除地面/天花板）。
    """
    if xyz is None or len(xyz) == 0:
        return math.inf
    xyz = np.asarray(xyz, dtype=np.float64)
    ok = np.isfinite(xyz).all(axis=1) & (xyz[:, 0] > 0.0)
    if not ok.any():
        return math.inf
    pts = xyz[ok]
    ang = np.abs(np.arctan2(pts[:, 1], pts[:, 0]))
    mask = (ang <= math.radians(wedge_deg)) & \
        (pts[:, 2] >= height_min) & (pts[:, 2] <= height_max)
    if not mask.any():
        return math.inf
    return float(np.hypot(pts[mask, 0], pts[mask, 1]).min())



class SafeScriptedDrive(Node):
    def __init__(self):
        super().__init__("scripted_drive")
        # ---- 脚本漫游参数 ----
        self.forward_speed = self.declare_parameter("forward_speed", 0.4).value
        self.turn_speed = self.declare_parameter("turn_speed", 0.5).value
        self.forward_sec = self.declare_parameter("forward_sec", 8.0).value
        self.turn_sec = self.declare_parameter("turn_sec", 3.14).value
        # P2: 驱动模式 wander(漫游) / stop_go(前进-静止) / const_vel(低速近匀速直行)
        self.drive_mode = self.declare_parameter(
            "drive_mode", "wander").value
        self.pause_sec = self.declare_parameter(
            "pause_sec", 4.0).value  # stop_go 静止时长 (s)
        # 腿长由速度×时长折算（默认 0.4×8=3.2m）
        self.leg_m = self.declare_parameter(
            "leg_m", self.forward_speed * self.forward_sec).value
        # 自动转向角：30°（小角度逐次偏航，覆盖更多新区域；可用 turn_rad:=0.785 调 45°）
        self.turn_rad = self.declare_parameter("turn_rad", math.pi / 6).value
        # ---- 避障参数 ----
        self.stop_dist = self.declare_parameter("stop_dist", 0.8).value
        self.clear_dist = self.declare_parameter("clear_dist", 1.2).value
        self.wedge_deg = self.declare_parameter("wedge_deg", 60.0).value
        self.height_min = self.declare_parameter("height_min", -0.15).value
        self.height_max = self.declare_parameter("height_max", 0.5).value
        self.lidar_timeout = self.declare_parameter("lidar_timeout", 1.0).value
        # 避障步进转角：每次只转 30°，重新评估后再决定是否继续（避免原地打转）
        self.avoid_turn_rad = self.declare_parameter(
            "avoid_turn_rad", math.pi / 6).value
        # ---- 平滑参数 ----
        self.cmd_rate = self.declare_parameter("cmd_rate", 20.0).value
        self.accel = self.declare_parameter("accel", 0.25).value          # m/s^2
        self.ang_accel = self.declare_parameter("ang_accel", 0.25).value  # rad/s^2

        if self.clear_dist <= self.stop_dist:
            self.clear_dist = self.stop_dist * 1.5
            self.get_logger().warn(
                f"clear_dist<=stop_dist, 已自动修正为 {self.clear_dist:.2f}m")

        # ---- 话题/定时器 ----
        # 激光通常为 best-effort（SensorDataQoS），用 best-effort 订阅以兼容
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---- 状态（必须先于订阅/定时器创建，避免回调在 __init__ 未完成时触发） ----
        self.mode = "forward"      # forward / turn / paused（脚本漫游 & stop_go 子状态）
        self.pause_start = None    # stop_go 静止段起始仿真时刻
        self.avoiding = False      # 避障旋转中（优先级覆盖脚本）
        self.avoid_accum = 0.0     # 当前避障步累计转角（每 avoid_turn_rad 评估一次）
        self.leg_accum = 0.0       # 当前脚本腿累计距离/角度（避障期间暂停）
        self.last_t = None
        self.cur_vx = 0.0          # 当前发布线速度（斜坡平滑用）
        self.cur_wz = 0.0          # 当前发布角速度
        self.min_front = math.inf  # 前向锥最近水平距离
        self.cloud_t = None        # 最近点云到达墙钟时刻（超时判断用）
        self._stale_log_t = 0.0
        self._avoid_log_t = 0.0

        self.cloud_sub = self.create_subscription(
            PointCloud2, "/livox/lidar", self._on_cloud,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(1.0 / self.cmd_rate, self._tick)

        self.get_logger().info(
            f"drive_node 启动: mode={self.drive_mode}"
            f"(v={self.forward_speed:.2f}m/s, leg={self.leg_m:.2f}m, "
            f"turn={math.degrees(self.turn_rad):.0f}°, pause={self.pause_sec:.0f}s) "
            f"+ 避障(stop={self.stop_dist:.2f}m, clear={self.clear_dist:.2f}m, "
            f"wedge={self.wedge_deg:.0f}°, "
            f"step={math.degrees(self.avoid_turn_rad):.0f}°)")

    def _on_cloud(self, msg):
        self.cloud_t = _wall.monotonic()
        try:
            xyz = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
        except Exception as e:  # 解析异常时按"无有效数据"处理，避免误触发
            self.get_logger().warn(f"点云解析失败: {e}")
            return
        self.min_front = front_min_distance(
            xyz, self.wedge_deg, self.height_min, self.height_max)

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        if self.last_t is None:
            self.last_t = now
            self._ramped_publish(0.0, 0.0, 1.0 / self.cmd_rate)
            return
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0 or dt > 5.0:  # 时钟回跳/跳变保护
            dt = 1.0 / self.cmd_rate
        dt = min(dt, 1.0)

        # ---- 失效安全：激光数据超时 → 停车（防盲跑） ----
        if self.cloud_t is not None and \
                (_wall.monotonic() - self.cloud_t) > self.lidar_timeout:
            if _wall.monotonic() - self._stale_log_t > 5.0:
                self.get_logger().warn(
                    f"激光数据超时 {self.lidar_timeout}s, 失效安全停车 "
                    f"(mode={self.mode})")
                self._stale_log_t = _wall.monotonic()
            self._ramped_publish(0.0, 0.0, dt)
            return

        # ---- 脚本阶段推进（距离/角度积分；避障期间暂停） ----
        if not self.avoiding:
            if self.drive_mode == "const_vel":
                # 低速近匀速直行: 不转向 (P2 constant_velocity_corridor 实验)
                pass
            elif self.drive_mode == "stop_go":
                if self.mode == "forward":
                    self.leg_accum += self.cur_vx * dt
                    if self.leg_accum >= self.leg_m:
                        self.mode = "paused"
                        self.pause_start = now
                        self.leg_accum = 0.0
                elif self.mode == "paused":
                    if (now - self.pause_start).nanoseconds * 1e-9 >= self.pause_sec:
                        self.mode = "forward"
                else:  # turn (stop_go 同样支持转向腿, 防撞墙)
                    self.leg_accum += abs(self.cur_wz) * dt
                    if self.leg_accum >= self.turn_rad:
                        self.mode = "forward"
                        self.leg_accum = 0.0
            else:  # wander
                if self.mode == "forward":
                    self.leg_accum += self.cur_vx * dt
                    if self.leg_accum >= self.leg_m:
                        self.mode = "turn"
                        self.leg_accum = 0.0
                else:
                    self.leg_accum += abs(self.cur_wz) * dt
                    if self.leg_accum >= self.turn_rad:
                        self.mode = "forward"
                        self.leg_accum = 0.0

        # ---- 避障决策（FORWARD 期间前向锥见障 → 步进式 AVOID_TURN） ----
        # 每次只转 avoid_turn_rad(默认30°) 后重新评估：前方足够空间则恢复前进，
        # 否则再转一小步 —— 避免在原地大角度旋转"转来转去"。
        if self.mode == "forward":
            if self.avoiding:
                self.avoid_accum += abs(self.cur_wz) * dt
                if self.min_front >= self.clear_dist:
                    self.avoiding = False
                    self.avoid_accum = 0.0
                    self.get_logger().info(
                        f"前方已清空 {self.min_front:.2f}m >= {self.clear_dist}m "
                        f"→ 恢复 FORWARD")
                elif self.avoid_accum >= self.avoid_turn_rad:
                    if self.min_front >= self.stop_dist:
                        self.avoiding = False
                        self.avoid_accum = 0.0
                        self.get_logger().info(
                            f"转完一步{math.degrees(self.avoid_turn_rad):.0f}°, "
                            f"前方 {self.min_front:.2f}m >= {self.stop_dist}m "
                            f"→ 恢复 FORWARD")
                    else:
                        self.avoid_accum = 0.0  # 仍被挡, 继续下一小步
                        self.get_logger().warn(
                            f"转完一步仍被挡(前向 {self.min_front:.2f}m), "
                            f"继续下一小步")
            elif self.min_front < self.stop_dist:
                self.avoiding = True
                self.avoid_accum = 0.0
                self.get_logger().warn(
                    f"前方障碍 {self.min_front:.2f}m < {self.stop_dist}m "
                    f"→ AVOID_TURN")
        if self.avoiding and _wall.monotonic() - self._avoid_log_t > 5.0:
            self.get_logger().warn(
                f"AVOID_TURN 步进旋转中, 前向锥最近 {self.min_front:.2f}m")
            self._avoid_log_t = _wall.monotonic()

        # ---- 期望速度（避障优先级最高） ----
        if self.avoiding:
            target_x, target_z = 0.0, self.turn_speed
        elif self.mode == "forward":
            target_x, target_z = self.forward_speed, 0.0
        elif self.mode == "paused":   # stop_go 静止段
            target_x, target_z = 0.0, 0.0
        else:  # scripted turn：原地旋转本身即避让
            target_x, target_z = 0.0, self.turn_speed

        self._ramped_publish(target_x, target_z, dt)

    # ------------------------------------------------------------------
    # 平滑发布
    # ------------------------------------------------------------------
    def _ramped_publish(self, target_x, target_z, dt):
        dvx = max(-self.accel * dt, min(self.accel * dt, target_x - self.cur_vx))
        self.cur_vx += dvx
        dwz = max(-self.ang_accel * dt, min(self.ang_accel * dt, target_z - self.cur_wz))
        self.cur_wz += dwz
        msg = Twist()
        msg.linear.x = self.cur_vx
        msg.angular.z = self.cur_wz
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SafeScriptedDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT 时 rclpy 已内部 shutdown，此处二次调用会抛 "rcl_shutdown already called"
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

