#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lio_evaluation 在线采集节点 (M4)

统一负责 lio_evaluation 的全部落盘:
  - TUM 轨迹: /gt_odom -> groundtruth.txt, /Odometry -> fastlio.txt, 可选 /xxx -> liosam.txt
  - 退化分数: /lio/degeneracy_score -> degen_exact.csv
              /lio/degeneracy_score_proxy -> degen_proxy.csv
  - IEKF 迭代观测: /lio/degeneracy_iterations -> degen_iterations.csv

fast_lio 内部只发布不写盘; 所有 CSV/TUM 都由本节点统一处理,
避免文件 IO 混入核心算法线程的性能测量。
"""

import os
import csv

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from lio_interfaces.msg import DegeneracyScore, DegeneracyIteration
from lio_interfaces.msg import LioHealth, ErrorAttribution


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RecorderNode(Node):
    def __init__(self):
        super().__init__("lio_recorder")
        self.out_dir = self.declare_parameter("output_dir", "").value
        # 话题->文件名 映射 (TUM 轨迹), 逗号分隔: "topic:file,topic:file"
        self.traj_map = self.declare_parameter(
            "traj_map", "/gt_odom:groundtruth.txt,/Odometry:fastlio.txt").value
        self.enable_degen = self.declare_parameter("enable_degen", True).value
        self.enable_iter = self.declare_parameter("enable_iter", True).value

        if not self.out_dir:
            self.get_logger().error("output_dir 为空, 无法记录!")
            raise RuntimeError("output_dir must be set")
        os.makedirs(self.out_dir, exist_ok=True)

        self._tum_handles = {}    # topic -> file handle
        self._degen_handles = {}  # topic -> (file, csv_writer)
        self._iter_handle = None
        self._health_handle = None    # (file, csv_writer) lio_health.csv
        self._attr_handle = None      # (file, csv_writer) error_attribution.csv

        # ---- TUM 轨迹订阅 ----
        for entry in self.traj_map.split(","):
            entry = entry.strip()
            if not entry:
                continue
            if ":" not in entry:
                self.get_logger().warn(f"traj_map 条目格式错误, 跳过: {entry}")
                continue
            topic, fname = entry.split(":", 1)
            fname = fname.strip()
            path = os.path.join(self.out_dir, fname)
            f = open(path, "w")
            self._tum_handles[topic] = f
            self.create_subscription(
                Odometry, topic,
                lambda msg, t=topic: self._on_odom(t, msg),
                10)
            self.get_logger().info(f"记录 TUM: {topic} -> {path}")

        # ---- 退化分数订阅 ----
        if self.enable_degen:
            self._degen_topics = {
                "/lio/degeneracy_score": "degen_exact.csv",
                "/lio/degeneracy_score_proxy": "degen_proxy.csv",
            }
            for topic, fname in self._degen_topics.items():
                path = os.path.join(self.out_dir, fname)
                f = open(path, "w")
                w = csv.writer(f)
                w.writerow([
                    "timestamp", "score", "is_degenerate",
                    "trans_ratio", "rot_ratio", "condition_number",
                    "normal_concentration",
                    "e0", "e1", "e2", "e3", "e4", "e5",
                    "w0", "w1", "w2", "w3", "w4", "w5",
                    "effective_feature_num", "mean_residual",
                    "pos_x", "pos_y", "pos_z",
                ])
                self._degen_handles[topic] = (f, w)
                self.create_subscription(
                    DegeneracyScore, topic,
                    lambda msg, t=topic: self._on_degen(t, msg),
                    10)
                self.get_logger().info(f"记录退化分数: {topic} -> {path}")

        # ---- IEKF 迭代观测订阅 ----
        if self.enable_iter:
            path = os.path.join(self.out_dir, "degen_iterations.csv")
            f = open(path, "w")
            w = csv.writer(f)
            w.writerow([
                "timestamp", "iteration", "frame_index",
                "total_residual", "mean_residual", "effective_feature_num",
                "trans_ratio", "rot_ratio", "lambda_max", "lambda_min",
            ])
            self._iter_handle = (f, w)
            self.create_subscription(
                DegeneracyIteration, "/lio/degeneracy_iterations",
                self._on_iter, 40)
            self.get_logger().info(f"记录 IEKF 迭代: /lio/degeneracy_iterations -> {path}")

        # ---- P2 LIO Health / Error Attribution 订阅 ----
        if self.enable_degen:
            path = os.path.join(self.out_dir, "lio_health.csv")
            f = open(path, "w")
            w = csv.writer(f)
            w.writerow([
                "timestamp", "score", "is_degenerate",
                "trans_ratio", "rot_ratio", "condition_number", "normal_concentration",
                "e0", "e1", "e2", "e3", "e4", "e5",
                "w0", "w1", "w2", "w3", "w4", "w5",
                "weak_mode_index", "dominant_weak_axis",
                "wt0", "wt1", "wt2", "wr0", "wr1", "wr2",
                "effective_feature_num", "candidate_feature_num",
                "effective_feature_ratio", "mean_residual", "residual_p90",
                "total_residual",
                "pose_cov0", "pose_cov1", "pose_cov2", "pose_cov3", "pose_cov4", "pose_cov5",
                "pos_cov_trace", "rot_cov_trace",
                "bg_cov0", "bg_cov1", "bg_cov2", "ba_cov0", "ba_cov1", "ba_cov2",
                "bg_cov_trace", "ba_cov_trace",
                "imu_sample_num",
                "gyr_rms0", "gyr_rms1", "gyr_rms2",
                "gyr_var0", "gyr_var1", "gyr_var2",
                "acc_var0", "acc_var1", "acc_var2",
                "imu_gyr_excitation", "imu_acc_excitation",
                # P3 adaptive (detected scales 本帧; lidar_meas_cov 是 applied, 滞后一帧)
                "lidar_meas_cov", "geometry_scale", "matching_scale",
                "final_scale", "adaptive_active",
                "pos_x", "pos_y", "pos_z",
            ])
            self._health_handle = (f, w)
            self.create_subscription(
                LioHealth, "/lio/health", self._on_health, 20)
            self.get_logger().info(f"记录 LIO Health: /lio/health -> {path}")

            path = os.path.join(self.out_dir, "error_attribution.csv")
            f = open(path, "w")
            w = csv.writer(f)
            w.writerow([
                "timestamp", "label", "label_str", "margin",
                "severity_normal", "severity_geometry",
                "severity_correspondence", "severity_imu_weak",
                "is_geometry_degen", "is_matching_failure", "is_imu_low_excitation",
                "geom_risk_score", "imu_risk_score",
                "trans_ratio", "rot_ratio", "score",
                "effective_feature_num", "effective_feature_ratio",
                "mean_residual", "residual_p90",
                "imu_gyr_excitation", "imu_acc_excitation",
                "pos_cov_trace",
                "wt0", "wt1", "wt2", "weak_mode_index", "dominant_weak_axis",
            ])
            self._attr_handle = (f, w)
            self.create_subscription(
                ErrorAttribution, "/lio/error_attribution", self._on_attribution, 20)
            self.get_logger().info(
                f"记录 Error Attribution: /lio/error_attribution -> {path}")

        self.get_logger().info("lio_recorder started")

    def _on_odom(self, topic, msg):
        f = self._tum_handles.get(topic)
        if f is None:
            return
        t = stamp_to_sec(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        f.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        f.flush()

    def _on_degen(self, topic, msg):
        entry = self._degen_handles.get(topic)
        if entry is None:
            return
        _, w = entry
        t = stamp_to_sec(msg.header.stamp)
        w.writerow([
            f"{t:.9f}", f"{msg.score:.6f}", int(msg.is_degenerate),
            f"{msg.trans_ratio:.6f}", f"{msg.rot_ratio:.6f}", f"{msg.condition_number:.3f}",
            f"{msg.normal_concentration:.6f}",
            *[f"{v:.6e}" for v in msg.eigenvalues],
            *[f"{v:.6f}" for v in msg.weak_direction],
            msg.effective_feature_num, f"{msg.mean_residual:.6f}",
            f"{msg.pos_x:.6f}", f"{msg.pos_y:.6f}", f"{msg.pos_z:.6f}",
        ])
        self._degen_handles[topic][0].flush()

    def _on_iter(self, msg):
        if self._iter_handle is None:
            return
        f, w = self._iter_handle
        t = stamp_to_sec(msg.header.stamp)
        w.writerow([
            f"{t:.9f}", msg.iteration, msg.frame_index,
            f"{msg.total_residual:.6f}", f"{msg.mean_residual:.6f}", msg.effective_feature_num,
            f"{msg.trans_ratio:.6f}", f"{msg.rot_ratio:.6f}",
            f"{msg.lambda_max:.6e}", f"{msg.lambda_min:.6e}",
        ])
        f.flush()

    def _on_health(self, msg):
        if self._health_handle is None:
            return
        f, w = self._health_handle
        t = stamp_to_sec(msg.header.stamp)
        w.writerow([
            f"{t:.9f}", f"{msg.score:.6f}", int(msg.is_degenerate),
            f"{msg.trans_ratio:.6f}", f"{msg.rot_ratio:.6f}",
            f"{msg.condition_number:.3f}", f"{msg.normal_concentration:.6f}",
            *[f"{v:.6e}" for v in msg.eigenvalues],
            *[f"{v:.6f}" for v in msg.weak_direction],
            msg.weak_mode_index, msg.dominant_weak_axis,
            *[f"{v:.6f}" for v in msg.weak_translation_direction],
            *[f"{v:.6f}" for v in msg.weak_rotation_direction],
            msg.effective_feature_num, msg.candidate_feature_num,
            f"{msg.effective_feature_ratio:.6f}",
            f"{msg.mean_residual:.6f}", f"{msg.residual_p90:.6f}",
            f"{msg.total_residual:.6f}",
            *[f"{v:.6e}" for v in msg.pose_cov_diag],
            f"{msg.pos_cov_trace:.6e}", f"{msg.rot_cov_trace:.6e}",
            *[f"{v:.6e}" for v in msg.bg_cov_diag],
            *[f"{v:.6e}" for v in msg.ba_cov_diag],
            f"{msg.bg_cov_trace:.6e}", f"{msg.ba_cov_trace:.6e}",
            msg.imu_sample_num,
            *[f"{v:.6e}" for v in msg.imu_gyr_rms],
            *[f"{v:.6e}" for v in msg.imu_gyr_var],
            *[f"{v:.6e}" for v in msg.imu_acc_var],
            f"{msg.imu_gyr_excitation:.6e}", f"{msg.imu_acc_excitation:.6e}",
            f"{msg.lidar_meas_cov:.6e}", f"{msg.geometry_scale:.6f}",
            f"{msg.matching_scale:.6f}", f"{msg.final_scale:.6f}",
            int(msg.adaptive_active),
            f"{msg.pos_x:.6f}", f"{msg.pos_y:.6f}", f"{msg.pos_z:.6f}",
        ])
        f.flush()

    def _on_attribution(self, msg):
        if self._attr_handle is None:
            return
        f, w = self._attr_handle
        t = stamp_to_sec(msg.header.stamp)
        w.writerow([
            f"{t:.9f}", msg.label, msg.label_str, f"{msg.margin:.6f}",
            f"{msg.severity_normal:.6f}", f"{msg.severity_geometry:.6f}",
            f"{msg.severity_correspondence:.6f}", f"{msg.severity_imu_weak:.6f}",
            int(msg.is_geometry_degen), int(msg.is_matching_failure),
            int(msg.is_imu_low_excitation),
            f"{msg.geom_risk_score:.6f}", f"{msg.imu_risk_score:.6f}",
            f"{msg.trans_ratio:.6f}", f"{msg.rot_ratio:.6f}", f"{msg.score:.6f}",
            msg.effective_feature_num, f"{msg.effective_feature_ratio:.6f}",
            f"{msg.mean_residual:.6f}", f"{msg.residual_p90:.6f}",
            f"{msg.imu_gyr_excitation:.6e}", f"{msg.imu_acc_excitation:.6e}",
            f"{msg.pos_cov_trace:.6e}",
            *[f"{v:.6f}" for v in msg.weak_translation_direction],
            msg.weak_mode_index, msg.dominant_weak_axis,
        ])
        f.flush()

    def shutdown(self):
        for f in self._tum_handles.values():
            f.close()
        for f, _ in self._degen_handles.values():
            f.close()
        if self._iter_handle:
            self._iter_handle[0].close()
        if self._health_handle:
            self._health_handle[0].close()
        if self._attr_handle:
            self._attr_handle[0].close()
        self.get_logger().info("lio_recorder closed all files")


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

