#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 online Error Attribution node (rule-based)。

订阅 /lio/health (LioHealth) → attribution_rules.classify → 发布 /lio/error_attribution (ErrorAttribution)。
阈值表: config/attribution_params.yaml (参数 params_file), 与离线 --tune 共用同一规则模块。

用法:
  python3 attribution_node.py --ros-args -p params_file:=... -p use_sim_time:=true
"""
import os
import sys
import collections
import statistics

import yaml
import rclpy
from rclpy.node import Node
from lio_interfaces.msg import LioHealth, ErrorAttribution

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from attribution_rules import classify, snapshot_from_health, LABELS  # noqa: E402


def load_cfg(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return (data.get("attribution") or {})


class ErrorAttributionNode(Node):
    def __init__(self):
        super().__init__("error_attribution")
        params_file = self.declare_parameter(
            "params_file",
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "config", "attribution_params.yaml")).value
        if not os.path.isabs(params_file):
            params_file = os.path.abspath(params_file)
        self.cfg = load_cfg(params_file) if os.path.exists(params_file) else {}
        self.get_logger().info(f"attribution 阈值表: {params_file} "
                               f"({len(self.cfg)} 项)")
        # 滚动基线: (时间戳, effective_feature_ratio, effective_feature_num) 用于相对骤降判定
        self._ratio_hist = collections.deque()
        self._base_window_s = float(self.cfg.get("ratio_baseline_window_s", 10.0))

        self.sub = self.create_subscription(
            LioHealth, "/lio/health", self._on_health, 20)
        self.pub = self.create_publisher(
            ErrorAttribution, "/lio/error_attribution", 20)
        self.get_logger().info("error_attribution_node started: "
                               "/lio/health -> /lio/error_attribution")

    def _on_health(self, msg):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # 维护滚动 ratio/num 基线 (近 ratio_baseline_window_s 秒中位数)
        self._ratio_hist.append((t_sec, float(msg.effective_feature_ratio),
                                 float(msg.effective_feature_num)))
        while self._ratio_hist and t_sec - self._ratio_hist[0][0] > self._base_window_s:
            self._ratio_hist.popleft()
        baseline = None
        num_baseline = None
        if len(self._ratio_hist) >= 5:
            baseline = statistics.median([r for _, r, _ in self._ratio_hist])
            num_baseline = statistics.median([n for _, _, n in self._ratio_hist])
        context = {"ratio_baseline": baseline, "num_baseline": num_baseline}

        h = {
            "score": msg.score,
            "trans_ratio": msg.trans_ratio,
            "rot_ratio": msg.rot_ratio,
            "effective_feature_num": msg.effective_feature_num,
            "candidate_feature_num": msg.candidate_feature_num,
            "effective_feature_ratio": msg.effective_feature_ratio,
            "mean_residual": msg.mean_residual,
            "residual_p90": msg.residual_p90,
            "imu_gyr_excitation": msg.imu_gyr_excitation,
            "imu_acc_excitation": msg.imu_acc_excitation,
            "pos_cov_trace": msg.pos_cov_trace,
            "weak_translation_direction": list(msg.weak_translation_direction),
            "weak_mode_index": msg.weak_mode_index,
            "dominant_weak_axis": msg.dominant_weak_axis,
        }
        r = classify(h, self.cfg, context)

        out = ErrorAttribution()
        out.header = msg.header
        out.label = r["label"]
        out.label_str = r["label_str"]
        out.margin = float(r["margin"])
        out.severity_normal = float(r["severity_normal"])
        out.severity_geometry = float(r["severity_geometry"])
        out.severity_correspondence = float(r["severity_correspondence"])
        out.severity_imu_weak = float(r["severity_imu_weak"])
        out.is_geometry_degen = bool(r["is_geometry_degen"])
        out.is_matching_failure = bool(r["is_matching_failure"])
        out.is_imu_low_excitation = bool(r["is_imu_low_excitation"])
        out.geom_risk_score = float(r["geom_risk_score"])
        out.imu_risk_score = float(r["imu_risk_score"])
        out.trans_ratio = msg.trans_ratio
        out.rot_ratio = msg.rot_ratio
        out.score = msg.score
        out.effective_feature_num = msg.effective_feature_num
        out.effective_feature_ratio = msg.effective_feature_ratio
        out.mean_residual = msg.mean_residual
        out.residual_p90 = msg.residual_p90
        out.imu_gyr_excitation = msg.imu_gyr_excitation
        out.imu_acc_excitation = msg.imu_acc_excitation
        out.pos_cov_trace = msg.pos_cov_trace
        for i in range(3):
            out.weak_translation_direction[i] = msg.weak_translation_direction[i]
        out.weak_mode_index = msg.weak_mode_index
        out.dominant_weak_axis = msg.dominant_weak_axis
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ErrorAttributionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
