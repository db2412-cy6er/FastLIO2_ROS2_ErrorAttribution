#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P3 parity check: FAST-LIO C++ adaptive helper vs Python attribution_rules 一致性回归。

背景 (P3 评审 R13/R14):
  FAST-LIO 内嵌的 adaptive controller 用 C++ 复刻了 P2 attribution_rules.py 的
  matching severity 与 geometry severity 公式及冻结阈值, 分别作为 aggressive/mild
  两个降权 channel 的检测信号。本脚本用同一份 lio_health.csv 离线逐帧跑 Python
  attribution_rules, 与 C++ 发布的 geometry_scale / matching_scale 对比 (scale =
  1 + (max-1)*severity), 防两套规则在后续演进中分叉。

用法:
  python3 parity_check.py <experiment_dir>
  [--params config/attribution_params.yaml]   # Python 侧阈值表 (默认冻结 v1.1)
  参数 max_geom/max_match 自动从 <exp_dir>/config.yaml 的 resolved.adaptive 读取。

输出: 逐帧对比统计 + PASS/FAIL 判定。
"""
import argparse
import collections
import csv
import os
import statistics
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from attribution_rules import classify  # noqa: E402


def load_health_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            d = {}
            for k, v in r.items():
                try:
                    d[k] = float(v)
                except (ValueError, TypeError):
                    d[k] = v
            rows.append(d)
    return rows


def resolve_adaptive_params(exp_dir):
    """从 <exp_dir>/config.yaml 的 resolved.adaptive 读取 max scales (launch 唯一参数源)。"""
    max_geom = 2.0
    max_match = 50.0
    cfg_path = os.path.join(exp_dir, "config.yaml")
    if os.path.exists(cfg_path):
        try:
            with open(cfg_path) as f:
                text = f.read()
            idx = text.find("resolved:")
            if idx >= 0:
                data = yaml.safe_load(text[idx:]) or {}
                resolved = data.get("resolved", {})
                ad = resolved.get("adaptive", {})
                if ad.get("max_geom_scale"):
                    max_geom = float(ad["max_geom_scale"])
                if ad.get("max_match_scale"):
                    max_match = float(ad["max_match_scale"])
        except Exception as e:  # noqa: BLE001
            print(f"[parity] 读取 config.yaml 失败, 用 launch 默认参数: {e}")
    return max_geom, max_match


def main():
    ap = argparse.ArgumentParser(description="P3 parity check (C++ helper vs Python rules)")
    ap.add_argument("exp_dir")
    ap.add_argument("--params", default=None,
                    help="attribution_params.yaml (Python 侧阈值表)")
    args = ap.parse_args()

    exp_dir = os.path.abspath(args.exp_dir)
    health_csv = os.path.join(exp_dir, "lio_health.csv")
    if not os.path.exists(health_csv):
        print(f"[parity] 找不到 lio_health.csv: {health_csv}")
        sys.exit(1)

    params_path = args.params or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "config", "attribution_params.yaml")
    with open(params_path) as f:
        cfg = (yaml.safe_load(f) or {}).get("attribution", {})
    max_geom, max_match = resolve_adaptive_params(exp_dir)
    print(f"[parity] exp={os.path.basename(exp_dir)}  "
          f"max_geom={max_geom}  max_match={max_match}")

    rows = load_health_csv(health_csv)
    if not rows:
        print("[parity] lio_health.csv 为空")
        sys.exit(1)

    base_window_s = float(cfg.get("ratio_baseline_window_s", 30.0))
    hist = collections.deque()
    n = 0
    n_active = 0
    max_g_err = 0.0
    max_m_err = 0.0
    sum_g_err = 0.0
    sum_m_err = 0.0
    n_geom_flag_diff = 0   # C++ adaptive_active vs Python (severity>0) 不一致帧
    n_match_flag_diff = 0

    for r in rows:
        t = r["timestamp"]
        hist.append((t, r["effective_feature_ratio"], r["effective_feature_num"]))
        while hist and t - hist[0][0] > base_window_s:
            hist.popleft()
        context = {}
        if len(hist) >= 5:
            context = {
                "ratio_baseline": statistics.median(x[1] for x in hist),
                "num_baseline": statistics.median(x[2] for x in hist),
            }
        h = {
            "score": r.get("score", 0.0),
            "trans_ratio": r.get("trans_ratio", 1.0),
            "rot_ratio": r.get("rot_ratio", 1.0),
            "effective_feature_num": r.get("effective_feature_num", 0.0),
            "candidate_feature_num": r.get("candidate_feature_num", 0.0),
            "effective_feature_ratio": r.get("effective_feature_ratio", 1.0),
            "mean_residual": r.get("mean_residual", 0.0),
            "residual_p90": r.get("residual_p90", r.get("mean_residual", 0.0)),
        }
        res = classify(h, cfg, context)

        py_geom = 1.0 + (max_geom - 1.0) * res["severity_geometry"]
        py_match = 1.0 + (max_match - 1.0) * res["severity_correspondence"]

        cpp_geom = r.get("geometry_scale", 1.0)
        cpp_match = r.get("matching_scale", 1.0)
        if cpp_geom <= 0.0 or cpp_match <= 0.0:
            continue  # adaptive 未启用时 C++ 字段为默认 1.0, 跳过 (无检测信号)

        g_err = abs(cpp_geom - py_geom)
        m_err = abs(cpp_match - py_match)
        max_g_err = max(max_g_err, g_err)
        max_m_err = max(max_m_err, m_err)
        sum_g_err += g_err
        sum_m_err += m_err
        n += 1
        if res["severity_geometry"] > 0.0 or res["severity_correspondence"] > 0.0:
            n_active += 1
        # 活性一致性 (含 1e-6 死区)
        if (cpp_geom > 1.0 + 1e-6) != (res["severity_geometry"] > 0.0):
            n_geom_flag_diff += 1
        if (cpp_match > 1.0 + 1e-6) != (res["severity_correspondence"] > 0.0):
            n_match_flag_diff += 1

    if n == 0:
        print("[parity] 无有效帧 (adaptive 可能未启用, C++ 字段为默认 1.0) —— "
              "请对 --adaptive 启用的实验运行")
        sys.exit(2)

    tol = 1e-4
    ok = (max_g_err <= tol and max_m_err <= tol
          and n_geom_flag_diff == 0 and n_match_flag_diff == 0)
    print(f"[parity] 对比帧数: {n}  (Python 侧 active 帧: {n_active})")
    print(f"[parity] geometry_scale   mean|err|={sum_g_err / n:.2e}  "
          f"max|err|={max_g_err:.2e}  活性不一致={n_geom_flag_diff}")
    print(f"[parity] matching_scale   mean|err|={sum_m_err / n:.2e}  "
          f"max|err|={max_m_err:.2e}  活性不一致={n_match_flag_diff}")
    print(f"[parity] 判定: {'PASS' if ok else 'FAIL'} (tolerance={tol})")
    if not ok:
        print("[parity] 存在 C++/Python 不一致帧 → 检查阈值或规则是否分叉")
        sys.exit(1)


if __name__ == "__main__":
    main()
