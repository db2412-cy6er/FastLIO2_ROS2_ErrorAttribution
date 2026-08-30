#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 冻结: 跨实验汇总 (替代手工整理)。

遍历 data/experiments/experiment_*/ 或 --exps 指定列表, 读取 result.yaml +
attribution_report.yaml, 统一生成:
  scenario / ATE(rmse,mean,max) / RPE / 主标签分布 / 多标签 flag 命中率 /
  per-class precision·recall / CPU·mem。

输出: 表格 (stdout) + data/experiments/aggregate_summary.yaml。

用法:
  python3 aggregate_experiments.py [--exps 027,029,031] [--out .../aggregate_summary.yaml]
"""
import os
import sys
import glob
import argparse

import yaml

EXP_BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "data", "experiments")
DEFAULT_OUT = os.path.join(EXP_BASE, "aggregate_summary.yaml")

LABELS = ["NORMAL", "GEOMETRY_DEGENERATION",
          "CORRESPONDENCE_FAILURE", "IMU_LOW_EXCITATION"]
FLAGS = ["is_geometry_degen", "is_matching_failure", "is_imu_low_excitation"]


def load_yaml(path):
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def summarize_exp(exp_dir):
    """返回该实验的汇总 dict。"""
    rid = os.path.basename(exp_dir)
    res = load_yaml(os.path.join(exp_dir, "result.yaml"))
    attr = load_yaml(os.path.join(exp_dir, "attribution_report.yaml"))
    meta = load_yaml(os.path.join(exp_dir, "config.yaml"))
    # config.yaml 是文本 + yaml 混合, 只取 resolved 段
    resolved = meta.get("resolved", {}) if isinstance(meta, dict) else {}

    ate = res.get("ate", {}) or {}
    rpe = res.get("rpe", {}) or {}
    rt = res.get("runtime", {}) or {}
    ld = attr.get("label_distribution", {}) or {}
    fec = attr.get("flag_error_condition", {}) or {}
    cls = (attr.get("classification") or {}).get("per_class", {}) or {}

    row = {
        "experiment": rid,
        "scenario": (meta.get("scenario") if isinstance(meta, dict) else None) or "",
        "ate_rmse_m": round(ate.get("rmse_m", float("nan")), 4),
        "ate_mean_m": round(ate.get("mean_m", float("nan")), 4),
        "ate_max_m": round(ate.get("max_m", float("nan")), 4),
        "rpe_rmse_m": round(rpe.get("rmse_m", float("nan")), 4),
        "n_frames": attr.get("n_frames", 0),
        "label_ratio": {k: round(ld.get(k, 0.0), 3) for k in LABELS},
        "flag_ratio": {},
        "classification": {},
        "drive_mode": (resolved.get("drive") or {}).get("mode") if isinstance(resolved, dict) else None,
        "cpu_percent": rt.get("cpu_percent"),
        "rss_mb": rt.get("rss_mb"),
    }
    for fl in FLAGS:
        c = fec.get(fl) or {}
        row["flag_ratio"][fl] = round(c.get("n_ratio", 0.0), 3)
    for cname, v in cls.items():
        row["classification"][cname] = {
            "precision": round(v.get("precision", 0.0), 3),
            "recall": round(v.get("recall", 0.0), 3),
            "n_true": v.get("n_true", 0),
            "n_pred": v.get("n_pred", 0),
        }
    return row


def main():
    p = argparse.ArgumentParser(description="跨实验汇总")
    p.add_argument("--exps", default=None,
                   help="逗号分隔实验号 (如 027,029,031); 默认遍历全部")
    p.add_argument("--out", default=DEFAULT_OUT)
    a = p.parse_args()

    if a.exps:
        dirs = [os.path.join(EXP_BASE, f"experiment_{e.strip():0>3}")
                for e in a.exps.split(",") if e.strip()]
    else:
        dirs = sorted(glob.glob(os.path.join(EXP_BASE, "experiment_*")))
    dirs = [d for d in dirs if os.path.isdir(d)]

    rows = [summarize_exp(d) for d in dirs]
    rows.sort(key=lambda r: r["experiment"])

    # stdout 表格
    hdr = f"{'exp':<6}{'场景':<12}{'ATErmse':>9}{'ATEmax':>9}{'RPErms':>9}" \
          f"{'NORMAL':>8}{'GEOM':>8}{'CORR':>8}{'imu':>6}  per-class P/R"
    print(hdr)
    print("-" * 100)
    for r in rows:
        cls = " ".join(f"{k}:{v['precision']}/{v['recall']}"
                       for k, v in r["classification"].items()) or "-"
        print(f"{r['experiment'][-3:]:<6}{str(r['scenario'])[:10]:<12}"
              f"{r['ate_rmse_m']:>9.4f}{r['ate_max_m']:>9.4f}{r['rpe_rmse_m']:>9.4f}"
              f"{r['label_ratio']['NORMAL']:>8.2f}{r['label_ratio']['GEOMETRY_DEGENERATION']:>8.2f}"
              f"{r['label_ratio']['CORRESPONDENCE_FAILURE']:>8.2f}"
              f"{r['flag_ratio']['is_imu_low_excitation']:>6.2f}  {cls}")

    with open(a.out, "w") as f:
        yaml.dump({"generated_at": __import__("time").strftime("%Y-%m-%d %H:%M:%S"),
                   "experiments": rows}, f, default_flow_style=False,
                  allow_unicode=True, sort_keys=False)
    print(f"\n已写入: {a.out} ({len(rows)} 个实验)")


if __name__ == "__main__":
    main()
