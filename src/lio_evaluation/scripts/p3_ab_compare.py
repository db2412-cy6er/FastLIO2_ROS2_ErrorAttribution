#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P3 A/B 严格对比聚合 (评审 R23/R26: median of ≥3 + 多维指标)。

用法:
  python3 p3_ab_compare.py --name corridor \
      --degen exp_031 exp_045 ...          # degen-only 实验目录 (建议 ≥3)
      --adaptive exp_053 exp_0XX ...       # adaptive 实验目录 (建议 ≥3)
      [--fault]                            # 计算 fault 段 window error growth / recovery

指标 (每个实验 + 每组聚合):
  ATE rmse/mean/max / RPE rmse / final drift (对齐后终点误差) / max error
  --fault: 每个 correspondence_failure 标注段内 error growth + window max error
           + 段后 recovery time (回落至 fault 前基线 + 0.05m 的时间)。
判定 (median of ≥3):
  adaptive median < degen median 且差 > 运行噪声 spread (degen runs 的 max-min),
  才算"显著改进"; 改进量 ≤ 噪声 → "不显著" (如实报告)。
"""
import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from attribution_validation import compute_aligned_error  # noqa: E402


def load_result(exp_dir):
    with open(os.path.join(exp_dir, "result.yaml")) as f:
        return yaml.safe_load(f) or {}


def load_annotations(exp_dir):
    p = os.path.join(exp_dir, "scenario_annotations.yaml")
    if not os.path.exists(p):
        return []
    with open(p) as f:
        data = yaml.safe_load(f) or {}
    return [a for a in data.get("scenarios", [])
            if a.get("ground_truth") == "correspondence_failure"]


def fault_windows_from_ann(exp_dir, est_t, err):
    """每个 correspondence_failure 段: window max / error growth / recovery time。

    start/end 为相对 t0 (首帧) 的秒数。recovery = 段结束后误差首次回落至
    pre-base(段前 5s median) + 0.05m 的时间(相对段结束); 实验结束仍未回落记 None。
    """
    anns = load_annotations(exp_dir)
    t0 = est_t[0]
    rel = est_t - t0
    mag = np.linalg.norm(err, axis=1)
    out = []
    for a in anns:
        s, e = float(a["start"]), float(a["end"])
        pre = (rel >= s - 5.0) & (rel < s)
        seg = (rel >= s) & (rel <= e)
        post = rel > e
        pre_base = float(np.median(mag[pre])) if pre.any() else float(mag[0])
        d = {
            "name": a.get("name", "seg"),
            "window_s": [s, e],
            "window_max_err_m": float(mag[seg].max()) if seg.any() else np.nan,
            "error_growth_m": (float(mag[seg][-1]) - float(mag[seg][0]))
                              if seg.any() else np.nan,
        }
        thr = pre_base + 0.05
        hit = post & (mag <= thr)
        if post.any() and np.any(hit):
            d["recovery_time_s"] = float(rel[hit][0] - e)
        else:
            d["recovery_time_s"] = None  # 实验结束仍未回落
        out.append(d)
    return out


def exp_metrics(exp_dir, with_fault=False):
    """单个实验指标 (result.yaml + 对齐误差重算 final drift / max err / fault 段)。"""
    gt_tum = os.path.join(exp_dir, "groundtruth.txt")
    est_tum = os.path.join(exp_dir, "fastlio.txt")
    res = load_result(exp_dir)
    ate = res.get("ate", {}) or {}
    rpe = res.get("rpe", {}) or {}
    m = {
        "id": os.path.basename(exp_dir),
        "ate_rmse": float(ate.get("rmse_m", np.nan)),
        "ate_mean": float(ate.get("mean_m", np.nan)),
        "ate_max": float(ate.get("max_m", np.nan)),
        "rpe_rmse": float(rpe.get("rmse_m", np.nan)),
        "final_drift": np.nan,
        "max_err": np.nan,
        "fault_windows": [],
    }
    if os.path.exists(gt_tum) and os.path.exists(est_tum):
        est_t, err, _ = compute_aligned_error(gt_tum, est_tum)
        mag = np.linalg.norm(err, axis=1)
        if len(mag):
            m["final_drift"] = float(mag[-1])
            m["max_err"] = float(mag.max())
        if with_fault:
            m["fault_windows"] = fault_windows_from_ann(exp_dir, est_t, err)
    return m


def agg_stats(vals):
    a = np.array([v for v in vals if v is not None and not np.isnan(v)], dtype=float)
    if len(a) == 0:
        return {"median": np.nan, "min": np.nan, "max": np.nan, "n": 0}
    return {"median": float(np.median(a)), "min": float(a.min()),
            "max": float(a.max()), "n": int(len(a))}


def fmt(v):
    return f"{v:.4f}" if v is not None and not (isinstance(v, float) and np.isnan(v)) else "  n/a"


def main():
    ap = argparse.ArgumentParser(description="P3 A/B 严格对比聚合")
    ap.add_argument("--name", default="", help="对比组名")
    ap.add_argument("--degen", nargs="+", required=True, help="degen-only 实验目录")
    ap.add_argument("--adaptive", nargs="+", required=True, help="adaptive 实验目录")
    ap.add_argument("--fault", action="store_true",
                    help="计算 fault 段 window 指标 (需实验含 scenario_annotations.yaml)")
    args = ap.parse_args()

    degen = [exp_metrics(os.path.abspath(e), with_fault=args.fault) for e in args.degen]
    adapt = [exp_metrics(os.path.abspath(e), with_fault=args.fault) for e in args.adaptive]

    print(f"==== P3 A/B: {args.name} ====")
    hdr = f"{'exp':<14}{'ATE_r':>9}{'ATE_m':>9}{'ATE_M':>9}{'RPE':>9}{'fDrift':>9}{'mErr':>9}"
    print(hdr)
    print("-" * len(hdr))
    for m in degen + adapt:
        print(f"{m['id']:<14}{fmt(m['ate_rmse']):>9}{fmt(m['ate_mean']):>9}"
              f"{fmt(m['ate_max']):>9}{fmt(m['rpe_rmse']):>9}"
              f"{fmt(m['final_drift']):>9}{fmt(m['max_err']):>9}")

    print("\n---- 组聚合 (median / min / max) ----")
    keys = ["ate_rmse", "ate_mean", "ate_max", "rpe_rmse", "final_drift", "max_err"]
    for k in keys:
        d = agg_stats([m[k] for m in degen])
        a = agg_stats([m[k] for m in adapt])
        print(f"{k:<12} degen: med={fmt(d['median'])} [{fmt(d['min'])}~{fmt(d['max'])}]  "
              f"adapt: med={fmt(a['median'])} [{fmt(a['min'])}~{fmt(a['max'])}]")

    print("\n---- 判定 (改进 > 运行噪声 spread 才算显著) ----")
    for k in keys:
        d = agg_stats([m[k] for m in degen])
        a = agg_stats([m[k] for m in adapt])
        if d["n"] == 0 or a["n"] == 0:
            print(f"{k:<12} 数据不足, 跳过")
            continue
        spread = d["max"] - d["min"]
        diff = d["median"] - a["median"]
        if diff > 0 and diff > spread:
            print(f"{k:<12} 显著改进: med 差 {diff:.4f} > 噪声 {spread:.4f}  ✓")
        elif diff > 0:
            print(f"{k:<12} 改进不显著: med 差 {diff:.4f} ≤ 噪声 {spread:.4f} (噪声内)")
        elif diff == 0:
            print(f"{k:<12} 无差异")
        else:
            print(f"{k:<12} 变差: med 差 {diff:.4f} (<0)")

    if args.fault:
        print("\n---- fault 段 window 指标 (每实验) ----")
        for m in degen + adapt:
            w = m.get("fault_windows", [])
            if not w:
                print(f"{m['id']}: 无 correspondence_failure 标注")
                continue
            for d in w:
                rec = d["recovery_time_s"]
                rec_s = f"{rec:.1f}" if rec is not None else "no-reset"
                print(f"{m['id']} {d['name']} "
                      f"window[{d['window_s'][0]:.0f},{d['window_s'][1]:.0f}] "
                      f"maxErr={fmt(d['window_max_err_m'])} "
                      f"growth={fmt(d['error_growth_m'])} recovery={rec_s}s")


if __name__ == "__main__":
    main()
