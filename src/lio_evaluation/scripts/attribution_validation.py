#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 误差归因离线验证 (两层验证, P2 修订版)。

第一层 · 分类正确性 (Classification):
  predicted (flags + 主标签) vs scenario_annotations.yaml 真值 (原因类别 × 原因类别)
  → per-class precision/recall; 每个标注段内的主标签多数票 + flag 检出率。

第二层 · 后果严重度 (Consequence):
  GT 误差区间 (LOW/MID/HIGH) 按标签做条件误差分布 (不叫 confusion matrix);
  每标注段的 ATE / error growth; 对齐后的弱方向误差占比。

坐标统一 (P2 修订版): FAST-LIO/GT 做 SE(3) Umeyama 刚性对齐 (无 scale),
  得到 R_align 后, 先把 weak_translation_direction 旋转到 GT 系再与位置误差点乘。

用法:
  python3 attribution_validation.py <experiment_dir> \
      [--params config/attribution_params.yaml] [--annotations <.../scenario_annotations.yaml>] \
      [--tune] [--bands 0.05,0.2]
  --tune: 从 lio_health.csv 用给定阈值离线重分类 (不依赖 online error_attribution.csv)
输出: attribution_report.yaml + attribution_distribution.png, 摘要并入 result.yaml
"""
import os
import sys
import csv
import argparse

import numpy as np
import yaml

from evo.core import sync
from evo.core import geometry
from evo.tools import file_interface

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from attribution_rules import classify, LABELS  # noqa: E402


# ----------------------------------------------------------------------
# IO helpers
# ----------------------------------------------------------------------
def load_tum(path):
    times, pos, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            times.append(float(p[0]))
            pos.append([float(x) for x in p[1:4]])
            quat.append([float(x) for x in p[4:8]])
    return np.array(times), np.array(pos), np.array(quat)


def load_health_csv(path):
    """lio_health.csv -> {timestamp: {field: value}}"""
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            t = float(r["timestamp"])
            d = {}
            for k, v in r.items():
                if k == "timestamp":
                    continue
                try:
                    d[k] = float(v)
                except ValueError:
                    d[k] = v
            rows[t] = d
    return rows


def load_attr_csv(path):
    """error_attribution.csv -> {timestamp: {field: value}}"""
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            t = float(r["timestamp"])
            d = {}
            for k, v in r.items():
                if k == "timestamp":
                    continue
                if k in ("is_geometry_degen", "is_matching_failure", "is_imu_low_excitation"):
                    d[k] = float(v) > 0.5
                else:
                    try:
                        d[k] = float(v)
                    except ValueError:
                        d[k] = v
            rows[t] = d
    return rows


def load_annotations(path):
    """scenario_annotations.yaml -> list of {name, start, end, ground_truth}"""
    if not os.path.exists(path):
        return []
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    anns = data.get("scenarios", [])
    out = []
    for a in anns:
        out.append({
            "name": a.get("name", "ann"),
            "start": float(a["start"]),
            "end": float(a["end"]),
            "ground_truth": a.get("ground_truth", "normal"),
        })
    return out


# 标注类别 → 期望 flag / 期望主标签
ANNOTATION_TO_FLAG = {
    "normal": None,                  # 期望全部 flag False 且主标签 NORMAL
    "geometry_degeneration": "is_geometry_degen",
    "correspondence_failure": "is_matching_failure",
    "imu_low_excitation": "is_imu_low_excitation",
}


def band_of(err, bounds):
    """err(m) -> 'LOW'|'MID'|'HIGH'"""
    if err < bounds[0]:
        return "LOW"
    if err < bounds[1]:
        return "MID"
    return "HIGH"


def compute_aligned_error(gt_tum, est_tum):
    """SE(3) Umeyama 刚性对齐 (无 scale), 返回:
      - est_t (n,), est_err (n,3) 对齐后逐帧位置误差
      - R_align (3,3) est->gt 旋转 (用于旋转 weak direction)
    """
    gt_traj = file_interface.read_tum_trajectory_file(gt_tum)
    est_traj = file_interface.read_tum_trajectory_file(est_tum)
    gt_sync, est_sync = sync.associate_trajectories(gt_traj, est_traj, max_diff=0.05)
    # 先取对齐旋转 (est->gt), 再执行对齐
    r_a, t_a, s = geometry.umeyama_alignment(
        est_sync.positions_xyz.T, gt_sync.positions_xyz.T, with_scale=False)
    est_sync.align(gt_sync, correct_scale=False)  # 刚性 SE(3), 无 scale
    est_t = est_sync.timestamps
    est_p = est_sync.positions_xyz
    gt_p = gt_sync.positions_xyz
    err = est_p - gt_p
    return est_t, err, r_a


def interpolate_err_to(est_t, err, target_t):
    """把逐帧误差插值到 target_t 时间戳 (逐轴 np.interp)。"""
    out = np.empty((len(target_t), 3))
    for i in range(3):
        out[:, i] = np.interp(target_t, est_t, err[:, i])
    return out


def weak_dir_frac(err, wt_aligned):
    """误差在(对齐后)弱方向上的投影占比。err: (n,3), wt_aligned: (n,3)。"""
    n = err.shape[0]
    frac = np.empty(n)
    for i in range(n):
        w = wt_aligned[i]
        nrm = np.linalg.norm(w)
        w = w / nrm if nrm > 1e-9 else np.array([1.0, 0.0, 0.0])
        total = np.linalg.norm(err[i])
        frac[i] = abs(w @ err[i]) / total if total > 1e-12 else 0.0
    return frac


def err_stats(errs):
    """errs: (n,3) -> stats dict"""
    mag = np.linalg.norm(errs, axis=1)
    return {
        "mean_m": float(mag.mean()),
        "median_m": float(np.median(mag)),
        "p90_m": float(np.percentile(mag, 90)),
        "max_m": float(mag.max()),
        "rmse_m": float(np.sqrt(np.mean(mag ** 2))),
    }


def error_band_distribution(errs, bounds):
    mag = np.linalg.norm(errs, axis=1)
    counts = {"LOW": 0, "MID": 0, "HIGH": 0}
    for m in mag:
        counts[band_of(m, bounds)] += 1
    total = len(mag)
    return {k: (float(v / total) if total else 0.0) for k, v in counts.items()}


# ----------------------------------------------------------------------
# 两层验证
# ----------------------------------------------------------------------
def build_frames(health_rows, attr_rows, est_t, err, R_align, cfg, tune):
    """把 health + attribution + GT 误差合成逐帧记录。
    tune=True 时用 classify() 从 health 重算归因, 否则用 error_attribution.csv。
    """
    import statistics as _stat
    import collections as _col
    frames = []
    base_window_s = float(cfg.get("ratio_baseline_window_s", 10.0))
    ratio_hist = _col.deque()
    for t in sorted(health_rows.keys()):
        h = health_rows[t]
        # 滚动 ratio/num 基线 (与 online attribution_node 一致)
        ratio_hist.append((t, float(h.get("effective_feature_ratio", 1.0)),
                           float(h.get("effective_feature_num", 0.0))))
        while ratio_hist and t - ratio_hist[0][0] > base_window_s:
            ratio_hist.popleft()
        baseline = None
        num_baseline = None
        if len(ratio_hist) >= 5:
            baseline = _stat.median([r for _, r, _ in ratio_hist])
            num_baseline = _stat.median([n for _, _, n in ratio_hist])
        context = {"ratio_baseline": baseline, "num_baseline": num_baseline}

        wt = np.array([h.get("wt0", 0.0), h.get("wt1", 0.0), h.get("wt2", 0.0)])
        nrm = np.linalg.norm(wt)
        wt = wt / nrm if nrm > 1e-9 else np.array([1.0, 0.0, 0.0])
        wt_align = R_align @ wt  # 弱方向旋转到 GT 系 (坐标统一)
        if tune:
            r = classify(h, cfg, context)
        else:
            a = attr_rows.get(t, {})
            r = {
                "label": a.get("label", 0),
                "label_str": a.get("label_str", "NORMAL"),
                "severity_geometry": a.get("severity_geometry", 0.0),
                "severity_correspondence": a.get("severity_correspondence", 0.0),
                "severity_imu_weak": a.get("severity_imu_weak", 0.0),
                "is_geometry_degen": bool(a.get("is_geometry_degen", False)),
                "is_matching_failure": bool(a.get("is_matching_failure", False)),
                "is_imu_low_excitation": bool(a.get("is_imu_low_excitation", False)),
            }
        frames.append({
            "t": t,
            "label_str": r["label_str"],
            "is_geometry_degen": r["is_geometry_degen"],
            "is_matching_failure": r["is_matching_failure"],
            "is_imu_low_excitation": r["is_imu_low_excitation"],
            "wt_aligned": wt_align,
        })
    target_t = np.array([fr["t"] for fr in frames])
    if len(target_t) == 0:
        return frames
    err_i = interpolate_err_to(est_t, err, target_t)
    for i, fr in enumerate(frames):
        fr["err"] = err_i[i]
    return frames


def layer_b_consequence(frames, bounds):
    """第二层: 每主标签 / 每 flag 的条件误差分布。"""
    keys = list(LABELS.values()) + ["is_geometry_degen", "is_matching_failure", "is_imu_low_excitation"]
    out = {}
    n_total = max(len(frames), 1)
    for key in keys:
        if key in LABELS.values():
            idx = [i for i, fr in enumerate(frames) if fr["label_str"] == key]
        else:
            idx = [i for i, fr in enumerate(frames) if fr[key]]
        if not idx:
            out[key] = None
            continue
        errs = np.array([frames[i]["err"] for i in idx])
        wt = np.array([frames[i]["wt_aligned"] for i in idx])
        d = err_stats(errs)
        d["n_frames"] = len(idx)
        d["n_ratio"] = float(len(idx) / n_total)
        d["band_distribution"] = error_band_distribution(errs, bounds)
        d["weak_dir_err_fraction_mean"] = float(np.mean(weak_dir_frac(errs, wt)))
        out[key] = d
    return out


def layer_a_classification(frames, annotations, buffer_s=3.0):
    """第一层: predicted vs scenario_annotations 真值 (原因类别混淆 + PR)。

    注: scenario_annotations 的 start/end 以首帧到达时刻为基准 (injector t0),
    而 frames[t] 是绝对仿真时刻 → 统一减 t0 后再比较 (P2 冻结修复)。
    buffer_s: fault 段评估用 ±buffer 缓冲 (injector t0 与 health 首帧有 ~2s
    系统偏差 + 效应建立时间); 与 calibrate_thresholds.py 标定口径一致。
    """
    if not annotations:
        return None
    t0 = frames[0]["t"] if frames else 0.0
    classes = sorted({a["ground_truth"] for a in annotations})
    ann_of_frame = {}
    for a in annotations:
        for i, fr in enumerate(frames):
            rel = fr["t"] - t0
            if a["start"] - buffer_s <= rel <= a["end"] + buffer_s:
                ann_of_frame[i] = a["ground_truth"]
    annotated_idx = [i for i in sorted(ann_of_frame)]

    per_annotation = []
    for a in annotations:
        idx = [i for i in annotated_idx
               if a["start"] - buffer_s <= frames[i]["t"] - t0 <= a["end"] + buffer_s]
        if not idx:
            per_annotation.append({"name": a["name"], "ground_truth": a["ground_truth"],
                                   "n_frames": 0})
            continue
        labels = [frames[i]["label_str"] for i in idx]
        majority = max(set(labels), key=labels.count)
        target_flag = ANNOTATION_TO_FLAG.get(a["ground_truth"])
        flag_det = {}
        if target_flag is not None:
            flag_det[target_flag] = float(sum(1 for i in idx if frames[i][target_flag]) / len(idx))
        else:
            flag_det["no_flag"] = float(sum(1 for i in idx
                                            if not (frames[i]["is_geometry_degen"]
                                                    or frames[i]["is_matching_failure"]
                                                    or frames[i]["is_imu_low_excitation"])) / len(idx))
        per_annotation.append({
            "name": a["name"], "ground_truth": a["ground_truth"],
            "n_frames": len(idx),
            "predicted_majority": majority,
            "flag_detection_rate": flag_det,
        })

    per_class = {}
    for cls in classes:
        target_flag = ANNOTATION_TO_FLAG.get(cls)
        pos = [i for i in annotated_idx if ann_of_frame[i] == cls]
        if target_flag is None:
            pred = [i for i in annotated_idx
                    if not (frames[i]["is_geometry_degen"] or frames[i]["is_matching_failure"]
                            or frames[i]["is_imu_low_excitation"])]
        else:
            pred = [i for i in annotated_idx if frames[i][target_flag]]
        tp = len(set(pos) & set(pred))
        per_class[cls] = {
            "n_true": len(pos),
            "n_pred": len(pred),
            "precision": float(tp / len(pred)) if pred else 0.0,
            "recall": float(tp / len(pos)) if pos else 0.0,
        }
    return {"annotated_frames": len(annotated_idx),
            "per_annotation": per_annotation,
            "per_class": per_class}


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="P2 误差归因两层验证")
    ap.add_argument("exp_dir")
    ap.add_argument("--params", default=None, help="attribution_params.yaml 路径")
    ap.add_argument("--annotations", default=None, help="scenario_annotations.yaml 路径")
    ap.add_argument("--tune", action="store_true",
                    help="从 lio_health.csv 用给定阈值离线重分类")
    ap.add_argument("--buffer", type=float, default=3.0,
                    help="fault 段 ±buffer 缓冲评估 (默认 3.0s, 吸收 injector t0 偏差)")
    ap.add_argument("--bands", default="0.05,0.2", help="误差区间边界 LOW/MID/HIGH (m)")
    args = ap.parse_args()

    out_dir = os.path.abspath(args.exp_dir)
    gt_tum = os.path.join(out_dir, "groundtruth.txt")
    est_tum = os.path.join(out_dir, "fastlio.txt")
    health_csv = os.path.join(out_dir, "lio_health.csv")
    attr_csv = os.path.join(out_dir, "error_attribution.csv")
    if not (os.path.exists(gt_tum) and os.path.exists(est_tum) and os.path.exists(health_csv)):
        print(f"缺少输入: {gt_tum} / {est_tum} / {health_csv}")
        sys.exit(1)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    params_path = args.params or os.path.join(script_dir, "..", "config", "attribution_params.yaml")
    with open(params_path) as f:
        cfg = (yaml.safe_load(f) or {}).get("attribution", {})
    ann_path = args.annotations or os.path.join(out_dir, "scenario_annotations.yaml")
    annotations = load_annotations(ann_path)
    bounds = [float(x) for x in args.bands.split(",")]

    health_rows = load_health_csv(health_csv)
    attr_rows = load_attr_csv(attr_csv) if os.path.exists(attr_csv) else {}

    est_t, err, R_align = compute_aligned_error(gt_tum, est_tum)
    frames = build_frames(health_rows, attr_rows, est_t, err, R_align, cfg, args.tune)
    if not frames:
        print("无有效帧, 无法验证")
        sys.exit(1)

    consequence = layer_b_consequence(frames, bounds)
    classification = layer_a_classification(frames, annotations,
                                            buffer_s=getattr(args, "buffer", 3.0))

    report = {
        "n_frames": len(frames),
        "params_file": os.path.basename(params_path),
        "tune": bool(args.tune),
        "error_bands_m": {"low": bounds[0], "mid": bounds[1]},
        "label_distribution": {k: (consequence[k]["n_ratio"] if consequence[k]
                                   else 0.0) for k in LABELS.values()},
        "label_error_condition": {k: consequence[k] for k in LABELS.values()},
        "flag_error_condition": {k: consequence[k] for k in
                                 ("is_geometry_degen", "is_matching_failure", "is_imu_low_excitation")},
        "classification": classification,
    }
    report_path = os.path.join(out_dir, "attribution_report.yaml")
    with open(report_path, "w") as f:
        yaml.dump(report, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    # 绘图: 主标签时间线 + |err| 曲线
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = frames[0]["t"]
    t_sec = np.array([fr["t"] - t0 for fr in frames])
    err_mag = np.array([np.linalg.norm(fr["err"]) for fr in frames])
    label_map = {name: i for i, name in enumerate(LABELS.values())}
    y_label = np.array([label_map[fr["label_str"]] for fr in frames])
    fig, ax1 = plt.subplots(figsize=(12, 6))
    ax1.plot(t_sec, err_mag, label="|err| (m)", color="black", lw=1.0)
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("|err| (m)")
    ax1.set_ylim(0, max(1.0, float(err_mag.max()) * 1.1))
    ax2 = ax1.twinx()
    ax2.plot(t_sec, y_label, drawstyle="steps-post", color="tab:blue", alpha=0.7, lw=1.5)
    ax2.set_yticks(list(label_map.values()))
    ax2.set_yticklabels(list(label_map.keys()), fontsize=8)
    ax2.set_ylim(-0.2, len(label_map) - 0.8)
    ax2.set_ylabel("dominant risk label")
    for a in annotations:
        ax1.axvspan(a["start"] - t0, a["end"] - t0, color="orange", alpha=0.15)
    fig.tight_layout()
    fig_path = os.path.join(out_dir, "attribution_distribution.png")
    fig.savefig(fig_path, dpi=120)
    plt.close(fig)

    print("=== P2 误差归因两层验证 ===")
    print("dominant risk label 分布:", {k: round(report["label_distribution"][k], 3) for k in LABELS.values()})
    for k in LABELS.values():
        c = consequence[k]
        if c:
            print(f"  {k}: n={c['n_frames']} "
                  f"band={ {b: round(v, 2) for b, v in c['band_distribution'].items()} } "
                  f"weak_frac={c['weak_dir_err_fraction_mean']:.2f}")
    if classification:
        print("分类(第一层):")
        for cls, v in classification["per_class"].items():
            print(f"  {cls}: precision={v['precision']:.2f} recall={v['recall']:.2f} "
                  f"(true={v['n_true']} pred={v['n_pred']})")
    print("已写入:", report_path, fig_path)

    ry = os.path.join(out_dir, "result.yaml")
    if os.path.exists(ry):
        with open(ry) as f:
            result = yaml.safe_load(f) or {}
        result["attribution"] = report
        with open(ry, "w") as f:
            yaml.dump(result, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        print("已更新:", ry)


if __name__ == "__main__":
    main()
