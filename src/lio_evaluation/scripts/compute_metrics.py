#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lio_evaluation 离线评测 (M4)

输入: 一个 experiment_XXX/ 目录 (TUM 轨迹 + degen CSVs + resource/frametime CSV)
输出:
  - groundtruth_aligned.txt   (按 est 时间戳插值的 GT, 供 evo)
  - ate_result.zip / rpe_result.zip   (evo PE/RPE --save_results 同结构 zip)
  - runtime.csv                (帧耗时 + CPU + 内存合并)
  - result.yaml                (汇总指标)

时间对齐: GT 按 est 时间戳线性插值(位置) + slerp(姿态)
坐标对齐: Trajectory.aligned (SE(3) Umeyama), 解决 camera_init vs map
"""

import os
import sys
import csv
import subprocess

import numpy as np
import yaml

from evo.core import sync
from evo.core.metrics import APE, RPE, PoseRelation, Unit
from evo.tools import file_interface


# ----------------------------------------------------------------------
# 四元数/轨迹工具
# ----------------------------------------------------------------------
def slerp(q0, q1, t):
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    if q0.dot(q1) < 0.0:
        q1 = -q1
    omega = np.arccos(np.clip(q0.dot(q1), -1.0, 1.0))
    if abs(omega) < 1e-9:
        return q0
    return (np.sin((1.0 - t) * omega) * q0 + np.sin(t * omega) * q1) / np.sin(omega)


def interpolate_gt(gt, est_times):
    """gt: dict {time: (pos3, quat4)}, 返回按 est_times 插值后的 pos/quat 数组"""
    gt_times = np.array(sorted(gt.keys()))
    gt_pos = np.array([gt[t][0] for t in gt_times])
    gt_quat = np.array([gt[t][1] for t in gt_times])
    out_pos = np.empty((len(est_times), 3))
    out_quat = np.empty((len(est_times), 4))
    for i, t in enumerate(est_times):
        idx = np.searchsorted(gt_times, t)
        idx = min(max(idx, 1), len(gt_times) - 1)
        t0, t1 = gt_times[idx - 1], gt_times[idx]
        if t1 - t0 < 1e-12:
            out_pos[i] = gt_pos[idx]
            out_quat[i] = gt_quat[idx]
            continue
        s = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
        out_pos[i] = gt_pos[idx - 1] + s * (gt_pos[idx] - gt_pos[idx - 1])
        out_quat[i] = slerp(gt_quat[idx - 1], gt_quat[idx], s)
    return out_pos, out_quat


def load_tum(path):
    """返回 (times[], pos[], quat[]) 与 dict {time:(pos,quat)}"""
    times, pos, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            p = [float(x) for x in parts[1:4]]
            q = [float(x) for x in parts[4:8]]
            times.append(t)
            pos.append(p)
            quat.append(q)
    return np.array(times), np.array(pos), np.array(quat)


def write_tum(path, times, pos, quat):
    with open(path, "w") as f:
        for t, p, q in zip(times, pos, quat):
            f.write(f"{t:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


# ----------------------------------------------------------------------
# evo 指标
# ----------------------------------------------------------------------
def _run_evo_cli(tool, gt_tum, est_tum, out_zip):
    """调用 evo_ape / evo_rpe 生成标准 evo result zip"""
    try:
        cmd = [tool, "tum", gt_tum, est_tum, "-a", "--no_warnings"]
        if tool == "evo_rpe":
            cmd += ["--delta", "1", "--delta_unit", "m"]
        cmd += ["--save_results", out_zip]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            print(f"[warn] {tool} 失败 rc={proc.returncode}: "
                  f"stderr={proc.stderr[-500:]!r} stdout={proc.stdout[-200:]!r}")
    except FileNotFoundError:
        print(f"[warn] 未找到 {tool}, 跳过 zip 生成 (仍输出 stats)")


def compute_evo_metrics(gt_tum, est_tum, out_dir):
    """时间对齐(插值) + SE(3) Umeyama 对齐 + ATE/RPE + 生成 evo zip"""
    gt_times, gt_pos, gt_quat = load_tum(gt_tum)
    est_times, est_pos, est_quat = load_tum(est_tum)
    n_gt, n_est = len(gt_times), len(est_times)

    # 以 est 为基准, 对 GT 插值 -> groundtruth_aligned.txt
    gt_map = {t: (p, q) for t, p, q in zip(gt_times, gt_pos, gt_quat)}
    a_pos, a_quat = interpolate_gt(gt_map, est_times)
    aligned_gt = os.path.join(out_dir, "groundtruth_aligned.txt")
    write_tum(aligned_gt, est_times, a_pos, a_quat)

    gt_traj = file_interface.read_tum_trajectory_file(aligned_gt)
    est_traj = file_interface.read_tum_trajectory_file(est_tum)

    # evo 内部关联 (时间戳已对齐)
    gt_sync, est_sync = sync.associate_trajectories(gt_traj, est_traj, max_diff=0.05)
    n_used = len(gt_sync.timestamps)

    # SE(3) Umeyama 对齐 (坐标对齐: camera_init vs map); align() 原地修改 est_sync
    est_sync.align(gt_sync, correct_scale=False)
    est_aligned = est_sync

    # ATE
    ape = APE(pose_relation=PoseRelation.translation_part)
    ape.process_data((gt_sync, est_aligned))
    ape_stats = ape.get_all_statistics()

    # RPE (1m 间隔, 平移部分)
    # 轨迹过短（机器人几乎未移动/出生点被卡住）时 evo 会抛 FilterException，
    # 优雅降级为 rpe: null，避免整个 eval 崩溃（仍保留 ATE/directional 结果）。
    rpe_stats = None
    try:
        rpe = RPE(pose_relation=PoseRelation.translation_part, delta=1.0,
                  delta_unit=Unit.meters, all_pairs=False)
        rpe.process_data((gt_sync, est_aligned))
        rpe_stats = rpe.get_all_statistics()
    except Exception as e:
        print(f"[compute_metrics][WARN] RPE 计算失败(轨迹过短?): {e}")

    # 生成 evo result zip (同 evo_ape/evo_rpe --save_results 口径, 复用已对齐轨迹)
    ape_zip = os.path.join(out_dir, "ate_result.zip")
    rpe_zip = os.path.join(out_dir, "rpe_result.zip")
    _run_evo_cli("evo_ape", aligned_gt, est_tum, ape_zip)
    try:
        _run_evo_cli("evo_rpe", aligned_gt, est_tum, rpe_zip)
    except Exception as e:
        print(f"[compute_metrics][WARN] evo_rpe 失败: {e}")

    return {
        "n_gt": int(n_gt),
        "n_est": int(n_est),
        "n_used": int(n_used),
        "duration_s": float(est_times[-1] - est_times[0]) if n_est > 1 else 0.0,
        "length_m": float(np.sum(np.linalg.norm(np.diff(est_pos, axis=0), axis=1))),
        "dt_median_s": float(np.median(np.diff(est_times))) if n_est > 1 else 0.0,
        "ape": {
            "rmse_m": float(ape_stats["rmse"]),
            "mean_m": float(ape_stats["mean"]),
            "median_m": float(ape_stats["median"]),
            "max_m": float(ape_stats["max"]),
            "std_m": float(ape_stats["std"]),
            "align": "umeyama_se3",
        },
        "rpe": None if rpe_stats is None else {
            "delta_m": 1.0,
            "rmse_m": float(rpe_stats["rmse"]),
            "mean_m": float(rpe_stats["mean"]),
            "median_m": float(rpe_stats["median"]),
            "max_m": float(rpe_stats["max"]),
            "align": "umeyama_se3",
        },
    }

# ----------------------------------------------------------------------
# 退化分数统计 + 双检测器一致性
# ----------------------------------------------------------------------
def load_degen_csv(path):
    if not os.path.exists(path):
        return None
    rows = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows[float(r["timestamp"])] = {
                    "score": float(r["score"]),
                    "is_degenerate": int(r["is_degenerate"]),
                    "trans_ratio": float(r["trans_ratio"]),
                    "rot_ratio": float(r["rot_ratio"]),
                    "normal_concentration": float(r["normal_concentration"]),
                    "effective_feature_num": int(r["effective_feature_num"]),
                    "mean_residual": float(r["mean_residual"]),
                    "weak": [float(r[k]) for k in ("w0", "w1", "w2", "w3", "w4", "w5")],
                }
            except (ValueError, KeyError):
                continue
    return rows


def series_from_rows(rows, key):
    t = np.array(sorted(rows.keys()))
    v = np.array([rows[x][key] for x in t])
    return t, v


def interpolate_series(src_t, src_v, dst_t, tol=0.2):
    """把 src 序列按 dst 时间戳最近邻重采样 (窗口 tol 秒)"""
    out = np.full(len(dst_t), np.nan)
    for i, t in enumerate(dst_t):
        m = np.where(np.abs(src_t - t) <= tol)[0]
        if m.size == 0:
            continue
        j = m[np.argmin(np.abs(src_t[m] - t))]
        out[i] = src_v[j]
    return out


def pearson(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = ~(np.isnan(a) | np.isnan(b))
    if m.sum() < 3:
        return float("nan")
    a, b = a[m], b[m]
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def spearman(a, b):
    from scipy.stats import spearmanr
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = ~(np.isnan(a) | np.isnan(b))
    if m.sum() < 3:
        return float("nan")
    return float(spearmanr(a[m], b[m]).correlation)


def consistency_analysis(exact_rows, proxy_rows):
    if exact_rows is None or proxy_rows is None:
        return None
    et, es = series_from_rows(exact_rows, "score")
    pt, ps = series_from_rows(proxy_rows, "score")
    es_interp = interpolate_series(pt, ps, et)
    m = ~np.isnan(es_interp)
    if m.sum() < 5:
        return None

    rmse = float(np.sqrt(np.mean((es[m] - es_interp[m]) ** 2)))
    corr_p = pearson(es[m], es_interp[m])
    corr_s = spearman(es[m], es_interp[m])

    # event overlap (is_degenerate 二值)
    e_d = np.array([exact_rows[t]["is_degenerate"] for t in et if t in exact_rows])
    p_d = np.array([proxy_rows[t]["is_degenerate"] for t in pt])
    p_d_interp = interpolate_series(pt, p_d, et)
    p_d_bin = np.where(p_d_interp > 0.5, 1.0, 0.0)
    inter = float(np.sum((e_d > 0.5) & (p_d_bin > 0.5)))
    union = float(np.sum((e_d > 0.5) | (p_d_bin > 0.5)))
    overlap = inter / union if union > 0 else 0.0

    def first_enter(rows):
        for t in sorted(rows.keys()):
            if rows[t]["is_degenerate"]:
                return t
        return None

    t1, t2 = first_enter(exact_rows), first_enter(proxy_rows)
    delay = (t2 - t1) if (t1 is not None and t2 is not None) else None

    return {
        "pearson": corr_p,
        "spearman": corr_s,
        "rmse": rmse,
        "event_overlap": overlap,
        "detection_delay_s": delay,
    }


def degen_stats(rows):
    if rows is None or len(rows) == 0:
        return None
    _, score = series_from_rows(rows, "score")
    _, deg = series_from_rows(rows, "is_degenerate")
    _, trans = series_from_rows(rows, "trans_ratio")
    _, rot = series_from_rows(rows, "rot_ratio")
    _, nc = series_from_rows(rows, "normal_concentration")
    _, feats = series_from_rows(rows, "effective_feature_num")
    return {
        "frames": int(len(score)),
        "mean_score": float(np.nanmean(score)),
        "degraded_ratio": float(np.nanmean(deg)),
        "mean_trans_ratio": float(np.nanmean(trans)),
        "mean_rot_ratio": float(np.nanmean(rot)),
        "mean_normal_concentration": float(np.nanmean(nc)),
        "mean_effective_feature_num": float(np.nanmean(feats)),
    }


# ----------------------------------------------------------------------
# runtime.csv 合并
# ----------------------------------------------------------------------
def merge_runtime(out_dir):
    ft = os.path.join(out_dir, "frametime.csv")
    res = os.path.join(out_dir, "resource.csv")
    if not (os.path.exists(ft) and os.path.exists(res)):
        return None

    with open(ft) as f:
        ft_rows = list(csv.DictReader(f))
    with open(res) as f:
        res_rows = list(csv.DictReader(f))

    def num(x):
        try:
            v = float(x)
            return v if not np.isnan(v) else None
        except (ValueError, TypeError):
            return None

    res_wall = np.array([num(r["wall_time"]) for r in res_rows], dtype=float)
    out_path = os.path.join(out_dir, "runtime.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "loop_time_ms", "cpu_percent", "rss_mb", "vms_mb", "threads"])
        for r in ft_rows:
            t = float(r["timestamp"])
            lt = num(r["loop_time_ms"])
            cpu = rss = vms = thr = None
            valid = res_wall[~np.isnan(res_wall)]
            if valid.size:
                i = int(np.argmin(np.abs(valid - t)))
                rr = res_rows[i]
                cpu, rss, vms, thr = (num(rr["cpu_percent"]), num(rr["rss_mb"]),
                                      num(rr["vms_mb"]), num(rr["threads"]))
            w.writerow([
                f"{t:.9f}", f"{lt:.3f}" if lt is not None else "nan",
                f"{cpu:.2f}" if cpu is not None else "nan",
                f"{rss:.2f}" if rss is not None else "nan",
                f"{vms:.2f}" if vms is not None else "nan",
                f"{thr}" if thr is not None else "nan",
            ])
    return out_path


def runtime_stats(out_dir):
    rt = os.path.join(out_dir, "runtime.csv")
    if not os.path.exists(rt):
        return None
    with open(rt) as f:
        rows = list(csv.DictReader(f))
    lt = np.array([float(r["loop_time_ms"]) for r in rows if r["loop_time_ms"] != "nan"])
    cpu = np.array([float(r["cpu_percent"]) for r in rows if r["cpu_percent"] != "nan"])
    rss = np.array([float(r["rss_mb"]) for r in rows if r["rss_mb"] != "nan"])
    vms = np.array([float(r["vms_mb"]) for r in rows if r["vms_mb"] != "nan"])
    out = {}
    if lt.size:
        out["loop_time_ms"] = {"mean": float(lt.mean()), "max": float(lt.max()),
                               "p99": float(np.percentile(lt, 99))}
    if cpu.size:
        out["cpu_percent"] = {"mean": float(cpu.mean()), "max": float(cpu.max())}
    if rss.size:
        out["rss_mb"] = {"mean": float(rss.mean()), "max": float(rss.max())}
    if vms.size:
        out["vms_mb"] = {"mean": float(vms.mean()), "max": float(vms.max())}
    return out


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print("用法: compute_metrics.py <experiment_dir>")
        sys.exit(1)
    out_dir = sys.argv[1]
    gt_tum = os.path.join(out_dir, "groundtruth.txt")
    est_tum = os.path.join(out_dir, "fastlio.txt")
    if not (os.path.exists(gt_tum) and os.path.exists(est_tum)):
        print(f"缺少轨迹文件: {gt_tum} 或 {est_tum}")
        sys.exit(1)

    evo = compute_evo_metrics(gt_tum, est_tum, out_dir)
    print("ATE (rmse/mean/max): %.4f / %.4f / %.4f m" % (
        evo["ape"]["rmse_m"], evo["ape"]["mean_m"], evo["ape"]["max_m"]))
    if evo["rpe"] is None:
        print("RPE: null (轨迹过短, 未计算)")
    else:
        print("RPE (rmse/mean/max): %.4f / %.4f / %.4f m" % (
            evo["rpe"]["rmse_m"], evo["rpe"]["mean_m"], evo["rpe"]["max_m"]))

    exact_rows = load_degen_csv(os.path.join(out_dir, "degen_exact.csv"))
    proxy_rows = load_degen_csv(os.path.join(out_dir, "degen_proxy.csv"))

    # 先合并 runtime.csv, 再统计
    merge_runtime(out_dir)

    result = {
        "experiment": os.path.basename(out_dir.rstrip("/")),
        "trajectory": {
            "n_gt": evo["n_gt"], "n_est": evo["n_est"], "n_used": evo["n_used"],
            "duration_s": evo["duration_s"], "length_m": evo["length_m"],
            "dt_median_s": evo["dt_median_s"],
        },
        "ate": evo["ape"],
        "rpe": evo["rpe"],
        "degeneracy": {
            "exact": degen_stats(exact_rows),
            "proxy": degen_stats(proxy_rows),
            "consistency": None,
        },
        "runtime": runtime_stats(out_dir),
    }

    cons = consistency_analysis(exact_rows, proxy_rows)
    if cons:
        result["degeneracy"]["consistency"] = cons
        print("一致性 (pearson/spearman/rmse/overlap/delay): %.3f / %.3f / %.4f / %.3f / %s" % (
            cons["pearson"], cons["spearman"], cons["rmse"],
            cons["event_overlap"], cons["detection_delay_s"]))

    yaml_path = os.path.join(out_dir, "result.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(result, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print("已写入:", yaml_path)
    print("已生成:", os.path.join(out_dir, "ate_result.zip"),
          os.path.join(out_dir, "rpe_result.zip"),
          os.path.join(out_dir, "runtime.csv"))


if __name__ == "__main__":
    main()

