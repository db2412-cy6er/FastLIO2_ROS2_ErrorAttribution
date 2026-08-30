#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 阈值标定 (分路, 复用 attribution_rules.classify, 不跑仿真)。

用法:
  python3 calibrate_thresholds.py --path matching \\
      --exps data/experiments/experiment_029 [--out /tmp/calib_matching.yaml]
  python3 calibrate_thresholds.py --path imu \\
      --exps data/experiments/experiment_037 data/experiments/experiment_038 \\
      [--motion auto] [--out /tmp/calib_imu.yaml]

标定原则 (P2 冻结版, 用户定稿):
  - geometry 路不动 (P1 已标定, 不为 P2 标签分布好看而改)。
  - matching 路只用 C3 (exp_029) 强化 fault 的已知时间窗 (correspondence_failure 真值)。
    V5 (exp_036) 严格排除在搜索之外, 只做最终验证 (避免 data leakage)。
  - IMU 路按 motion state (静止/匀速/转动/加速) 标定 gyr_exc_th / acc_exc_th:
    静止/匀速段期望 flag=True, 转动/加速段期望 flag=False —— motion-state precision,
    不用 GT 定位误差反向调 (保持物理语义)。
"""
import os
import sys
import csv
import argparse

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from attribution_rules import classify  # noqa: E402


# ----------------------------------------------------------------------
# 加载
# ----------------------------------------------------------------------
def load_health_csv(path):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            t = float(r["timestamp"])
            d = {k: v for k, v in r.items() if k != "timestamp"}
            rows[t] = d
    return rows


def load_annotations(path, target=None):
    """scenario_annotations.yaml -> [(start, end)]; target 过滤 ground_truth。"""
    if not os.path.exists(path):
        return []
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    segs = []
    for a in data.get("scenarios", []):
        if target is None or a.get("ground_truth") == target:
            segs.append((float(a["start"]), float(a["end"])))
    return segs


def build_context(rows, base_window_s=10.0):
    """与 online/offline 一致的滚动 ratio/num 基线。返回 {t: context}。"""
    import collections
    import statistics as _stat
    hist = collections.deque()
    ctx = {}
    for t in sorted(rows.keys()):
        h = rows[t]
        hist.append((t, float(h.get("effective_feature_ratio", 1.0)),
                     float(h.get("effective_feature_num", 0.0))))
        while hist and t - hist[0][0] > base_window_s:
            hist.popleft()
        base = None
        nbase = None
        if len(hist) >= 5:
            base = _stat.median([r for _, r, _ in hist])
            nbase = _stat.median([n for _, _, n in hist])
        ctx[t] = {"ratio_baseline": base, "num_baseline": nbase}
    return ctx


# ----------------------------------------------------------------------
# 帧标注与评估
# ----------------------------------------------------------------------
def annotate_frames(rows, fault_segs, buffer_s=3.0):
    """返回 (times, gt_fault) — 帧是否落在 correspondence fault 真值段内。

    注: scenario_annotations 的 start/end 以"首帧到达时刻 (injector t0)"为基准,
    而 lio_health.csv 的 timestamp 是绝对仿真时刻 → 须用相对首帧时间判定。
    buffer_s: injector t0 与 health 首帧存在 ~2s 系统偏差 + 效应建立时间,
    评估 fault recall 时对标注窗口做 ±buffer_s 缓冲 (P2 冻结标定口径)。
    """
    times = np.array(sorted(rows.keys()))
    t0 = times[0]
    gt = np.zeros(len(times), dtype=bool)
    for i, t in enumerate(times):
        for (s, e) in fault_segs:
            if s - buffer_s <= t - t0 <= e + buffer_s:
                gt[i] = True
                break
    return times, gt


def eval_matching(rows, ctx, cfg, times, gt_fault, fault_segs=()):
    n_fault = int(gt_fault.sum())
    n_clean = len(times) - n_fault
    pred = np.zeros(len(times), dtype=bool)
    for i, t in enumerate(times):
        r = classify(rows[t], cfg, ctx[t])
        pred[i] = bool(r["is_matching_failure"])
    tp = int((pred & gt_fault).sum())
    fp = int((pred & ~gt_fault).sum())
    recall = tp / n_fault if n_fault else 1.0
    fp_rate = fp / n_clean if n_clean else 0.0
    seg_hit = 0
    for (s, e) in fault_segs:
        m = pred & gt_fault
        if m[(times >= s) & (times <= e)].any():
            seg_hit += 1
    return {"recall": recall, "fp_rate": fp_rate, "n_fault": n_fault,
            "seg_hit": seg_hit, "n_seg": len(fault_segs)}


def detect_motion_states(rows, gyr_still=0.02, acc_still=0.05, gyr_turn=0.30):
    """从 IMU 数据检测物理运动状态 (独立于候选阈值, 不涉及定位误差)。

    stationary: 严格静止; turning: 强角速度; const_vel: 低角速度+低加速度;
    其余为 mixed (含加速/转向混合)。
    """
    times = sorted(rows.keys())
    out = {}
    for t in times:
        g = float(rows[t].get("imu_gyr_excitation", 1.0))
        a = float(rows[t].get("imu_acc_excitation", 1.0))
        if g < gyr_still and a < acc_still:
            out[t] = "stationary"
        elif g > gyr_turn:
            out[t] = "turning"
        elif g < 0.15 and a < 0.5:
            out[t] = "const_vel"
        else:
            out[t] = "mixed"
    return out


def eval_imu(rows, ctx, cfg, motion):
    """motion-state 语义评估: stationary/const_vel → flag True; turning/mixed → False。"""
    stats = {k: {"n": 0, "flag": 0} for k in ("stationary", "const_vel",
                                             "turning", "mixed")}
    for t in sorted(rows.keys()):
        r = classify(rows[t], cfg, ctx[t])
        flag = bool(r["is_imu_low_excitation"])
        m = motion[t]
        stats[m]["n"] += 1
        if flag:
            stats[m]["flag"] += 1
    out = {}
    for m, s in stats.items():
        out[m] = (s["flag"] / s["n"]) if s["n"] else None  # flag 命中率
    return out

# ----------------------------------------------------------------------
# 网格 / greedy 搜索
# ----------------------------------------------------------------------
MATCHING_GRID = {
    "residual_high_th": [0.05, 0.08, 0.10, 0.12, 0.15],
    "residual_p90_th": [0.08, 0.12, 0.16, 0.20, 0.25, 0.30],
    "ratio_low_th": [0.20, 0.30, 0.40, 0.50],
    "feats_low_th": [50, 100, 150, 200],
    "ratio_drop_th": [0.30, 0.40, 0.50, 0.60, 0.70],
    "num_drop_th": [0.30, 0.40, 0.50, 0.60, 0.70],
}
IMU_GYR_GRID = [0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10]
IMU_ACC_GRID = [0.05, 0.08, 0.12, 0.15, 0.20, 0.30]


def search_matching(exps, base_cfg, rounds=2, out=None, buffer_s=3.0, base_window_s=30.0):
    """逐参数 greedy 一维搜索 (参数耦合时迭代 2 轮)。"""
    rows_all, ctx_all, times_all, gt_all = {}, {}, [], []
    fault_segs = []
    for exp in exps:
        rows = load_health_csv(os.path.join(exp, "lio_health.csv"))
        segs = load_annotations(os.path.join(exp, "scenario_annotations.yaml"),
                                target="correspondence_failure")
        ctx = build_context(rows, base_window_s=base_window_s)
        times, gt = annotate_frames(rows, segs, buffer_s=buffer_s)
        rows_all.update(rows)
        ctx_all.update(ctx)
        times_all.extend(times)
        gt_all.extend(gt.tolist())
        fault_segs.extend(segs)
    times_all = np.array(times_all)
    gt_all = np.array(gt_all, dtype=bool)
    print(f"[matching] calibration 帧 n={len(times_all)}, "
          f"fault±{buffer_s:.0f}s 帧 n={int(gt_all.sum())}, fault 段 n={len(fault_segs)}")

    cfg = dict(base_cfg)
    cfg["ratio_baseline_window_s"] = base_window_s
    print(f"\n{'参数':<20}{'候选':>8}{'recall':>8}{'fp_rate':>9}{'seg_hit':>8}")
    for _ in range(rounds):
        for pname, cands in MATCHING_GRID.items():
            best, best_score = None, None
            for v in cands:
                c = dict(cfg)
                c[pname] = v
                m = eval_matching(rows_all, ctx_all, c, times_all, gt_all, fault_segs)
                # 目标: recall>=0.85 中选 fp 最低; 都达不到则选 recall 最高
                score = (0.85 - m["fp_rate"]) if m["recall"] >= 0.85 else m["recall"] * 0.5
                if best_score is None or score > best_score:
                    best, best_score = v, score
                    best_m = m
                print(f"{pname:<20}{v:>8.2f}{m['recall']:>8.3f}"
                      f"{m['fp_rate']:>9.3f}{m['seg_hit']:>6}/{m['n_seg']}")
            cfg[pname] = best
            print(f"  -> {pname} = {best} (recall={best_m['recall']:.3f}, "
                  f"fp={best_m['fp_rate']:.3f})")
    print("\n=== 冻结候选 (matching 路) ===")
    for k in MATCHING_GRID:
        print(f"  {k}: {cfg[k]}")
    if out:
        with open(out, "w") as f:
            yaml.dump({"path": "matching", "recommended": cfg,
                       "grid": MATCHING_GRID,
                       "buffer_s": buffer_s, "base_window_s": base_window_s},
                      f, allow_unicode=True)
        print(f"已保存: {out}")
    return cfg


def search_imu(exps, base_cfg, out=None):
    rows_all, ctx_all = {}, {}
    for exp in exps:
        rows = load_health_csv(os.path.join(exp, "lio_health.csv"))
        rows_all.update(rows)
        ctx_all.update(build_context(rows))
    motion = detect_motion_states(rows_all)
    n_stat = sum(1 for v in motion.values() if v == "stationary")
    n_turn = sum(1 for v in motion.values() if v == "turning")
    print(f"[imu] 帧 n={len(rows_all)}, stationary={n_stat}, "
          f"turning={n_turn}, const_vel={sum(1 for v in motion.values() if v=='const_vel')}")

    print(f"\n{'gyr':>6}{'acc':>7}{'stat_hit':>10}{'cv_hit':>8}{'turn_fp':>9}{'acc':>7}")
    best = None
    for g in IMU_GYR_GRID:
        for a in IMU_ACC_GRID:
            cfg = dict(base_cfg)
            cfg["gyr_exc_th"] = g
            cfg["acc_exc_th"] = a
            st = eval_imu(rows_all, ctx_all, cfg, motion)
            stat_hit = st["stationary"]
            cv_hit = st["const_vel"]
            turn_fp = st["turning"]      # turning 段 flag 命中 = 误报
            mixed = st["mixed"]
            ok_m = [x for x in (stat_hit, cv_hit) if x is not None]
            acc = np.mean(ok_m) if ok_m else 0.0
            acc = acc - (turn_fp or 0.0) - (mixed or 0.0)
            print(f"{g:>6.2f}{a:>7.2f}"
                  f"{stat_hit if stat_hit is None else round(stat_hit,2):>10}"
                  f"{cv_hit if cv_hit is None else round(cv_hit,2):>8}"
                  f"{turn_fp if turn_fp is None else round(turn_fp,2):>9}"
                  f"{acc:>7.3f}")
            if best is None or acc > best[0]:
                best = (acc, g, a, stat_hit, cv_hit, turn_fp)
    acc, g, a, stat_hit, cv_hit, turn_fp = best
    print(f"\n=== 推荐 (IMU 路): gyr_exc_th={g}, acc_exc_th={a} ===")
    print(f"  stationary 命中={stat_hit:.3f}, const_vel 命中={cv_hit:.3f}, "
          f"turning 误报={turn_fp:.3f}")
    if out:
        with open(out, "w") as f:
            yaml.dump({"path": "imu", "recommended": {"gyr_exc_th": g, "acc_exc_th": a}},
                      f, allow_unicode=True)
        print(f"已保存: {out}")
    return {"gyr_exc_th": g, "acc_exc_th": a}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--path", choices=["matching", "imu"], required=True)
    p.add_argument("--exps", nargs="+", required=True)
    p.add_argument("--params",
                   default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "config", "attribution_params.yaml"))
    p.add_argument("--out", default=None)
    p.add_argument("--buffer", type=float, default=3.0,
                   help="fault 段 ±buffer 缓冲 (吸收 injector t0 与 health 首帧偏差)")
    p.add_argument("--base-window", type=float, default=30.0,
                   help="相对骤降滚动基线窗口 (s)")
    a = p.parse_args()

    with open(a.params) as f:
        cfg = (yaml.safe_load(f) or {}).get("attribution", {})
    if a.path == "matching":
        search_matching(a.exps, cfg, out=a.out, buffer_s=a.buffer,
                        base_window_s=a.base_window)
    else:
        search_imu(a.exps, cfg, out=a.out)


if __name__ == "__main__":
    main()

