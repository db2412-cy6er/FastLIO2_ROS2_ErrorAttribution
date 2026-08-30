#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P5.0 离线预演: first-iteration matching 统计分布分析与 gate 可行性评估。

背景 (P5 评审 #4/#5/#25):
  - P2 的 matching 阈值 (residual_high_th=0.05, residual_p90_th=0.08, feats_low_th=150,
    ratio_low_th=0.20, ratio_drop_th=0.30, num_drop_th=0.40) 是在 IEKF **收敛后**的
    final-iteration 上标定的;
  - P5 的 current-frame gate 要判断的是 **第一次迭代** (IMU propagated pose 下完成
    KD-tree correspondence 后的匹配质量), 该统计与 final-iteration 分布可能不同;
  - 本脚本读取 replay --debug-iterations 产出的 degen_iterations.csv, 取 iteration==1
    行 (= first-iteration), 分 clean / holdout fault / stress fault 三组:
      * 1) first-iter 与 final-iter 的分布差异量级 (决定复用 P2 阈值还是 P5 单独标定);
      * 2) first-iter 下 clean vs fault 的可分性 (决定 current-frame gate 是否可行);
      * 3) 用 P2 冻结阈值直接套 first-iter 的 false-rejection 率 (若正常帧误拒过多,
            证明必须单独标定);
      * 4) severity 直方图 (P2 规则口径的 severity_correspondence), 供 P5 标定参考。

输入: 一个或多个 exp 目录 (每个含 degen_iterations.csv)。
       fault 段真值由实验目录的 scenario_annotations.yaml 提供 (fault start/end)。

用法:
  python3 p5_analyze_first_iter.py <exp1_dir> [<exp2_dir> ...] [--fault-window S E]
输出: 控制台汇总 + p5_first_iter_report.yaml (默认写第一个 exp 目录)。
"""
import argparse
import csv
import os

import yaml

# P2 冻结阈值 (attribution_params.yaml v1.1, 与 attribution_rules.py / C++ 同源)
P2 = {
    "feats_critical_th": 30.0,
    "feats_low_th": 150.0,
    "ratio_low_th": 0.20,
    "residual_high_th": 0.05,
    "residual_p90_th": 0.08,
    "ratio_drop_th": 0.30,
    "num_drop_th": 0.40,
}


def clip01(x):
    return max(0.0, min(1.0, x))


def margin_above(value, thresh, sat=1.0):
    if sat <= 0.0:
        return 0.0
    return clip01((value - thresh) / sat)


def margin_below(value, thresh, sat=1.0):
    if sat <= 0.0:
        return 0.0
    return clip01((thresh - value) / sat)


def severity_correspondence(row):
    """复刻 attribution_rules.py classify() 的 severity_correspondence (P2 口径)。
    仅基于当前行字段, 不做 rolling baseline (P5.0 只评估绝对信号可分性)。"""
    eff = float(row["effective_feature_num"])
    cand = float(row["candidate_feature_num"])
    ratio = float(row["effective_feature_ratio"])
    if cand > 0:
        ratio = float(row["effective_feature_ratio"]) if ratio > 0 else eff / cand
    mean_res = float(row["mean_residual"])
    res_p90 = float(row["residual_p90"])
    s = max(
        margin_below(ratio, P2["ratio_low_th"], sat=max(P2["ratio_low_th"], 1e-6)),
        margin_below(eff, P2["feats_low_th"], sat=max(P2["feats_low_th"], 1.0)),
        margin_above(mean_res, P2["residual_high_th"], sat=max(P2["residual_high_th"], 1e-3)),
        margin_above(res_p90, P2["residual_p90_th"], sat=max(P2["residual_p90_th"], 1e-3)),
    )
    if eff < P2["feats_critical_th"]:
        s = 1.0
    return clip01(s)


def p2_matching_failure(row):
    """P2 冻结规则的 is_matching_failure (绝对信号, 不含 relative drop)。"""
    eff = float(row["effective_feature_num"])
    cand = float(row["candidate_feature_num"])
    ratio = float(row["effective_feature_ratio"]) if cand > 0 else 1.0
    mean_res = float(row["mean_residual"])
    res_p90 = float(row["residual_p90"])
    return (eff < P2["feats_low_th"] or ratio < P2["ratio_low_th"]
            or mean_res > P2["residual_high_th"] or res_p90 > P2["residual_p90_th"]
            or eff < P2["feats_critical_th"])


def relative_drop_mutation(rows, base_window_s=30.0, drop_ratio=0.30, drop_num=0.40):
    """对 first-iteration 时间序列计算 rolling-baseline relative drop (P2 通道)。

    P2.3 冻结: ratio_drop_th=0.30, num_drop_th=0.40, baseline window=30s。
    返回每个检测帧的 (timestamp, rel_ratio, rel_num, drop_trigger)。
    语义: rel = cur / baseline_median; drop_trigger = rel_ratio < 0.30 or rel_num < 0.40。
    baseline 只取 fault 前 (最近 30s 无 drop 触发帧的 median), 避免长 fault 段基线污染
    (P2 v1.1 修复; 这里用"上一个非 drop 帧之后的 clean 窗口")。
    """
    rows = sorted(rows, key=lambda r: float(r["timestamp"]))
    out = []
    base_ratio = []
    base_num = []
    # 初始化: 前 30s 的 rolling buffer
    ts = [float(r["timestamp"]) for r in rows]
    t0 = ts[0]
    for r in rows:
        t = float(r["timestamp"])
        # 清理过期
        while base_ratio and t - base_ratio[0][0] > base_window_s:
            base_ratio.pop(0)
        while base_num and t - base_num[0][0] > base_window_s:
            base_num.pop(0)
        eff = float(r["effective_feature_num"])
        cand = float(r["candidate_feature_num"])
        ratio = float(r["effective_feature_ratio"]) if cand > 0 else (eff / cand if cand > 0 else 1.0)
        rel_ratio = ratio / _median([v for _, v in base_ratio]) if base_ratio else None
        rel_num = eff / _median([v for _, v in base_num]) if base_num else None
        drop = (rel_ratio is not None and rel_ratio < drop_ratio) or \
               (rel_num is not None and rel_num < drop_num)
        out.append({"t": t, "rel_ratio": rel_ratio, "rel_num": rel_num,
                    "eff": eff, "ratio": ratio, "drop": bool(drop)})
        if not drop:
            base_ratio.append((t, ratio))
            base_num.append((t, eff))
    return out


def _median(vals):
    if not vals:
        return None
    s = sorted(vals)
    return s[len(s) // 2]

def _load_rows(exp_dir):
    """读 exp_dir/degen_iterations.csv, 返回 (rows_all, rows_first, rows_final)。
    iteration 语义: 每次 measurement update 内 h_share_model 自增, 从 1 起。
    first = iteration==1, final = 每帧最大 iteration (收敛后)。"""
    path = os.path.join(exp_dir, "degen_iterations.csv")
    if not os.path.isfile(path):
        return None, None, None
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        return None, None, None
    max_iter = {}
    for r in rows:
        try:
            fi = int(r["frame_index"])
        except (ValueError, KeyError):
            continue
        it = int(r["iteration"])
        max_iter[fi] = max(max_iter.get(fi, 0), it)
    first = [r for r in rows if int(r["iteration"]) == 1]
    final = [r for r in rows if int(r["iteration"]) == max_iter.get(int(r["frame_index"]), -1)]
    return rows, first, final


def _load_fault_windows(exp_dir):
    """读 scenario_annotations.yaml 的 fault 段 [start,end] 列表 (秒)。"""
    path = os.path.join(exp_dir, "scenario_annotations.yaml")
    if not os.path.isfile(path):
        return []
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    out = []
    for sc in (data.get("scenarios") or []):
        if "fault_" in str(sc.get("name", "")):
            out.append((float(sc["start"]), float(sc["end"])))
    return out


def _percentile(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    k = (len(s) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    w = k - lo
    return s[lo] * (1 - w) + s[hi] * w


def _stats(vals):
    if not vals:
        return {"n": 0, "mean": 0.0, "p10": 0.0, "p50": 0.0, "p90": 0.0,
                "min": 0.0, "max": 0.0}
    return {
        "n": len(vals),
        "mean": float(sum(vals) / len(vals)),
        "p10": _percentile(vals, 0.10),
        "p50": _percentile(vals, 0.50),
        "p90": _percentile(vals, 0.90),
        "min": min(vals),
        "max": max(vals),
    }

def _analyze_exp(exp_dir, fault_windows, buffer_s=3.0):
    rows, first, final = _load_rows(exp_dir)
    if first is None:
        print(f"[P5.0] {os.path.basename(exp_dir)}: 无 degen_iterations.csv (跳过)")
        return None
    print(f"[P5.0] {os.path.basename(exp_dir)}: 帧迭代 {len(rows)} 行, "
          f"first-iter {len(first)} 行, final-iter {len(final)} 行")

    def _in_fault(t):
        for s, e in fault_windows:
            if (s - buffer_s) <= t <= (e + buffer_s):
                return True
        return False

    def _part(rs):
        clean = [r for r in rs if not _in_fault(float(r["timestamp"]))]
        fault = [r for r in rs if _in_fault(float(r["timestamp"]))]
        return clean, fault

    fields = ["effective_feature_num", "effective_feature_ratio",
              "mean_residual", "residual_p90"]
    result = {"exp": os.path.basename(exp_dir), "fault_windows": fault_windows}
    # 1) first-iter vs final-iter 分布 (全体帧, 只用于看量级差异)
    for fn in fields:
        result[f"first_{fn}"] = _stats([float(r[fn]) for r in first])
        result[f"final_{fn}"] = _stats([float(r[fn]) for r in final])

    # 2) first-iter clean vs fault 可分性
    clean_f, fault_f = _part(first)
    for fn in fields:
        result[f"clean_first_{fn}"] = _stats([float(r[fn]) for r in clean_f])
        result[f"fault_first_{fn}"] = _stats([float(r[fn]) for r in fault_f])

    # 3) severity + P2 matching failure (first-iter)
    sev_all = [severity_correspondence(r) for r in first]
    sev_clean = [severity_correspondence(r) for r in clean_f]
    sev_fault = [severity_correspondence(r) for r in fault_f]
    result["severity_first_all"] = _stats(sev_all)
    result["severity_first_clean"] = _stats(sev_clean)
    result["severity_first_fault"] = _stats(sev_fault)

    mf_clean = [1.0 if p2_matching_failure(r) else 0.0 for r in clean_f]
    mf_fault = [1.0 if p2_matching_failure(r) else 0.0 for r in fault_f]
    result["p2_fail_first_clean"] = _stats(mf_clean)
    result["p2_fail_first_fault"] = _stats(mf_fault)
    result["_p2_fail_clean_count"] = int(sum(mf_clean))
    result["_p2_fail_clean_total"] = len(mf_clean)
    result["_p2_fail_fault_count"] = int(sum(mf_fault))
    result["_p2_fail_fault_total"] = len(mf_fault)

    hist = [0.0] * 5
    for s in sev_fault:
        idx = min(4, int(s * 5))
        hist[idx] += 1
    result["severity_fault_hist_020"] = [round(h / max(len(sev_fault), 1), 4) for h in hist]

    # 4) P5 专用: rolling-baseline relative drop (P2 通道, first-iter 上评估)
    #    不依赖 fault 标注绝对时间 (已验证 annotations 偏移 -3~-8s 不可靠),
    #    直接用信号突变评估 current-frame gate 的检测能力。
    mut = relative_drop_mutation(first, base_window_s=30.0,
                                 drop_ratio=P2["ratio_drop_th"],
                                 drop_num=P2["num_drop_th"])
    # 突变检测帧: drop=True 且 前 3s 内没有 drop (避免长段连续触发)
    trigger_frames = []
    last_trigger_t = -1e9
    for m in mut:
        if m["drop"] and (m["t"] - last_trigger_t) > 3.0:
            trigger_frames.append(m)
            last_trigger_t = m["t"]
    # 用 fault 窗口真值 (±缓冲) 计算: 突变帧落在 fault 段内 = 检出; 落在 clean = 误报
    hit, fp, total_trig = 0, 0, len(trigger_frames)
    for m in trigger_frames:
        if _in_fault(m["t"]):
            hit += 1
        else:
            fp += 1
    result["rel_drop_mutations"] = {
        "trigger_frames": total_trig,
        "in_fault": hit,
        "false_positive": fp,
        "clean_frames": sum(1 for r in first if not _in_fault(float(r["timestamp"]))),
        "fault_frames": sum(1 for r in first if _in_fault(float(r["timestamp"]))),
    }
    result["rel_drop_trigger_details"] = [
        {"t": round(m["t"], 2), "rel_ratio": round(m["rel_ratio"], 3) if m["rel_ratio"] else None,
         "rel_num": round(m["rel_num"], 3) if m["rel_num"] else None,
         "eff": m["eff"], "ratio": round(m["ratio"], 3)} for m in trigger_frames[:12]
    ]
    print(f"    rel-drop 突变 (P2 通道, first-iter): {total_trig} 次触发, "
          f"fault 段内 {hit}, clean 误报 {fp}")
    if trigger_frames:
        for m in trigger_frames[:12]:
            print(f"      t={m['t']:.1f}s rel_ratio={m['rel_ratio']} rel_num={m['rel_num']} "
                  f"eff={m['eff']} ratio={m['ratio']:.3f}")

    print(f"    first-iter: eff_num p50={result['first_effective_feature_num']['p50']:.1f} "
          f"ratio p50={result['first_effective_feature_ratio']['p50']:.3f} "
          f"mean_res p50={result['first_mean_residual']['p50']:.4f} "
          f"p90 p50={result['first_residual_p90']['p50']:.4f}")
    print(f"    final-iter: eff_num p50={result['final_effective_feature_num']['p50']:.1f} "
          f"ratio p50={result['final_effective_feature_ratio']['p50']:.3f} "
          f"mean_res p50={result['final_mean_residual']['p50']:.4f} "
          f"p90 p50={result['final_residual_p90']['p50']:.4f}")
    print(f"    P2 阈值直套 first-iter: clean 误拒 "
          f"{result['_p2_fail_clean_count']}/{result['_p2_fail_clean_total']} "
          f"({result['p2_fail_first_clean']['mean']*100:.2f}%), fault 命中 "
          f"{result['_p2_fail_fault_count']}/{result['_p2_fail_fault_total']} "
          f"({result['p2_fail_first_fault']['mean']*100:.2f}%)")
    if result["_p2_fail_fault_total"] == 0:
        print(f"    [info] 该 exp 无 fault 段真值窗口 (clean 对照, fault 指标为 0)")
    print(f"    severity: clean p50={result['severity_first_clean']['p50']:.3f} "
          f"p90={result['severity_first_clean']['p90']:.3f}, "
          f"fault p50={result['severity_first_fault']['p50']:.3f} "
          f"p90={result['severity_first_fault']['p90']:.3f}")
    return result


def main():
    ap = argparse.ArgumentParser(description="P5.0 first-iteration 分布分析")
    ap.add_argument("exps", nargs="+", help="experiment 目录 (含 degen_iterations.csv)")
    ap.add_argument("--tag", default="p5p0")
    ap.add_argument("--fault-window", nargs=2, type=float, default=None, metavar=("S", "E"),
                    help="手动 fault 窗口 (若目录内无 scenario_annotations.yaml)")
    ap.add_argument("--buffer", type=float, default=3.0,
                    help="fault 窗口 ±缓冲 (s, P2 口径: 吸收 injector t0 与 health 首帧 ~2s 偏差; "
                         "默认 3.0)")
    args = ap.parse_args()

    out_dir = os.path.abspath(args.exps[0])
    report = {"tag": args.tag, "exps": []}
    for exp in args.exps:
        fw = _load_fault_windows(exp)
        if not fw and args.fault_window:
            fw = [tuple(args.fault_window)]
        res = _analyze_exp(exp, fw, buffer_s=args.buffer)
        if res:
            report["exps"].append(res)

    print("\n========== P5.0 汇总 ==========")
    for res in report["exps"]:
        c_p50 = res["severity_first_clean"]["p50"]
        c_p90 = res["severity_first_clean"]["p90"]
        f_p50 = res["severity_first_fault"]["p50"]
        f_p90 = res["severity_first_fault"]["p90"]
        gap = max(0.0, f_p50 - c_p90)
        verdict = "可分 (fault p50 > clean p90)" if gap > 0.05 else \
                  ("部分可分 (fault p50 高于 clean p50)" if f_p50 > c_p50 else "不可分")
        print(f"[{res['exp']}] clean sev p50/p90 = {c_p50:.3f}/{c_p90:.3f}, "
              f"fault sev p50/p90 = {f_p50:.3f}/{f_p90:.3f}, gap={gap:.3f} -> {verdict}")

    report_path = os.path.join(out_dir, "p5_first_iter_report.yaml")
    with open(report_path, "w") as f:
        yaml.dump(report, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"\n[P5.0] 报告已写入 {report_path}")


if __name__ == "__main__":
    main()

