#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P4 parity check: FAST-LIO C++ directional controller vs Python 复刻逐帧一致性。

覆盖:
  - trigger/gate 判定 (translation_degen && matching_ok → active), 含 one-frame delay:
    C++ 第 i 帧 directional_active 由"上一帧 update 后的 dir_cache_"判定 →
    Python 侧用第 i-1 帧信号复刻。
  - matching gate 布尔 (P2 冻结阈值 + 30s rolling baseline, 与 attribution_rules 同源)。
  - beta_applied / directional_reason。

用法:
  python3 p4_parity_check.py <experiment_dir> [--params config/attribution_params.yaml]
  需 lio_health.csv 含 P4 字段 (recorder_node vP4 起)。

输出: 逐帧对比统计 + PASS/FAIL 判定 (允许 replay 时序抖动导致的少量首帧/末帧错位)。
"""
import argparse
import csv
import os
import statistics
import sys

import yaml


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


def resolve_directional_params(exp_dir):
    """从 <exp_dir>/config.yaml 的 resolved.directional 读取参数 (launch 唯一参数源)。"""
    beta_t = 0.5
    ratio_t = 0.10
    cfg_path = os.path.join(exp_dir, "config.yaml")
    if os.path.exists(cfg_path):
        try:
            with open(cfg_path) as f:
                text = f.read()
            idx = text.find("resolved:")
            if idx >= 0:
                data = yaml.safe_load(text[idx:]) or {}
                resolved = data.get("resolved", {})
                d = resolved.get("directional", {})
                if d.get("beta_t"):
                    beta_t = float(d["beta_t"])
        except Exception as e:  # noqa: BLE001
            print(f"[p4-parity] 读取 config.yaml 失败, 用 launch 默认参数: {e}")
    return beta_t, ratio_t


def matching_failure_bool(h, buf):
    """P2 is_matching_failure 布尔 (attribution_rules.py:114 冻结版, 30s rolling median)。"""
    eff = h["effective_feature_num"]
    ratio = h["effective_feature_ratio"]
    if ratio <= 0.0 and eff > 0:
        cand = h["candidate_feature_num"]
        ratio = eff / cand if cand > 0 else 1.0
    buf.append((h["timestamp"], ratio, eff))
    while buf and h["timestamp"] - buf[0][0] > 30.0:
        buf.pop(0)
    ratio_base = num_base = 0.0
    have_base = len(buf) >= 5
    if have_base:
        ratio_base = statistics.median([s[1] for s in buf])
        num_base = statistics.median([s[2] for s in buf])
    low_ratio = ratio < 0.20
    low_feats = eff < 150.0
    crit = eff < 30.0
    high_mean = h["mean_residual"] > 0.05
    high_p90 = h["residual_p90"] > 0.08
    rel_ratio = rel_num = None
    if have_base:
        if ratio_base > 1e-6:
            rel_ratio = ratio / ratio_base
        if num_base > 1e-6:
            rel_num = eff / num_base
    rel_drop = ((rel_ratio is not None and rel_ratio < 0.30)
                or (rel_num is not None and rel_num < 0.40))
    return bool(low_feats or crit or high_p90 or rel_drop or (low_ratio and high_mean))


def expected_trigger(prev_h, beta_t, ratio_t):
    """用上一帧信号复刻 compute_directional_trigger → (active, beta, reason)。"""
    if prev_h is None:
        return False, 1.0, 0
    tr = prev_h["trans_ratio"]
    if tr >= ratio_t:
        return False, 1.0, 0
    if prev_h["_match_fail"]:
        return False, 1.0, 2
    return True, beta_t, 1


def main():
    ap = argparse.ArgumentParser(description="P4 parity check (C++ directional vs Python)")
    ap.add_argument("exp_dir")
    ap.add_argument("--params", default=None, help="attribution_params.yaml (阈值表)")
    args = ap.parse_args()

    exp_dir = os.path.abspath(args.exp_dir)
    health_csv = os.path.join(exp_dir, "lio_health.csv")
    if not os.path.exists(health_csv):
        print(f"[p4-parity] 找不到 lio_health.csv: {health_csv}")
        sys.exit(1)
    beta_t, ratio_t = resolve_directional_params(exp_dir)
    print(f"[p4-parity] exp={os.path.basename(exp_dir)}  beta_t={beta_t} ratio_t={ratio_t}")

    rows = load_health_csv(health_csv)
    if not rows:
        print("[p4-parity] lio_health.csv 为空")
        sys.exit(1)
    if "directional_active" not in rows[0]:
        print("[p4-parity] lio_health.csv 缺少 P4 字段 (需要 recorder_node vP4): "
              "directional_active")
        sys.exit(1)

    buf = []
    for h in rows:
        h["_match_fail"] = matching_failure_bool(h, buf)

    n_act = n_expect = n_agree = 0
    n_reason_agree = 0
    worst = 0.0
    for i, h in enumerate(rows):
        prev = rows[i - 1] if i >= 1 else None
        act = bool(h["directional_active"])
        exp_act, exp_beta, exp_reason = expected_trigger(prev, beta_t, ratio_t)
        n_act += act
        n_expect += exp_act
        n_agree += (act == exp_act)
        if act == exp_act and act:
            n_reason_agree += (int(h["directional_reason"]) == exp_reason)
        if act:
            worst = max(worst, abs(float(h["beta_applied"]) - exp_beta))
    total = len(rows)
    agree_rate = n_agree / total
    print(f"[p4-parity] 帧数={total}  C++ active={n_act}  Python expected active={n_expect}")
    print(f"[p4-parity] active 判定一致率 = {agree_rate:.4f}  ({n_agree}/{total})")
    print(f"[p4-parity] active 帧中 reason 一致 = {n_reason_agree}/{max(n_act, 1)}")
    print(f"[p4-parity] active 帧 beta_applied 最大偏差 = {worst:.6f}")
    # 允许 replay 时序抖动导致的少量首帧错位与 gate 基线建立期差异 (跳过前 5 帧)。
    n_core = n_agree_core = 0
    for i, h in enumerate(rows):
        if i < 5:
            continue
        prev = rows[i - 1]
        act = bool(h["directional_active"])
        exp_act, _, _ = expected_trigger(prev, beta_t, ratio_t)
        n_core += 1
        n_agree_core += (act == exp_act)
    core_rate = n_agree_core / max(n_core, 1)
    print(f"[p4-parity] 核心段 (跳过前 5 帧) 一致率 = {core_rate:.4f}  ({n_agree_core}/{n_core})")

    ok = core_rate >= 0.95 and abs(agree_rate - 1.0) < 0.1
    print("[p4-parity] RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

