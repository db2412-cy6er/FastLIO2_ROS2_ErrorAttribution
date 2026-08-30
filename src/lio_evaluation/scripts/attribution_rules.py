#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P2 误差归因规则 (单一事实源)。

attribution_node.py (online) 与 attribution_validation.py --tune (offline)
都 import 本模块, 保证 online/offline 规则不产生分叉。

三层概念 (P2 修订版):
  Health Signal     -- 原始观测 (LioHealth 字段)
  Error Attribution -- 本模块输出: 信号级 flags + severity + 主标签 (风险来源)
  Error Consequence -- 由 GT 离线统计 (attribution_validation.py)

核心输出语义 (P2 冻结版):
  - flags (is_geometry_degen / is_matching_failure / is_imu_low_excitation)
    是核心结果, 可同时成立 (真实 LIO 故障中几何退化与低 IMU 激励完全可能共存);
  - severity_* 是 0~1 heuristic 分数 (margin 归一化, 不是概率意义上的置信度);
  - label 是显示摘要 = dominant_risk_label = argmax(severity_*), tie-break 用可配置
    priority; 它表示"当前同时存在的多个风险中 dominant 的那一个", 不是互斥的
    ground-truth class —— 走廊中 GEOMETRY_DEGENERATION 与 IMU_LOW_EXCITATION
    完全可能同时为 true, 此时 label 只回答"哪个更 dominant";
  - is_imu_low_excitation 只由 IMU 信号判定; imu_risk_score = 低激励 × 依赖几何
    (依赖几何 = 几何弱 或 位置协方差高), 表示"低激励是否正在影响定位"。
"""

LABELS = {
    0: "NORMAL",
    1: "GEOMETRY_DEGENERATION",
    2: "CORRESPONDENCE_FAILURE",
    3: "IMU_LOW_EXCITATION",
}
LABEL_IDS = {v: k for k, v in LABELS.items()}


def clip01(x):
    return max(0.0, min(1.0, x))


def margin_above(value, thresh, sat=1.0):
    """value 超过 thresh 越多 margin 越大; margin = clip01((value-thresh)/sat)。"""
    if sat <= 0.0:
        return 0.0
    return clip01((value - thresh) / sat)


def margin_below(value, thresh, sat=1.0):
    """value 低于 thresh 越多 margin 越大; margin = clip01((thresh-value)/sat)。"""
    if sat <= 0.0:
        return 0.0
    return clip01((thresh - value) / sat)

def classify(health, cfg, context=None):
    """health: dict, 字段与 LioHealth.msg 同名 (CSV 解析后或 msg 转 dict)。
    cfg: dict 阈值表 (attribution_params.yaml 的 attribution: 段)。
    context: dict, 可选在线/离线滚动基线 (如 {"ratio_baseline": 0.8})。
    返回 dict: label/label_str/margin/severity_*/flags/risk scores。
    """
    h = health
    c = cfg

    # ---- 阈值 (带默认值, 防缺失) ----
    feats_crit = c.get("feats_critical_th", 30)
    feats_low = c.get("feats_low_th", 100)
    ratio_low = c.get("ratio_low_th", 0.3)
    res_high = c.get("residual_high_th", 0.10)
    res_p90_th = c.get("residual_p90_th", res_high * 2.0)
    score_geom = c.get("score_geom_th", 0.50)
    ratio_t = c.get("ratio_t_th", 0.10)
    ratio_r = c.get("ratio_r_th", 0.02)
    gyr_exc_th = c.get("gyr_exc_th", 0.05)
    acc_exc_th = c.get("acc_exc_th", 0.15)
    cov_pos_high = c.get("cov_pos_high_th", 1e-2)
    ratio_drop_th = c.get("ratio_drop_th", 0.5)     # 相对占比骤降阈值 (相对滚动基线)
    num_drop_th = c.get("num_drop_th", 0.5)         # 相对有效特征数骤降阈值
    priority = c.get("priority",
                     ["CORRESPONDENCE_FAILURE", "GEOMETRY_DEGENERATION",
                      "IMU_LOW_EXCITATION", "NORMAL"])

    # ---- 输入 (缺省安全值) ----
    eff_num = float(h.get("effective_feature_num", 0.0))
    ratio = float(h.get("effective_feature_ratio", 1.0))
    if ratio <= 0.0 and eff_num > 0:
        cand = float(h.get("candidate_feature_num", 0.0))
        ratio = eff_num / cand if cand > 0 else 1.0
    mean_res = float(h.get("mean_residual", 0.0))
    res_p90 = float(h.get("residual_p90", mean_res))
    score = float(h.get("score", 0.0))
    trans_ratio = float(h.get("trans_ratio", 1.0))
    rot_ratio = float(h.get("rot_ratio", 1.0))
    gyr_exc = float(h.get("imu_gyr_excitation", 0.0))
    acc_exc = float(h.get("imu_acc_excitation", 0.0))
    pos_cov = float(h.get("pos_cov_trace", 0.0))

    # ========== 信号级 flags (可同时成立) ==========
    low_ratio = ratio < ratio_low
    low_feats = eff_num < feats_low
    crit_feats = eff_num < feats_crit
    high_mean = mean_res > res_high
    high_p90 = res_p90 > res_p90_th

    # 相对特征骤降 (相对滚动基线, 解决绝对阈值跨场景不泛化问题)
    ratio_baseline = (context or {}).get("ratio_baseline")
    num_baseline = (context or {}).get("num_baseline")
    rel_ratio = None
    rel_num = None
    if ratio_baseline and ratio_baseline > 1e-6:
        rel_ratio = ratio / ratio_baseline
    if num_baseline and num_baseline > 1e-6:
        rel_num = eff_num / num_baseline
    rel_drop = ((rel_ratio is not None and rel_ratio < ratio_drop_th)
                or (rel_num is not None and rel_num < num_drop_th))

    # P2 冻结修订: low_feats 独立触发 (数量型退化, 如 dropout/fov_crop 特征骤减但
    # ratio/残差不变), 不再要求同时 low_ratio 或 high_mean; 否则数量退化会漏检。
    is_matching_failure = bool(
        low_feats or crit_feats or high_p90 or rel_drop or (low_ratio and high_mean))

    geom_trigger = (score > score_geom or trans_ratio < ratio_t or rot_ratio < ratio_r)
    is_geometry_degen = bool(geom_trigger and not is_matching_failure)

    gyr_weak = gyr_exc < gyr_exc_th
    acc_weak = acc_exc < acc_exc_th
    is_imu_low_excitation = bool(gyr_weak and acc_weak)



    # ========== severity (0~1, heuristic margin) ==========
    s_geom = max(
        margin_above(score, score_geom, sat=max(score_geom, 1e-6)),
        margin_below(trans_ratio, ratio_t, sat=max(ratio_t, 1e-6)),
        margin_below(rot_ratio, ratio_r, sat=max(ratio_r, 1e-6)),
    )
    s_match = max(
        margin_below(ratio, ratio_low, sat=max(ratio_low, 1e-6)),
        margin_below(eff_num, feats_low, sat=max(feats_low, 1.0)),
        margin_above(mean_res, res_high, sat=max(res_high, 1e-3)),
        margin_above(res_p90, res_p90_th, sat=max(res_p90_th, 1e-3)),
    )
    if rel_ratio is not None:
        s_match = max(s_match, margin_below(rel_ratio, ratio_drop_th,
                                            sat=max(ratio_drop_th, 1e-6)))
    if rel_num is not None:
        s_match = max(s_match, margin_below(rel_num, num_drop_th,
                                            sat=max(num_drop_th, 1e-6)))
    if crit_feats:
        s_match = 1.0

    severity_geometry = clip01(s_geom)
    severity_correspondence = clip01(s_match)

    # covariance 仅作辅助 (v1 不单独触发)
    cov_score = margin_above(pos_cov, cov_pos_high, sat=max(cov_pos_high, 1e-6))

    # imu risk: 低激励 × 依赖几何 (几何弱 或 位置协方差高)
    reliance = max(severity_geometry, cov_score)
    imu_risk = is_imu_low_excitation * reliance
    severity_imu_weak = clip01(imu_risk)

    severity_normal = clip01(1.0 - max(severity_geometry,
                                       severity_correspondence,
                                       severity_imu_weak))

    severities = {
        "NORMAL": severity_normal,
        "GEOMETRY_DEGENERATION": severity_geometry,
        "CORRESPONDENCE_FAILURE": severity_correspondence,
        "IMU_LOW_EXCITATION": severity_imu_weak,
    }

    # ========== dominant risk label (显示摘要; argmax severity, tie-break 用 priority) ==========
    best = "NORMAL"
    best_s = -1.0
    for name in priority:
        s = severities.get(name, 0.0)
        if s > best_s + 1e-9:
            best_s = s
            best = name

    return {
        "label": LABEL_IDS[best],
        "label_str": best,
        "dominant_risk_label": best,   # P2 冻结版语义: 与 label 相同; 表示 dominant risk
        "margin": best_s,  # heuristic: 主标签的 severity, 非概率
        "severity_normal": severity_normal,
        "severity_geometry": severity_geometry,
        "severity_correspondence": severity_correspondence,
        "severity_imu_weak": severity_imu_weak,
        "is_geometry_degen": is_geometry_degen,
        "is_matching_failure": is_matching_failure,
        "is_imu_low_excitation": is_imu_low_excitation,
        "geom_risk_score": severity_geometry,
        "imu_risk_score": imu_risk,
        "cov_score": cov_score,
    }


def snapshot_from_health(health, cfg):
    """返回 ErrorAttribution 触发快照字段 (与 LioHealth 同名子集)。"""
    res = classify(health, cfg)
    res.update({
        "trans_ratio": health.get("trans_ratio", 0.0),
        "rot_ratio": health.get("rot_ratio", 0.0),
        "score": health.get("score", 0.0),
        "effective_feature_num": health.get("effective_feature_num", 0),
        "effective_feature_ratio": health.get("effective_feature_ratio", 0.0),
        "mean_residual": health.get("mean_residual", 0.0),
        "residual_p90": health.get("residual_p90", 0.0),
        "imu_gyr_excitation": health.get("imu_gyr_excitation", 0.0),
        "imu_acc_excitation": health.get("imu_acc_excitation", 0.0),
        "pos_cov_trace": health.get("pos_cov_trace", 0.0),
        "weak_translation_direction": list(health.get(
            "weak_translation_direction", [0.0, 0.0, 1.0])),
        "weak_mode_index": health.get("weak_mode_index", 0),
        "dominant_weak_axis": health.get("dominant_weak_axis", 2),
    })
    return res
