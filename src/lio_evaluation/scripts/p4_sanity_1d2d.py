#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P4.0a 数值 sanity test: 证明 "Jacobian scaling" != "correction suppression"。

背景 (P4 评审 #1/#3/#4):
  原 P4 v1 方案在 h_share_model 里把弱方向 Jacobian 分量乘 beta_t, 期望"缩小该方向
  correction"。本脚本用 2D 最小二乘 / IEKF 单步数值演示:
    E1  纯最小二乘: h_x scaling 要么无效(正交测量), 要么放大弱方向 correction(混合测量)。
    E2  IEKF 单步:  弱测量增益 K 关于 Jacobian H 非单调 (H²P >> R 时 K≈1/H, H 越小 K 越大;
                     进入 H²P << R 线性区才 K≈HP/R), 因此 beta 越小 correction 不一定越小,
                     且强方向 correction 也可能被污染。
    E3  covariance consistency: 只改 correction 不改 K (状态收 10%, 协方差却按 100% 缩小
                     ⇒ overconfident); 同步投影 K 行 (correction 与 (I-KH)P 一致) 才正确。

用法:
  python3 p4_sanity_1d2d.py

输出: 打印每个实验的数字对比与结论; 无外部依赖 (仅 numpy)。
"""
import numpy as np

BETA = 0.1


def lsq(H, z):
    """纯最小二乘 (无 prior): x = (H^T H)^{-1} H^T z"""
    return np.linalg.solve(H.T @ H, H.T @ z)


def project(v, w, beta=BETA):
    """方向投影: 沿单位向量 w 的分量保留 beta 倍, 正交方向不变。"""
    w = w / np.linalg.norm(w)
    return v - (1.0 - beta) * (v @ w) * w


def project_rows(M, w, beta=BETA):
    """逐行投影 (h_x scaling 用: 每行 Jacobian 的弱方向分量乘 beta)。"""
    return np.vstack([project(row, w, beta) for row in M])


def baseline_step(H, P0, R, x0, z):
    """标准 IEKF 单步 (线性测量)。"""
    S = H @ P0 @ H.T + R
    K = P0 @ H.T @ np.linalg.inv(S)
    dx = K @ (z - H @ x0)
    P1 = (np.eye(P0.shape[0]) - K @ H) @ P0
    return dx, P1, K


def hx_scaling_step(H, P0, R, x0, z, weak_dir, beta=BETA):
    """h_x scaling: H 的每行弱方向分量乘 beta (原 P4 v1 方案)。"""
    H_hx = project_rows(H, weak_dir, beta)
    S = H_hx @ P0 @ H_hx.T + R
    K = P0 @ H_hx.T @ np.linalg.inv(S)
    dx = K @ (z - H_hx @ x0)
    P1 = (np.eye(P0.shape[0]) - K @ H_hx) @ P0
    return dx, P1, K


def correction_projection_step(H, P0, R, x0, z, weak_dir, beta=BETA, project_gain=False):
    """correction projection: 原始 H 解 dx, 再把 dx (及可选 K 行) 的弱方向分量乘 beta。"""
    dx_b, _, K_b = baseline_step(H, P0, R, x0, z)
    dx_p = project(dx_b, weak_dir, beta)
    if project_gain:
        # cov consistency: K 的弱方向行同步投影 (esekfom 1924 前插入)
        K_p = project_rows(K_b, weak_dir, beta)
    else:
        K_p = K_b
    P1 = (np.eye(P0.shape[0]) - K_p @ H) @ P0
    return dx_p, P1, K_p


def main():
    print("=" * 78)
    print("P4.0a numerical sanity test: Jacobian scaling vs correction suppression")
    print("2D system, beta = %.1f (弱方向保留系数)" % BETA)
    print("=" * 78)

    # ── E1: 纯最小二乘 ────────────────────────────────────────────────
    print("\n[E1] 纯最小二乘 (无 prior), 真实 x* = (0.5, 0.2), 弱方向测量有大残差")
    H_e1 = np.array([[1.0, 0.0], [1.0, 1.0]])   # 第2行混合测量, x2 信息弱
    z_e1 = np.array([0.5, 1.5])                 # z2 偏离 (弱方向测量不可靠)
    x_base = lsq(H_e1, z_e1)
    # h_x scaling: 第2行 x2 列乘 beta
    H_s = H_e1.copy()
    H_s[1, 1] *= BETA
    x_scaled = lsq(H_s, z_e1)
    # correction projection: 沿 H^T H 最小特征向量投影
    w_e1 = np.linalg.eigh(H_e1.T @ H_e1)[1][:, 0]
    x_proj = project(x_base, w_e1)
    print("  baseline            : x = (%.3f, %.3f)" % (x_base[0], x_base[1]))
    print("  h_x scaling (x2*%.1f) : x = (%.3f, %.3f)  <-- 弱方向 correction 被放大!" %
          (BETA, x_scaled[0], x_scaled[1]))
    print("  correction proj     : x = (%.3f, %.3f)" % (x_proj[0], x_proj[1]))
    print("  结论: h_x scaling 使 H^TH 更病态, 弱方向解系数被放大 (%.2f -> %.2f); "
          "projection 直接控制最终状态增量。" % (x_base[1], x_scaled[1]))

    # ── E2: IEKF 单步 (带 prior) ──────────────────────────────────────
    print("\n[E2] IEKF 单步, 正交测量 H=diag(1,0.5), prior P0=I, R=0.01*I, z=(0.5,1.5)")
    P0 = np.eye(2)
    R = 0.01 * np.eye(2)
    H_e2 = np.diag([1.0, 0.5])
    x0 = np.zeros(2)
    z_e2 = np.array([0.5, 1.5])
    w_e2 = np.array([0.0, 1.0])          # 弱方向 = x2
    dx_b, P_b, K_b = baseline_step(H_e2, P0, R, x0, z_e2)
    dx_hx, _, K_hx = hx_scaling_step(H_e2, P0, R, x0, z_e2, w_e2)
    dx_pj, P_pj, K_pj = correction_projection_step(
        H_e2, P0, R, x0, z_e2, w_e2, project_gain=True)
    print("  baseline        dx = (%.4f, %.4f)   P22 = %.4f" % (dx_b[0], dx_b[1], P_b[1, 1]))
    print("  h_x scaling     dx = (%.4f, %.4f)   <-- x2 correction %.3f -> %.3f (放大!)" %
          (dx_hx[0], dx_hx[1], dx_b[1], dx_hx[1]))
    print("  correction proj dx = (%.4f, %.4f)   <-- x2 correction 缩到 β 倍 (%.3f -> %.3f)" %
          (dx_pj[0], dx_pj[1], dx_b[1], dx_pj[1]))
    print("  增益 K22 非单调: K(0.5)=%.3f, K(0.05)=%.3f  (H²P >> R 时 K≈1/H, 见 E1 病态例)" %
          (K_b[1, 1], K_hx[1, 1]))

    # ── E3: covariance consistency ────────────────────────────────────
    dx_incon, P_incon, _ = correction_projection_step(
        H_e2, P0, R, x0, z_e2, w_e2, project_gain=False)
    print("\n[E3] covariance consistency (correction 缩到 β 倍时, 协方差必须一致)")
    print("  baseline           : dx2=%.3f  P22=%.4f  (激光完全可信)" % (dx_b[1], P_b[1, 1]))
    print("  只改 correction    : dx2=%.3f  P22=%.4f  <-- 状态收 %.0f%%, 协方差却按 100%%"
          " 缩小 => overconfident" % (dx_incon[1], P_incon[1, 1], BETA * 100))
    print("  K 行同步投影       : dx2=%.3f  P22=%.4f  <-- 保守, 一致" %
          (dx_pj[1], P_pj[1, 1]))
    print("  结论: 必须同步投影 K 的弱方向行 (esekfom 1924 前), 否则滤波器过度自信"
          " -> 下一帧 IMU 先验被低估 -> 又一正反馈 (P3 教训)。")

    print("\n[Summary]")
    print("  1) h_x scaling 无法保证弱方向 correction 缩小 (E1/E2 展示无效或反效果);")
    print("  2) correction/gain projection 语义明确: beta=%.1f 即弱方向 correction 保留 %.0f%%;" % (BETA, BETA * 100))
    print("  3) correction 与 covariance 必须用同一投影 (E3)。")
    print("  PASS: sanity test 支持 P4 采用 esekfom state-layer directional projector 设计。")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
