// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <Eigen/Eigenvalues>
#include <limits>
#include <deque>
#include <vector>
#include <algorithm>
#include <lio_interfaces/msg/degeneracy_score.hpp>
#include <lio_interfaces/msg/degeneracy_iteration.hpp>
#include <lio_interfaces/msg/lio_health.hpp>
#include <lio_interfaces/msg/error_attribution.hpp>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

/*** ==================== Degeneracy Detection (P1) ====================
 * 内嵌精确检测器: 基于真实参与 IEKF 更新的测量 Jacobian (h_x) 累积
 * LiDAR geometric Hessian (measurement Hessian):
 *   Ht = Σ n nᵀ              (3x3, 平动块, 世界系法向量)
 *   Hr = Σ (p×n)(p×n)ᵀ       (3x3, 转动块, body 系)
 *   H6 = Σ j jᵀ,  j = h_x[i,0:6]  (6x6, 完整 pose geometric Hessian, 仅保存)
 * 理论口径: 该矩阵只反映当前 scan-to-map 匹配提供的几何约束,
 * 不是滤波器完整信息状态 (不含 measurement noise / state covariance)。
 ***********************************************************************/
bool   degeneracy_enable = false;
bool   degeneracy_debug_iterations = false;
double degeneracy_thresh_t = 0.02;   // Ht 平动块 ratio 阈值
double degeneracy_thresh_r = 0.02;   // Hr 转动块 ratio 阈值
double degeneracy_ema_alpha = 0.5;   // score EMA 系数
double degeneracy_score_enter = 0.4; // 进入退化状态的 EMA score 阈值
double degeneracy_score_exit  = 0.2; // 退出退化状态的 EMA score 阈值
int    degeneracy_min_enter = 3;     // 进入退化所需连续帧数
int    degeneracy_min_exit  = 5;     // 退出退化所需连续帧数

/*** ==================== LIO Health Monitor (P2) ====================
 * 在 P1 退化检测之上聚合 P2 输入信号:
 *   - geometry / matching (来自 DegeneracyData)
 *   - filter covariance (kf.get_P(), DOF: pos0-2 rot3-5 ... bg15-17 ba18-20)
 *   - IMU excitation (rolling buffer, health.imu_window_sec)
 * 发布 /lio/health (degeneracy.enable && health.enable 门控, 默认不发布)。
 ***********************************************************************/
bool   health_enable = false;
double health_imu_window_sec = 0.5;  // rolling IMU 统计窗口 (s)
rclcpp::Publisher<lio_interfaces::msg::LioHealth>::SharedPtr health_pub_;

/*** ==================== P3 Adaptive LiDAR Weighting ====================
 * 机制: 把编译期常量 LASER_POINT_COV 换成运行时变量 lidar_meas_cov_:
 *   lidar_meas_cov_ = LASER_POINT_COV * applied_scale (attack/release EMA)。
 * 控制信号 (D1, 见 report/P3):
 *   geometry = P1 ema_score 连续值 → mild (max_geom_scale, 默认 2~10);
 *   matching = P2 matching severity (attribution_rules.py 复刻) → aggressive
 *              (max_match_scale, 默认 20~100)。
 * 保护 (评审 R20/R22):
 *   下限恒为 1.0 (绝不让 R 小于 baseline); 上限 = max_*_scale;
 *   fast attack / slow release; 连续高 scale 监测告警 (防正反馈循环)。
 * 局限 (R3/R9): one-frame delayed feedback controller —— 本帧 IEKF 使用上一帧
 *   收敛后 degen_data 算出的 scale; 对持续性退化有效, 对 first-frame abrupt
 *   fault (单帧突发 outlier) 不具备提前保护能力。
 * 参数来源: 仅 mapping.launch.py launch dict (与 health 同机制, 单一事实源,
 *   不写 mid360.yaml 防"文档配置"分叉)。默认全关 = 零侵入。
 ***********************************************************************/
bool   adaptive_enable = false;
double adaptive_max_geom_scale = 2.0;    // geometry mild channel 上限
double adaptive_max_match_scale = 50.0;  // matching aggressive channel 上限
double adaptive_attack_alpha = 0.5;      // fast attack (进入降权快)
double adaptive_release_alpha = 0.1;     // slow release (恢复慢, 防权重震荡)
double adaptive_high_scale_warn_th = 3.0;// 连续高 scale 告警阈值 (默认参数下仅 matching
                                         // 通道或大 geometry 配置可能触发; 避免正常退化刷屏)
// P2 matching 阈值 (attribution_params.yaml v1.1 冻结值, 与 attribution_rules.py 一致)
double adaptive_feats_crit_th = 30.0;    // 低于此直接 matching_failure (severity=1)
double adaptive_feats_low_th = 150.0;    // 绝对下限
double adaptive_ratio_low_th = 0.20;     // 有效占比下限
double adaptive_res_high_th = 0.05;      // mean_residual 异常 (m)
double adaptive_res_p90_th = 0.08;       // residual_p90 异常 (m)
double adaptive_ratio_drop_th = 0.30;    // 相对滚动基线占比骤降
double adaptive_num_drop_th = 0.40;      // 相对滚动基线数量骤降
double adaptive_ratio_base_window_s = 30.0;  // rolling baseline 窗口 (P2 v1.1 冻结: 长 fault 段防基线污染)
// P2 geometry 阈值 (P1.3 标定, 冻结不动; severity 复刻 attribution_rules.severity_geometry)
double adaptive_score_geom_th = 0.50;    // score 高于此 → 几何可观测性弱
double adaptive_ratio_t_th = 0.10;       // trans_ratio 低于此 → 平动弱约束 (P1.3 corridor 标定值)
double adaptive_ratio_r_th = 0.02;       // rot_ratio 低于此 → 转动弱约束 (P1 默认)
// runtime state
double lidar_meas_cov_ = LASER_POINT_COV;    // 本帧 IEKF 实际使用的 R
double adaptive_scale_ema_ = 1.0;            // applied scale (EMA)
double adaptive_geom_scale_det_ = 1.0;       // detected: 本帧几何路 scale
double adaptive_match_scale_det_ = 1.0;      // detected: 本帧匹配路 scale
double adaptive_final_scale_det_ = 1.0;      // detected: max(geom, match) (EMA 前)
bool   adaptive_active_det_ = false;         // detected: final_scale > 1+eps
double adaptive_high_scale_since_ = 0.0;     // 进入连续高 scale 的时刻 (lidar 时间)
struct AdaptiveBaselineSample
{
  double t; double ratio; double num;
};
std::deque<AdaptiveBaselineSample> adaptive_base_buf_;

struct ImuHealthSample
{
  double t;
  double ax, ay, az;
  double gx, gy, gz;
};
std::deque<ImuHealthSample> health_imu_buf_;

rclcpp::Publisher<lio_interfaces::msg::DegeneracyScore>::SharedPtr degen_pub_;
rclcpp::Publisher<lio_interfaces::msg::DegeneracyIteration>::SharedPtr degen_iter_pub_;
rclcpp::Clock::SharedPtr degen_clock_;
int degen_frame_index = 0;

struct DegeneracyData
{
  bool valid = false;
  double trans_ratio = 1.0;               // Ht λmin/λmax
  double rot_ratio = 1.0;                 // Hr λmin/λmax
  double normal_concentration = 1.0 / 3.0;// 法向量集中度 μ1/Σμ
  double condition_number = 1.0;          // H6 条件数
  Eigen::VectorXd eigenvalues6 = Eigen::VectorXd::Ones(6);   // H6 特征值(升序)
  Eigen::VectorXd weak_direction = Eigen::VectorXd::Zero(6); // H6 最小特征向量
  int effective_feature_num = 0;
  double mean_residual = 0.0;
  // 平滑状态机 (EMA + hysteresis)
  bool ema_inited = false;
  double ema_score = 0.0;
  bool is_degenerate = false;
  int enter_count = 0;
  int exit_count = 0;
  // debug
  int iter_count = 0;
  // P2 health 扩展
  int candidate_feature_num = 0;          // 候选匹配点数 (feats_down_size)
  double effective_feature_ratio = 1.0;   // effective/candidate
  double residual_p90 = 0.0;              // 有效点残差 90 分位
  Eigen::Vector3d weak_translation_direction = Eigen::Vector3d::UnitZ(); // Ht 最小特征向量 (世界系)
  Eigen::Vector3d weak_rotation_direction = Eigen::Vector3d::UnitZ();    // Hr 最小特征向量 (body 系)
  int weak_mode_index = 0;                // 最小特征值模态下标
  int dominant_weak_axis = 2;             // argmax(|weak_direction|) 状态维 0-5
};
DegeneracyData degen_data;

void compute_geometric_hessian(const esekfom::dyn_share_datastruct<double> &ekfom_data, DegeneracyData &out)
{
  const int m = static_cast<int>(ekfom_data.h_x.rows());
  out.valid = (m >= 1);
  if (!out.valid) return;

  Eigen::Matrix3d Ht = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Hr = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 6, 6> H6 = Eigen::Matrix<double, 6, 6>::Zero();

  for (int i = 0; i < m; i++)
  {
    Eigen::Matrix<double, 1, 12> h = ekfom_data.h_x.row(i);
    Eigen::Vector3d n = h.head<3>().transpose();     // 平动 Jacobian (世界系法向量)
    Eigen::Vector3d a = h.segment<3>(3).transpose(); // 转动 Jacobian (body 系 p×n)
    Ht.noalias() += n * n.transpose();
    Hr.noalias() += a * a.transpose();
    Eigen::Matrix<double, 6, 1> j;
    j << n, a;
    H6.noalias() += j * j.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_t(Ht);
  Eigen::Vector3d wt = es_t.eigenvalues(); // ascending
  out.trans_ratio = (wt(2) > 1e-12) ? wt(0) / wt(2) : 0.0;
  out.normal_concentration = (wt.sum() > 1e-12) ? wt(2) / wt.sum() : 0.0;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_r(Hr);
  Eigen::Vector3d wr = es_r.eigenvalues();
  out.rot_ratio = (wr(2) > 1e-12) ? wr(0) / wr(2) : 0.0;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es6(H6);
  out.eigenvalues6 = es6.eigenvalues();               // ascending
  out.weak_direction = es6.eigenvectors().col(0);     // 最小特征值特征向量
  out.condition_number = (out.eigenvalues6(0) > 1e-12)
    ? out.eigenvalues6(5) / out.eigenvalues6(0)
    : std::numeric_limits<double>::max();

  out.effective_feature_num = m;
  out.mean_residual = (m > 0) ? total_residual / m : 0.0;

  // ── P2 health 扩展 ──
  // 纯平移弱方向 (Ht 最小特征向量, 世界系; 与 GT 位置误差比较前需 R_align)
  out.weak_translation_direction = es_t.eigenvectors().col(0);
  // 纯转动弱方向 (Hr 最小特征向量, body 系)
  out.weak_rotation_direction = es_r.eigenvectors().col(0);
  // 最小特征值模态下标 (SelfAdjointEigenSolver 特征值升序, 显式 argmin 更稳)
  int argmin = 0;
  for (int i = 1; i < 6; i++)
    if (out.eigenvalues6(i) < out.eigenvalues6(argmin)) argmin = i;
  out.weak_mode_index = argmin;
  // 主弱轴: weak_direction 中绝对值最大的状态维 (粗略, 不作严格方向判定)
  double max_c = -1.0;
  int max_i = 2;
  for (int i = 0; i < 6; i++)
  {
    const double c = std::fabs(out.weak_direction(i));
    if (c > max_c) { max_c = c; max_i = i; }
  }
  out.dominant_weak_axis = max_i;
  // matching 相对指标
  out.candidate_feature_num = feats_down_size;
  out.effective_feature_ratio = (feats_down_size > 0)
    ? static_cast<double>(m) / static_cast<double>(feats_down_size) : 0.0;
  // 有效点残差 90 分位 (对局部匹配恶化更敏感)
  if (m > 0)
  {
    const long idx = static_cast<long>(0.9 * static_cast<double>(m - 1));
    std::vector<float> res(res_last, res_last + m);
    std::nth_element(res.begin(), res.begin() + idx, res.end());
    out.residual_p90 = res[idx];
  }
  else { out.residual_p90 = 0.0; }
}

void publish_degeneracy_iteration()
{
  if (!degen_iter_pub_ || !degen_data.valid) return;
  lio_interfaces::msg::DegeneracyIteration msg;
  msg.header.stamp = get_ros_time(lidar_end_time);
  msg.iteration = degen_data.iter_count;
  msg.frame_index = degen_frame_index;
  msg.total_residual = total_residual;
  msg.mean_residual = degen_data.mean_residual;
  msg.effective_feature_num = degen_data.effective_feature_num;
  msg.trans_ratio = degen_data.trans_ratio;
  msg.rot_ratio = degen_data.rot_ratio;
  msg.lambda_max = degen_data.eigenvalues6(5);
  msg.lambda_min = degen_data.eigenvalues6(0);
  degen_iter_pub_->publish(msg);
}

void publish_degeneracy_score()
{
  if (!degen_pub_ || !degen_data.valid) return;

  // raw score: 0 = 健康(ratio>=阈值), 越接近 1 = 越退化(ratio→0)
  double raw = std::max(
    1.0 - degen_data.trans_ratio / degeneracy_thresh_t,
    1.0 - degen_data.rot_ratio   / degeneracy_thresh_r);
  raw = std::min(1.0, std::max(0.0, raw));

  // EMA 平滑
  if (!degen_data.ema_inited)
  {
    degen_data.ema_score = raw;
    degen_data.ema_inited = true;
  }
  else
  {
    degen_data.ema_score =
      degeneracy_ema_alpha * raw + (1.0 - degeneracy_ema_alpha) * degen_data.ema_score;
  }

  // hysteresis 状态机 (进入/退出采用不同阈值 + 连续帧确认)
  if (!degen_data.is_degenerate)
  {
    if (degen_data.ema_score > degeneracy_score_enter)
    {
      if (++degen_data.enter_count >= degeneracy_min_enter)
      {
        degen_data.is_degenerate = true;
        degen_data.enter_count = 0;
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("laser_mapping"), *degen_clock_, 5000,
          "[Degeneracy] ENTER: score=%.3f trans_ratio=%.4f rot_ratio=%.4f feats=%d",
          degen_data.ema_score, degen_data.trans_ratio, degen_data.rot_ratio,
          degen_data.effective_feature_num);
      }
    }
    else { degen_data.enter_count = 0; }
  }
  else
  {
    if (degen_data.ema_score < degeneracy_score_exit)
    {
      if (++degen_data.exit_count >= degeneracy_min_exit)
      {
        degen_data.is_degenerate = false;
        degen_data.exit_count = 0;
      }
    }
    else { degen_data.exit_count = 0; }
  }

  lio_interfaces::msg::DegeneracyScore msg;
  msg.header.stamp = get_ros_time(lidar_end_time);
  msg.score = degen_data.ema_score;
  msg.is_degenerate = degen_data.is_degenerate;
  msg.trans_ratio = degen_data.trans_ratio;
  msg.rot_ratio = degen_data.rot_ratio;
  msg.condition_number = degen_data.condition_number;
  msg.normal_concentration = degen_data.normal_concentration;
  for (int i = 0; i < 6; i++)
  {
    msg.eigenvalues[i] = degen_data.eigenvalues6(i);
    msg.weak_direction[i] = degen_data.weak_direction(i);
  }
  msg.effective_feature_num = degen_data.effective_feature_num;
  msg.mean_residual = degen_data.mean_residual;
  msg.pos_x = state_point.pos(0);
  msg.pos_y = state_point.pos(1);
  msg.pos_z = state_point.pos(2);
  degen_pub_->publish(msg);
}

/*** P2: rolling IMU excitation 统计 (health_imu_buf_, 窗口 health.imu_window_sec)
 *  - gyr_exc  = RMS(|ω|)                        (转动激励)
 *  - acc_exc  = sqrt(var_ax+var_ay+var_az)      (平动动态激励, 三轴去均值后合成;
 *              静止≈0, 匀速≈0, 加减速/转向时明显升高 —— 比 |a|-g 更稳定)
 *  - 另输出三轴 RMS / 方差供离线分析。
 ***/
void compute_imu_excitation(unsigned int &sample_num, double gyro_rms[3],
    double gyro_var[3], double acc_var[3], double &gyr_exc, double &acc_exc)
{
  sample_num = 0;
  gyr_exc = 0.0;
  acc_exc = 0.0;
  for (int i = 0; i < 3; i++) { gyro_rms[i] = 0.0; gyro_var[i] = 0.0; acc_var[i] = 0.0; }
  if (health_imu_buf_.empty()) return;

  const double now_t = health_imu_buf_.back().t;
  while (!health_imu_buf_.empty() &&
         now_t - health_imu_buf_.front().t > health_imu_window_sec)
    health_imu_buf_.pop_front();

  const int n = static_cast<int>(health_imu_buf_.size());
  if (n < 1) return;

  double mgx = 0, mgy = 0, mgz = 0, max_ = 0, may = 0, maz = 0;
  for (const auto &s : health_imu_buf_)
  {
    mgx += s.gx; mgy += s.gy; mgz += s.gz;
    max_ += s.ax; may += s.ay; maz += s.az;
  }
  mgx /= n; mgy /= n; mgz /= n; max_ /= n; may /= n; maz /= n;

  double vgx = 0, vgy = 0, vgz = 0, vax = 0, vay = 0, vaz = 0, gyr_mag2 = 0;
  for (const auto &s : health_imu_buf_)
  {
    const double dgx = s.gx - mgx, dgy = s.gy - mgy, dgz = s.gz - mgz;
    vgx += dgx * dgx; vgy += dgy * dgy; vgz += dgz * dgz;
    const double dax = s.ax - max_, day = s.ay - may, daz = s.az - maz;
    vax += dax * dax; vay += day * day; vaz += daz * daz;
    gyr_mag2 += s.gx * s.gx + s.gy * s.gy + s.gz * s.gz;
  }
  vgx /= n; vgy /= n; vgz /= n; vax /= n; vay /= n; vaz /= n;

  sample_num = static_cast<unsigned int>(n);
  gyro_var[0] = vgx; gyro_var[1] = vgy; gyro_var[2] = vgz;
  acc_var[0] = vax;  acc_var[1] = vay;  acc_var[2] = vaz;
  gyro_rms[0] = std::sqrt(mgx * mgx + vgx);
  gyro_rms[1] = std::sqrt(mgy * mgy + vgy);
  gyro_rms[2] = std::sqrt(mgz * mgz + vgz);
  gyr_exc = std::sqrt(gyr_mag2 / n);
  acc_exc = std::sqrt(vax + vay + vaz);
}

void publish_lio_health()
{
  if (!health_pub_ || !degen_data.valid) return;

  lio_interfaces::msg::LioHealth msg;
  msg.header.stamp = get_ros_time(lidar_end_time);
  msg.header.frame_id = "camera_init";

  // ── geometry ──
  msg.score = degen_data.ema_score;
  msg.is_degenerate = degen_data.is_degenerate;
  msg.trans_ratio = degen_data.trans_ratio;
  msg.rot_ratio = degen_data.rot_ratio;
  msg.condition_number = degen_data.condition_number;
  msg.normal_concentration = degen_data.normal_concentration;
  for (int i = 0; i < 6; i++)
  {
    msg.eigenvalues[i] = degen_data.eigenvalues6(i);
    msg.weak_direction[i] = degen_data.weak_direction(i);
  }
  msg.weak_mode_index = static_cast<uint8_t>(degen_data.weak_mode_index);
  msg.dominant_weak_axis = static_cast<uint8_t>(degen_data.dominant_weak_axis);
  for (int i = 0; i < 3; i++)
  {
    msg.weak_translation_direction[i] = degen_data.weak_translation_direction(i);
    msg.weak_rotation_direction[i] = degen_data.weak_rotation_direction(i);
  }

  // ── matching ──
  msg.effective_feature_num = static_cast<uint32_t>(degen_data.effective_feature_num);
  msg.candidate_feature_num = static_cast<uint32_t>(degen_data.candidate_feature_num);
  msg.effective_feature_ratio = degen_data.effective_feature_ratio;
  msg.mean_residual = degen_data.mean_residual;
  msg.residual_p90 = degen_data.residual_p90;
  msg.total_residual = total_residual;

  // ── filter covariance (const ref, 无拷贝) ──
  const auto &P = kf.get_P();   // 23x23
  msg.pos_cov_trace = P(0, 0) + P(1, 1) + P(2, 2);
  msg.rot_cov_trace = P(3, 3) + P(4, 4) + P(5, 5);
  for (int i = 0; i < 3; i++)
  {
    msg.pose_cov_diag[i] = P(i, i);
    msg.pose_cov_diag[3 + i] = P(3 + i, 3 + i);
    msg.bg_cov_diag[i] = P(15 + i, 15 + i);
    msg.ba_cov_diag[i] = P(18 + i, 18 + i);
  }
  msg.bg_cov_trace = P(15, 15) + P(16, 16) + P(17, 17);
  msg.ba_cov_trace = P(18, 18) + P(19, 19) + P(20, 20);

  // ── IMU excitation ──
  unsigned int sample_num = 0;
  double gyro_rms[3] = {0.0};
  double gyro_var[3] = {0.0};
  double acc_var[3] = {0.0};
  double gyr_exc = 0.0, acc_exc = 0.0;
  compute_imu_excitation(sample_num, gyro_rms, gyro_var, acc_var, gyr_exc, acc_exc);
  msg.imu_sample_num = sample_num;
  for (int i = 0; i < 3; i++)
  {
    msg.imu_gyr_rms[i] = gyro_rms[i];
    msg.imu_gyr_var[i] = gyro_var[i];
    msg.imu_acc_var[i] = acc_var[i];
  }
  msg.imu_gyr_excitation = gyr_exc;
  msg.imu_acc_excitation = acc_exc;

  // ── P3 Adaptive LiDAR Weighting (detected scales 本帧; lidar_meas_cov applied) ──
  msg.lidar_meas_cov = lidar_meas_cov_;
  msg.geometry_scale = adaptive_geom_scale_det_;
  msg.matching_scale = adaptive_match_scale_det_;
  msg.final_scale = adaptive_final_scale_det_;
  msg.adaptive_active = adaptive_active_det_;

  msg.pos_x = state_point.pos(0);
  msg.pos_y = state_point.pos(1);
  msg.pos_z = state_point.pos(2);

  health_pub_->publish(msg);
}

/*** P3: Adaptive LiDAR Weighting helper (C++ 版, matching severity 复刻 P2) ***/

// margin helpers (与 attribution_rules.py margin_* 一致)
static inline double adaptive_clip01(double x)
{
  return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
}
static inline double adaptive_margin_below(double v, double th, double sat)
{
  return sat > 0.0 ? adaptive_clip01((th - v) / sat) : 0.0;
}
static inline double adaptive_margin_above(double v, double th, double sat)
{
  return sat > 0.0 ? adaptive_clip01((v - th) / sat) : 0.0;
}

// P2 matching severity [0,1] —— 与 attribution_rules.classify 同公式同阈值 (parity 目标)。
double compute_adaptive_matching_severity()
{
  const double eff_num = static_cast<double>(degen_data.effective_feature_num);
  const double cand    = static_cast<double>(degen_data.candidate_feature_num);
  double ratio = degen_data.effective_feature_ratio;
  if (ratio <= 0.0 && eff_num > 0.0 && cand > 0.0)
    ratio = eff_num / cand;
  const double mean_res = degen_data.mean_residual;
  const double res_p90  = degen_data.residual_p90;

  // rolling baseline (窗口内中位数, 与 attribution_node.py 的 _ratio_hist 一致)
  const double now = lidar_end_time;
  adaptive_base_buf_.push_back({now, ratio, eff_num});
  while (!adaptive_base_buf_.empty() &&
         now - adaptive_base_buf_.front().t > adaptive_ratio_base_window_s)
    adaptive_base_buf_.pop_front();

  double ratio_base = 0.0, num_base = 0.0;
  const bool have_base = (adaptive_base_buf_.size() >= 5);
  if (have_base)
  {
    std::vector<double> rv, nv;
    rv.reserve(adaptive_base_buf_.size());
    nv.reserve(adaptive_base_buf_.size());
    for (const auto &s : adaptive_base_buf_) { rv.push_back(s.ratio); nv.push_back(s.num); }
    const size_t mid = rv.size() / 2;
    std::nth_element(rv.begin(), rv.begin() + static_cast<std::ptrdiff_t>(mid), rv.end());
    std::nth_element(nv.begin(), nv.begin() + static_cast<std::ptrdiff_t>(mid), nv.end());
    ratio_base = rv[mid];
    num_base = nv[mid];
  }

  const bool low_ratio = ratio < adaptive_ratio_low_th;
  const bool low_feats = eff_num < adaptive_feats_low_th;
  const bool crit_feats = eff_num < adaptive_feats_crit_th;
  const bool high_mean = mean_res > adaptive_res_high_th;
  const bool high_p90 = res_p90 > adaptive_res_p90_th;

  double rel_ratio = -1.0, rel_num = -1.0;
  if (have_base && ratio_base > 1e-6) rel_ratio = ratio / ratio_base;
  if (have_base && num_base > 1e-6)   rel_num   = eff_num / num_base;
  const bool rel_drop = (rel_ratio >= 0.0 && rel_ratio < adaptive_ratio_drop_th) ||
                        (rel_num   >= 0.0 && rel_num   < adaptive_num_drop_th);

  // is_matching_failure 同 P2 (仅逻辑一致; severity 独立计算):
  //   low_feats || crit_feats || high_p90 || rel_drop || (low_ratio && high_mean)

  double s = 0.0;
  s = std::max(s, adaptive_margin_below(ratio,    adaptive_ratio_low_th, adaptive_ratio_low_th));
  s = std::max(s, adaptive_margin_below(eff_num,  adaptive_feats_low_th, adaptive_feats_low_th));
  s = std::max(s, adaptive_margin_above(mean_res, adaptive_res_high_th, adaptive_res_high_th));
  s = std::max(s, adaptive_margin_above(res_p90,  adaptive_res_p90_th,  adaptive_res_p90_th));
  if (rel_ratio >= 0.0)
    s = std::max(s, adaptive_margin_below(rel_ratio, adaptive_ratio_drop_th, adaptive_ratio_drop_th));
  if (rel_num >= 0.0)
    s = std::max(s, adaptive_margin_below(rel_num,   adaptive_num_drop_th,  adaptive_num_drop_th));
  if (crit_feats) s = 1.0;
  return adaptive_clip01(s);
}

// geometry severity [0,1] —— 复刻 attribution_rules.severity_geometry (score/trans/rot margin 组合)。
// 与 matching channel 分开: geometry 是 mild (max_geom_scale), matching 是 aggressive。
double compute_adaptive_geometry_severity()
{
  double s = 0.0;
  s = std::max(s, adaptive_margin_above(degen_data.ema_score,   adaptive_score_geom_th, adaptive_score_geom_th));
  s = std::max(s, adaptive_margin_below(degen_data.trans_ratio, adaptive_ratio_t_th,    adaptive_ratio_t_th));
  s = std::max(s, adaptive_margin_below(degen_data.rot_ratio,   adaptive_ratio_r_th,    adaptive_ratio_r_th));
  return adaptive_clip01(s);
}

// 每帧 IEKF update 收敛后调用: 用本帧 degen_data 计算 detected scales (发布用)。
void compute_adaptive_detected_scales()
{
  if (!adaptive_enable) return;
  const double g = compute_adaptive_geometry_severity();
  const double m = compute_adaptive_matching_severity();
  adaptive_geom_scale_det_  = 1.0 + (adaptive_max_geom_scale  - 1.0) * g;
  adaptive_match_scale_det_ = 1.0 + (adaptive_max_match_scale - 1.0) * m;
  adaptive_final_scale_det_ = std::max(adaptive_geom_scale_det_, adaptive_match_scale_det_);
  adaptive_active_det_ = adaptive_final_scale_det_ > 1.0 + 1e-6;
}

// 每帧 IEKF update 前调用: 用上一帧 detected final_scale 更新 applied EMA 并写 lidar_meas_cov_。
// one-frame delayed feedback: target 是上一帧 compute_adaptive_detected_scales() 的结果。
void apply_adaptive_lidar_cov()
{
  if (!adaptive_enable) { lidar_meas_cov_ = LASER_POINT_COV; return; }
  const double target = adaptive_final_scale_det_;  // 上一帧 detected (滞后一帧)
  const double alpha  = (target > adaptive_scale_ema_) ? adaptive_attack_alpha
                                                       : adaptive_release_alpha;
  adaptive_scale_ema_ += alpha * (target - adaptive_scale_ema_);
  const double cap = std::max(adaptive_max_geom_scale, adaptive_max_match_scale);
  if (adaptive_scale_ema_ < 1.0) adaptive_scale_ema_ = 1.0;  // 下限保护: 不弱于 baseline
  if (adaptive_scale_ema_ > cap) adaptive_scale_ema_ = cap;  // 上限保护
  lidar_meas_cov_ = LASER_POINT_COV * adaptive_scale_ema_;

  // 正反馈监测: 连续高 scale 超过 ~5s 告警 (防"匹配差→降权→更漂→更差"循环)
  const bool high = adaptive_scale_ema_ > adaptive_high_scale_warn_th;
  const double now = lidar_end_time;
  if (high)
  {
    if (adaptive_high_scale_since_ <= 0.0) adaptive_high_scale_since_ = now;
    if (now - adaptive_high_scale_since_ > 5.0)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("laser_mapping"), *degen_clock_, 5000,
        "[Adaptive] WARNING: lidar weight scale %.2fx sustained >5s "
        "(possible feedback loop: matching↓→downweight→drift→matching↓). "
        "geom_det=%.2f match_det=%.2f", adaptive_scale_ema_,
        adaptive_geom_scale_det_, adaptive_match_scale_det_);
    }
  }
  else adaptive_high_scale_since_ = 0.0;
}

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    // P2: 追加到 health rolling IMU buffer (窗口内样本统计 excitation)
    ImuHealthSample hs;
    hs.t = timestamp;
    hs.ax = msg->linear_acceleration.x;
    hs.ay = msg->linear_acceleration.y;
    hs.az = msg->linear_acceleration.z;
    hs.gx = msg->angular_velocity.x;
    hs.gy = msg->angular_velocity.y;
    hs.gz = msg->angular_velocity.z;
    health_imu_buf_.push_back(hs);
    // 内存有界: 只保留最近 ~4× 窗口
    while (health_imu_buf_.size() > 2 &&
           timestamp - health_imu_buf_.front().t > 4.0 * health_imu_window_sec)
      health_imu_buf_.pop_front();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.child_frame_id = "body";
    trans.header.stamp = get_ros_time(lidar_end_time);
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

    /*** Degeneracy Detection (P1): LiDAR geometric Hessian from real IEKF Jacobians ***/
    if (degeneracy_enable)
    {
        degen_data.iter_count++;
        compute_geometric_hessian(ekfom_data, degen_data);
        if (degeneracy_debug_iterations)
        {
            publish_degeneracy_iteration();
        }
    }

    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<bool>("degeneracy.enable", false);
        this->declare_parameter<bool>("degeneracy.debug_iterations", false);
        this->declare_parameter<double>("degeneracy.threshold_t", 0.02);
        this->declare_parameter<double>("degeneracy.threshold_r", 0.02);
        this->declare_parameter<double>("degeneracy.ema_alpha", 0.5);
        this->declare_parameter<double>("degeneracy.score_enter", 0.4);
        this->declare_parameter<double>("degeneracy.score_exit", 0.2);
        this->declare_parameter<int>("degeneracy.min_enter_frames", 3);
        this->declare_parameter<int>("degeneracy.min_exit_frames", 5);

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        this->get_parameter_or<bool>("degeneracy.enable", degeneracy_enable, false);
        this->get_parameter_or<bool>("degeneracy.debug_iterations", degeneracy_debug_iterations, false);
        this->get_parameter_or<double>("degeneracy.threshold_t", degeneracy_thresh_t, 0.02);
        this->get_parameter_or<double>("degeneracy.threshold_r", degeneracy_thresh_r, 0.02);
        this->get_parameter_or<double>("degeneracy.ema_alpha", degeneracy_ema_alpha, 0.5);
        this->get_parameter_or<double>("degeneracy.score_enter", degeneracy_score_enter, 0.4);
        this->get_parameter_or<double>("degeneracy.score_exit", degeneracy_score_exit, 0.2);
        this->get_parameter_or<int>("degeneracy.min_enter_frames", degeneracy_min_enter, 3);
        this->get_parameter_or<int>("degeneracy.min_exit_frames", degeneracy_min_exit, 5);
        // P2 health 参数: 经 launch 参数 dict 传入 (yaml 嵌套块解析有缺陷)。
        // 显式 declare 为 bool/double, 使 launch 传入的字符串 override 可正确类型转换。
        this->declare_parameter<bool>("health.enable", health_enable);
        this->declare_parameter<double>("health.imu_window_sec", health_imu_window_sec);
        this->get_parameter_or<bool>("health.enable", health_enable, false);
        this->get_parameter_or<double>("health.imu_window_sec", health_imu_window_sec, 0.5);
        // P3 adaptive 参数: 经 launch 参数 dict 传入 (唯一参数源, 不写 mid360.yaml)。
        // 显式 declare 为 bool/double, 使 launch 传入的字符串 override 可正确类型转换。
        this->declare_parameter<bool>("adaptive.enable", adaptive_enable);
        this->declare_parameter<double>("adaptive.max_geom_scale", adaptive_max_geom_scale);
        this->declare_parameter<double>("adaptive.max_match_scale", adaptive_max_match_scale);
        this->declare_parameter<double>("adaptive.attack_alpha", adaptive_attack_alpha);
        this->declare_parameter<double>("adaptive.release_alpha", adaptive_release_alpha);
        this->declare_parameter<double>("adaptive.high_scale_warn_th", adaptive_high_scale_warn_th);
        this->get_parameter_or<bool>("adaptive.enable", adaptive_enable, false);
        this->get_parameter_or<double>("adaptive.max_geom_scale", adaptive_max_geom_scale, 2.0);
        this->get_parameter_or<double>("adaptive.max_match_scale", adaptive_max_match_scale, 50.0);
        this->get_parameter_or<double>("adaptive.attack_alpha", adaptive_attack_alpha, 0.5);
        this->get_parameter_or<double>("adaptive.release_alpha", adaptive_release_alpha, 0.1);
        this->get_parameter_or<double>("adaptive.high_scale_warn_th", adaptive_high_scale_warn_th, 3.0);
        if (adaptive_enable && !degeneracy_enable)
        {
            RCLCPP_WARN(this->get_logger(),
                "adaptive.enable=true 但 degeneracy.enable=false: adaptive 降权不会生效 "
                "(matching/geometry 信号来自 degeneracy 门控内的 degen_data)");
        }

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="camera_init";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        degen_pub_ = this->create_publisher<lio_interfaces::msg::DegeneracyScore>("/lio/degeneracy_score", 20);
        degen_iter_pub_ = this->create_publisher<lio_interfaces::msg::DegeneracyIteration>("/lio/degeneracy_iterations", 40);
        degen_clock_ = this->get_clock();
        health_pub_ = this->create_publisher<lio_interfaces::msg::LioHealth>("/lio/health", 20);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
    void timer_callback()
    {
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            degen_data.iter_count = 0;
            // P3: 用上一帧 detected final_scale 更新本帧 applied covariance (one-frame delay)
            apply_adaptive_lidar_cov();
            kf.update_iterated_dyn_share_modified(lidar_meas_cov_, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /*** Publish degeneracy score (once per LiDAR measurement update) ***/
            if (degeneracy_enable)
            {
                degen_frame_index = frame_num;
                publish_degeneracy_score();
            }

            /*** P3: 用本帧 degen_data 计算 detected scales (发布用, one-frame delay 的"检测"端) ***/
            if (degeneracy_enable && adaptive_enable)
            {
                compute_adaptive_detected_scales();
            }

            /*** Publish LIO health (P2, same timestamp as degen score) ***/
            if (degeneracy_enable && health_enable)
            {
                publish_lio_health();
            }

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
