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
double adaptive_max_downweight_duration_s = 3.0;  // 连续降权时长上限: 超时强制回 baseline
                                         // (打破"降权→漂移→失配→误判匹配失败→继续降权"
                                         //  的正反馈循环, 评审 R22 的直接实现)
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
double adaptive_downweight_since_ = 0.0;     // 进入降权(scale>1.1)的时刻, 用于时长上限
double adaptive_warn_since_ = 0.0;           // 进入高 scale(>warn_th)的时刻, 用于告警
struct AdaptiveBaselineSample
{
  double t; double ratio; double num;
};
std::deque<AdaptiveBaselineSample> adaptive_base_buf_;

/*** ==================== P4 Attribution-aware Directional Update ====================
 * 机制: 在 esekfom::update_iterated_dyn_share_modified 的 state correction 层做
 *   directional projector —— 本帧 IEKF 对 pos(世界系) 沿 weak translation direction
 *   的 correction 保留 beta_t 倍, 强方向不动; K_x 的 pos 行同步投影保持 covariance 一致
 *   (P4 评审 R1/R3/R4/R8; sanity E2/E3 数值验证)。
 * 触发 (R14/R15/R16): 仅当 translation geometry 退化 (trans_ratio < ratio_t_th)
 *   且 P2 matching gate 通过 (is_matching_failure == false) 时允许。matching failure 时
 *   关闭 projector —— 错误 correspondence 构造的 weak direction 无物理意义。
 * 信号路径 (R6/R7): P1 检测器 (compute_geometric_hessian) 永远基于原始 h_x;
 *   P4 控制器只作用在 esekfom state 层, 两者数据路径严格分离, 无自反馈污染。
 * 时序: one-frame delay —— 本帧 IEKF 用上一帧 update 后 dir_cache_ 的方向/ratio。
 * v1 范围: 仅平动 (beta_r 恒 1)。未来转动抑制时, body 系弱方向需 R_prev->R_cur 变换
 *   (见 esekfom.hpp 注释, R10)。参数唯一来源 = mapping.launch.py launch dict。
 * 与 P3 关系: --adaptive 与 --directional 正式实验互斥 (R29); 本块不依赖 adaptive_enable。
 ***********************************************************************/
bool directional_enable = false;
double directional_beta_t = 0.5;         // 平动弱方向 correction 保留系数 (0=完全抑制, 1=不抑制)
double directional_beta_r = 1.0;         // v1 恒 1: 不抑制转动 (保留接口, R10 见 esekfom 注释)
bool directional_ht_accum_enable = false;// Ht 时间累积独立开关 (R13: 不用 0 值承担"关闭"语义)
double directional_ht_accum_alpha = 0.5; // Ht_cum = a*Ht + (1-a)*Ht_cum, 再特征分解 → 稳定弱方向
bool directional_imu_safety_limiter_enable = false;  // v1 仅记录/接口预留 (R17)
double directional_imu_beta_floor = 0.5; // IMU 低激励时的 beta 下限 (安全联动, 默认不启用)
// P2 冻结阈值 (attribution_params.yaml v1.1, 与 attribution_rules.py / P3 matching 同源)
double directional_ratio_t_th = 0.10;    // trans_ratio 低于此 → translation 退化 (P1.3 corridor 标定)
double directional_gyr_exc_th = 0.03;    // imu_gyr_excitation 低于此 → 转动激励不足
double directional_acc_exc_th = 0.30;    // imu_acc_excitation 低于此 → 平动激励不足
// P4 rolling baseline (P2 matching 相对骤降判定; 独立 buffer, 与 P3 adaptive 数据路径分离)
std::deque<AdaptiveBaselineSample> directional_base_buf_;

struct DirectionalCache
{
  bool valid = false;
  double trans_ratio = 1.0;              // 上帧 Ht ratio (world-frame translation)
  bool matching_ok = true;               // 上帧 P2 is_matching_failure == false
  Eigen::Vector3d weak_translation_direction = Eigen::Vector3d::UnitX(); // 上帧 Ht 最小特征向量 (世界系, 归一化)
  bool imu_low_excitation = false;       // 上帧 is_imu_low_excitation (gyr_weak && acc_weak)
};
DirectionalCache dir_cache_;
// P4 发布状态 (本帧 IEKF 实际生效; msg 输出用于事后归因 R26)
bool dir_active_ = false;
double dir_beta_applied_ = 1.0;
uint8_t dir_reason_ = 0;                 // 0=inactive 1=translation_degen 2=matching_gate_blocked 3=imu_safety_limited
bool dir_matching_gate_passed_ = true;
double dir_translation_severity_ = 0.0;  // 1 - trans_ratio/threshold, clamp[0,1] (本帧 detected)
Eigen::Matrix3d ht_cum_ = Eigen::Matrix3d::Zero();
bool ht_cum_inited_ = false;

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
  // P4: 原始 Ht 矩阵 (供 Ht 时间累积 → 稳定弱方向; 检测器始终基于原始 h_x, 与 P4 控制器数据路径分离)
  Eigen::Matrix3d Ht = Eigen::Matrix3d::Zero();
};
DegeneracyData degen_data;

/*** ==================== P5 Current-frame Measurement Gate ====================
 * 机制 (评审 R1-R3/R6-R9): 在 esekfom::update_iterated_dyn_share_modified 第一次
 *   h_dyn_share() 返回后 (KD-tree correspondence 完成, degen_data 已填充
 *   first-iteration 统计), 由 callback 判定本帧 LiDAR measurement 是否可信:
 *     - 可信  → 正常执行 IEKF measurement correction;
 *     - 不可信 → 显式恢复 propagated state/P (esekfom 内完成), 本帧只有 IMU
 *               propagation; laserMapping 侧同时跳过 map_incremental() (评审 R6/R7:
 *               拒绝帧的预测位姿点云不得污染 ikd-Tree, 否则下一帧在污染地图上匹配)。
 * 判定通道 (P5.0 冻结): P2 final-iteration 的 absolute 阈值 (feats_low/residual)
 *   在 first-iteration 上不可用 (normal_indoor 干净场景 p90 也超阈值; fault 段
 *   first-iter residual 反而低)。P5 gate 使用 rolling-baseline relative 通道:
 *     reject = (rel_num < num_drop_th) OR (rel_ratio < ratio_drop_th)
 *              OR (rel_p90 > p90_rise_th)      # residual_p90 相对基线飙升
 *   geometry 防护 (评审 #13): 不单独用点数绝对阈值; single_plane 的 rel_num/rel_ratio
 *   均稳定 (P5.0 验证 0 触发), 天然不误杀 geometry 退化。
 * Recovery state machine (评审 #8-#12): NORMAL → REJECT_ONCE → DEGRADED(2帧) →
 *   RECOVERY_REQUIRED(5帧); 每帧始终重新尝试 matching (不降门槛), 恢复用 hysteresis
 *   (recover 阈值放宽 + 连续确认帧), 拒绝≠停止匹配。
 * 参数来源: 仅 mapping.launch.py launch dict; 默认全关 = 零侵入。
 ***********************************************************************/
bool   robust_gate_enable = false;
double gate_num_drop_th = 0.40;    // rel_num 低于此 → 有效特征数骤降 (P2 v1.1 冻结值)
double gate_ratio_drop_th = 0.30;  // rel_ratio 低于此 → 有效占比骤降 (P2 v1.1 冻结值)
double gate_p90_rise_th = 2.0;     // residual_p90 相对基线 > 此倍数 → 残差突变 (P5.0)
double gate_base_window_s = 30.0;  // rolling baseline 窗口 (s, P2 v1.1 冻结)
double gate_recover_num_th = 0.55; // hysteresis: recover 用更宽松的 rel_num 阈值
double gate_recover_ratio_th = 0.45;// hysteresis: recover rel_ratio 阈值
double gate_recover_p90_th = 1.5;  // hysteresis: recover p90 相对倍数
int    gate_degraded_after = 2;    // 连续拒绝 ≥2 → DEGRADED
int    gate_recovery_required_after = 5;  // 连续拒绝 ≥5 → RECOVERY_REQUIRED
int    gate_recover_confirm_frames = 2;   // 恢复需连续满足的帧数

// P5 recovery state machine (发布语义, 0=NORMAL 1=REJECT_ONCE 2=DEGRADED 3=RECOVERY_REQUIRED)
enum class P5GateState : uint8_t { NORMAL = 0, REJECT_ONCE = 1, DEGRADED = 2, RECOVERY_REQUIRED = 3 };
P5GateState gate_state_ = P5GateState::NORMAL;
int   gate_consecutive_reject_ = 0;    // 内部 int (saturation 到 INT_MAX, 防溢出回 0)
int   gate_recover_confirm_count_ = 0; // 连续满足 recover 条件的帧数
bool  gate_rejected_ = false;          // 本帧 rejected (out-param 回报主循环)
bool  gate_map_update_skipped_ = false;// 本帧是否跳过 map insertion
bool  gate_recovery_allow_map_ = false;// RECOVERY_REQUIRED 时允许传播位姿插图 (防地图冻结)
double gate_matching_severity_current_ = 0.0;  // 本帧 first-iter relative severity
double gate_rel_num_ = 1.0, gate_rel_ratio_ = 1.0, gate_rel_p90_ = 1.0;
std::deque<AdaptiveBaselineSample> gate_base_buf_;  // 独立 buffer (与 P3/P4 数据路径分离)
bool  gate_baseline_inited_ = false;
double gate_base_median_ratio_ = 1.0, gate_base_median_num_ = 1.0, gate_base_median_p90_ = 0.0;



// P5 gate 判定: 基于当前帧 first-iteration 统计 + rolling baseline。
// 返回值 true = reject (拒绝本帧 measurement)。
static bool robust_gate_reject_check()
{
  // 读 degen_data (当前 = first-iteration, 因 callback 在第一次 h_dyn_share 后调用)
  const double eff = static_cast<double>(degen_data.effective_feature_num);
  const double cand = static_cast<double>(degen_data.candidate_feature_num);
  const double ratio = (cand > 0) ? eff / cand : (eff > 0 ? 1.0 : 0.0);
  const double p90 = degen_data.residual_p90;

  const double now_t = lidar_end_time;
  // 滚动清理 (窗口 gate_base_window_s)
  while (!gate_base_buf_.empty() && now_t - gate_base_buf_.front().t > gate_base_window_s)
    gate_base_buf_.pop_front();

  // 当前帧 relative 信号 (相对 baseline median; baseline 未初始化时用当前帧自身 = no reject)
  double rel_num = 1.0, rel_ratio = 1.0, rel_p90 = 1.0;
  if (gate_baseline_inited_)
  {
    rel_num = (gate_base_median_num_ > 1e-6) ? eff / gate_base_median_num_ : 1.0;
    rel_ratio = (gate_base_median_ratio_ > 1e-6) ? ratio / gate_base_median_ratio_ : 1.0;
    rel_p90 = (gate_base_median_p90_ > 1e-9) ? p90 / gate_base_median_p90_ : 1.0;
  }
  gate_rel_num_ = rel_num; gate_rel_ratio_ = rel_ratio; gate_rel_p90_ = rel_p90;

  // reject 判定 (P5.0: relative 通道 + 残差确认; 不设 absolute 点数阈值 → single_plane 不误杀)
  // 评审 #13 (关键修正): "特征少但残差正常"不是 matching failure (dropout 保留的 20%
  // 点残差仍正常, 可提供有效 correction; 单靠 rel_num 拒绝会不必要地丢帧导致漂移)。
  // 因此数量/占比骤降必须与残差异常 AND 组合才 reject —— 只有"点少且残差差"(outlier/
  // noise 型) 才拒绝。p90 通道同样要求与数量/占比骤降 AND (clean 场景 p90 相对波动
  // 天然存在, parity check 实测单独 p90 会误报)。
  const bool reject_by_num = rel_num < gate_num_drop_th;
  const bool reject_by_ratio = rel_ratio < gate_ratio_drop_th;
  const bool res_abnormal = rel_p90 > gate_p90_rise_th
                         || (p90 > 1e-6 && degen_data.mean_residual > 0.02);
  const bool reject = (reject_by_num || reject_by_ratio) && res_abnormal;

  // severity (relative 口径, 0~1): 取各通道 margin 的 max
  double sev = 0.0;
  if (rel_num < 1.0)   sev = std::max(sev, std::min(1.0, (1.0 - rel_num) / (1.0 - gate_num_drop_th)));
  if (rel_ratio < 1.0) sev = std::max(sev, std::min(1.0, (1.0 - rel_ratio) / (1.0 - gate_ratio_drop_th)));
  if (rel_p90 > 1.0)   sev = std::max(sev, std::min(1.0, (rel_p90 - 1.0) / (gate_p90_rise_th - 1.0)));
  gate_matching_severity_current_ = std::min(1.0, std::max(0.0, sev));

  // 更新 rolling baseline (只用非 reject 帧, 防长 fault 段基线污染; P2 v1.1 同语义)。
  // 注: 曾尝试 "非 reject 且残差不异常" (评审 #13 防 fault 帧污染 median), 但实测
  //   V5 上使 gate 更敏感 → 拒绝更多可用帧 → 漂移 → ATE 从 2.16m 恶化到 10.46m。
  //   权衡后恢复只用非 reject 帧: fault 帧进入 baseline 虽有污染, 但"接受可用帧
  //   保持位姿"比"过早拒绝导致漂移"更重要 (与评审 #9 语义一致: 拒绝越少越好)。
  if (!reject)
  {
    gate_base_buf_.push_back({now_t, ratio, eff});
    std::vector<double> rs, ns;
    for (const auto &s : gate_base_buf_) { rs.push_back(s.ratio); ns.push_back(s.num); }
    if (!rs.empty())
    {
      std::sort(rs.begin(), rs.end()); std::sort(ns.begin(), ns.end());
      gate_base_median_ratio_ = rs[rs.size() / 2];
      gate_base_median_num_ = ns[ns.size() / 2];
      gate_baseline_inited_ = true;
    }
    // p90 baseline: 独立 deque (pair<t,p90>)
    static std::deque<std::pair<double, double>> p90_base;
    p90_base.push_back({now_t, p90});
    while (!p90_base.empty() && now_t - p90_base.front().first > gate_base_window_s)
      p90_base.pop_front();
    std::vector<double> pv;
    for (const auto &s : p90_base) pv.push_back(s.second);
    if (!pv.empty()) { std::sort(pv.begin(), pv.end()); gate_base_median_p90_ = pv[pv.size() / 2]; }
  }
  return reject;
}

// P5 gate callback (esekfom 调用, void* ctx 解耦): 更新 state machine 并返回是否拒绝。
static bool robust_gate_callback(void * /*ctx*/)
{
  const bool reject = robust_gate_reject_check();
  gate_rejected_ = reject;

  if (reject)
  {
    gate_consecutive_reject_ = std::min(gate_consecutive_reject_ + 1, INT_MAX);
    gate_recover_confirm_count_ = 0;
    // 状态转移: 0→REJECT_ONCE, ≥2→DEGRADED, ≥5→RECOVERY_REQUIRED
    if (gate_consecutive_reject_ >= gate_recovery_required_after)
    {
      gate_state_ = P5GateState::RECOVERY_REQUIRED;
      // 评审 #6/#7 + 长 fault 边界 (评审 #10) 的折中:
      //   - 短 fault (REJECT_ONCE/DEGRADED): 拒绝帧不插图, 防污染 (用户 #7 核心诉求);
      //   - RECOVERY_REQUIRED (连续≥5帧): 地图若不更新会"冻结"→ 机器人移动后旧地图
      //     完全失配 → 永远无法恢复 → 纯 IMU 漂移 (实测 V5 165m)。此时允许用传播
      //     位姿插图让地图跟上 (IMU 短时漂移小, 通常 <0.5m/5s), 给下一帧恢复机会。
      gate_recovery_allow_map_ = true;
    }
    else
    {
      gate_state_ = (gate_consecutive_reject_ >= gate_degraded_after)
                  ? P5GateState::DEGRADED : P5GateState::REJECT_ONCE;
      gate_recovery_allow_map_ = false;
    }
  }
  else
  {
    gate_recovery_allow_map_ = false;
    // 恢复判定 (hysteresis: 更宽松阈值 + 连续确认帧; 评审 #12)
    const bool ok_num = gate_rel_num_ > gate_recover_num_th;
    const bool ok_ratio = gate_rel_ratio_ > gate_recover_ratio_th;
    const bool ok_p90 = gate_rel_p90_ < gate_recover_p90_th;
    if (ok_num && ok_ratio && ok_p90)
    {
      if (++gate_recover_confirm_count_ >= gate_recover_confirm_frames)
      {
        gate_consecutive_reject_ = 0;
        gate_state_ = P5GateState::NORMAL;
        gate_recover_confirm_count_ = 0;
      }
    }
    else gate_recover_confirm_count_ = 0;
  }
  return reject;
}

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
  out.Ht = Ht;  // P4: 原始 Ht (检测器数据路径, 绝不被 P4 控制器修改)
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
  msg.candidate_feature_num = degen_data.candidate_feature_num;
  msg.effective_feature_ratio = degen_data.effective_feature_ratio;
  msg.residual_p90 = degen_data.residual_p90;
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

  // ── P4 Directional Update (本帧 applied; severity/gate 本帧 detected) ──
  msg.directional_active = dir_active_;
  msg.beta_applied = dir_beta_applied_;
  for (int i = 0; i < 3; i++)
    msg.suppressed_weak_direction[i] = dir_cache_.valid
      ? dir_cache_.weak_translation_direction(i) : degen_data.weak_translation_direction(i);
  msg.directional_reason = dir_reason_;
  msg.translation_severity = dir_translation_severity_;
  msg.matching_gate_passed = dir_matching_gate_passed_;

  // ── P5 Current-frame Measurement Gate (本帧 first-iteration 判定) ──
  msg.measurement_rejected = gate_rejected_;
  msg.matching_severity_current = gate_matching_severity_current_;
  msg.consecutive_reject_count = static_cast<uint16_t>(
      std::min(gate_consecutive_reject_, static_cast<int>(UINT16_MAX)));
  msg.recovery_state = static_cast<uint8_t>(gate_state_);
  msg.map_update_skipped = gate_map_update_skipped_;

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
// 降权时长上限 (R22 直接实现): 连续降权超过 max_downweight_duration_s 后强制 target=1,
// 让激光重新主导 —— 打破"降权→IMU 漂移→scan-to-map 失配→feats 骤降被误判匹配失败→继续降权"
// 的正反馈循环 (exp_059 clean65-75 段实证: 无 fault 但 match_scale 升到 15.9x)。
void apply_adaptive_lidar_cov()
{
  if (!adaptive_enable) { lidar_meas_cov_ = LASER_POINT_COV; return; }
  const double now = lidar_end_time;
  const double detected = adaptive_final_scale_det_;  // 上一帧 detected (滞后一帧)

  double target = detected;
  const bool downweighting = adaptive_scale_ema_ > 1.1;
  if (downweighting)
  {
    if (adaptive_downweight_since_ <= 0.0) adaptive_downweight_since_ = now;
    if (now - adaptive_downweight_since_ > adaptive_max_downweight_duration_s)
      target = 1.0;  // 超时强制回 baseline (让激光重新主导, 打破正反馈)
  }
  else adaptive_downweight_since_ = 0.0;

  const double alpha = (target > adaptive_scale_ema_) ? adaptive_attack_alpha
                                                       : adaptive_release_alpha;
  adaptive_scale_ema_ += alpha * (target - adaptive_scale_ema_);
  const double cap = std::max(adaptive_max_geom_scale, adaptive_max_match_scale);
  if (adaptive_scale_ema_ < 1.0) adaptive_scale_ema_ = 1.0;  // 下限保护: 不弱于 baseline
  if (adaptive_scale_ema_ > cap) adaptive_scale_ema_ = cap;  // 上限保护
  lidar_meas_cov_ = LASER_POINT_COV * adaptive_scale_ema_;

  // 正反馈监测告警: 连续高 scale 超过 ~5s 提醒
  const bool warn_high = adaptive_scale_ema_ > adaptive_high_scale_warn_th;
  if (warn_high)
  {
    if (adaptive_warn_since_ <= 0.0) adaptive_warn_since_ = now;
    if (now - adaptive_warn_since_ > 5.0)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("laser_mapping"), *degen_clock_, 5000,
        "[Adaptive] WARNING: lidar weight scale %.2fx sustained >5s "
        "(check feedback loop; downweight duration cap=%.1fs forces recovery). "
        "geom_det=%.2f match_det=%.2f", adaptive_scale_ema_,
        adaptive_max_downweight_duration_s,
        adaptive_geom_scale_det_, adaptive_match_scale_det_);
    }
  }
  else adaptive_warn_since_ = 0.0;
}

/*** ==================== P4 helper (Attribution-aware Directional Update) ====================
 * 数据路径分离 (R6/R7): 本组函数只读 degen_data (由 P1 compute_geometric_hessian 用原始
 *   h_x 算出) 与 health IMU 缓冲; 不读、不写任何被 P4 esekfom projector 影响的状态。
 * one-frame delay: update_directional_cache() 在 update 收敛后调用, compute_directional_trigger()
 *   在下一次 update 前调用 → 本帧抑制方向来自上一帧检测, 无同帧自反馈 (R11)。
 ***********************************************************************/

// P2 is_matching_failure 布尔判定 (attribution_rules.classify, 冻结阈值, 独立 rolling baseline)。
// 与 P3 adaptive matching channel 同源但走独立 buffer —— 保证 P4 不依赖 adaptive_enable 开关。
bool compute_directional_matching_gate()
{
  const double eff_num = static_cast<double>(degen_data.effective_feature_num);
  const double cand    = static_cast<double>(degen_data.candidate_feature_num);
  double ratio = degen_data.effective_feature_ratio;
  if (ratio <= 0.0 && eff_num > 0.0 && cand > 0.0)
    ratio = eff_num / cand;
  const double mean_res = degen_data.mean_residual;
  const double res_p90  = degen_data.residual_p90;

  // rolling baseline (30s 窗口中位数; 与 attribution_node 的 ratio_hist 一致)
  const double now = lidar_end_time;
  directional_base_buf_.push_back({now, ratio, eff_num});
  while (!directional_base_buf_.empty() &&
         now - directional_base_buf_.front().t > adaptive_ratio_base_window_s)
    directional_base_buf_.pop_front();

  double ratio_base = 0.0, num_base = 0.0;
  const bool have_base = (directional_base_buf_.size() >= 5);
  if (have_base)
  {
    std::vector<double> rv, nv;
    rv.reserve(directional_base_buf_.size());
    nv.reserve(directional_base_buf_.size());
    for (const auto &s : directional_base_buf_) { rv.push_back(s.ratio); nv.push_back(s.num); }
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

  // P2 冻结版 (attribution_rules.py:114): low_feats 独立触发, 不再要求同时 low_ratio/high_mean。
  return low_feats || crit_feats || high_p90 || rel_drop || (low_ratio && high_mean);
}

// 每帧 IEKF update 收敛后调用: 用本帧 degen_data 填充 dir_cache_ (供下一帧 projector 使用)。
void update_directional_cache()
{
  if (!directional_enable) { dir_cache_.valid = false; return; }
  if (!degen_data.valid) return;

  dir_cache_.valid = true;
  dir_cache_.trans_ratio = degen_data.trans_ratio;
  dir_cache_.matching_ok = !compute_directional_matching_gate();

  // 弱方向: 可选 Ht 时间累积 (R12: 先累积 Ht 再特征分解, 避免 eigenvector sign ambiguity)。
  //         独立开关 ht_accum_enable (R13: 不用 alpha=0 表示"关闭")。
  if (directional_ht_accum_enable)
  {
    if (!ht_cum_inited_) { ht_cum_ = degen_data.Ht; ht_cum_inited_ = true; }
    else
      ht_cum_ = directional_ht_accum_alpha * degen_data.Ht
              + (1.0 - directional_ht_accum_alpha) * ht_cum_;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(ht_cum_);
    dir_cache_.weak_translation_direction = es.eigenvectors().col(0); // 升序最小特征向量
  }
  else
  {
    dir_cache_.weak_translation_direction = degen_data.weak_translation_direction;
  }
  const double nrm = dir_cache_.weak_translation_direction.norm();
  if (nrm < 1e-6) { dir_cache_.weak_translation_direction = Eigen::Vector3d::UnitX(); }
  else            { dir_cache_.weak_translation_direction /= nrm; }

  // IMU 低激励 (P2: gyr_weak && acc_weak, 冻结阈值)。v1 只记录; imu_safety_limiter_enable
  // 时在 trigger 里限制 beta 下限 (R17 接口预留)。
  unsigned int imu_n = 0;
  double grms[3] = {0.0}, gvar[3] = {0.0}, avar[3] = {0.0};
  double gyr_exc = 0.0, acc_exc = 0.0;
  compute_imu_excitation(imu_n, grms, gvar, avar, gyr_exc, acc_exc);
  dir_cache_.imu_low_excitation = (gyr_exc < directional_gyr_exc_th)
                               && (acc_exc < directional_acc_exc_th);
}

// 每帧 IEKF update 前调用: 判定本帧是否应用 directional projector, 输出 (w, beta)。
// 触发 = translation 退化 (trans_ratio < threshold, 独立于 rotation, R14) && matching gate (R15/R16)。
void compute_directional_trigger(double &out_beta, const Eigen::Vector3d *&out_w)
{
  out_beta = 1.0; out_w = nullptr;
  dir_active_ = false; dir_reason_ = 0;
  dir_translation_severity_ = 0.0;
  if (!directional_enable || !dir_cache_.valid) return;

  const double tr = dir_cache_.trans_ratio;
  dir_translation_severity_ = (directional_ratio_t_th > 0.0)
    ? std::min(1.0, std::max(0.0, 1.0 - tr / directional_ratio_t_th)) : 0.0;
  dir_matching_gate_passed_ = dir_cache_.matching_ok;  // msg 始终反映本帧 detected gate

  // R14: translation-only trigger (P1 全局 is_degenerate 受 rotation 影响, 这里不用)
  if (tr >= directional_ratio_t_th) { dir_reason_ = 0; return; }

  // R15/R16: matching gate —— correspondence 不可信时弱方向本身可能失真, 关闭 projector
  if (!dir_cache_.matching_ok) { dir_reason_ = 2; return; }

  // R17: IMU 低激励 safety limiter (v1 可选, 默认记录不联动)
  double beta = directional_beta_t;
  if (directional_imu_safety_limiter_enable && dir_cache_.imu_low_excitation)
  {
    beta = std::max(beta, directional_imu_beta_floor);
    dir_reason_ = 3;
  }
  else dir_reason_ = 1;

  out_w = &dir_cache_.weak_translation_direction;
  out_beta = beta;
  dir_beta_applied_ = beta;
  dir_active_ = true;
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
        this->declare_parameter<double>("adaptive.max_downweight_duration_s",
                                        adaptive_max_downweight_duration_s);
        this->get_parameter_or<bool>("adaptive.enable", adaptive_enable, false);
        this->get_parameter_or<double>("adaptive.max_geom_scale", adaptive_max_geom_scale, 2.0);
        this->get_parameter_or<double>("adaptive.max_match_scale", adaptive_max_match_scale, 50.0);
        this->get_parameter_or<double>("adaptive.attack_alpha", adaptive_attack_alpha, 0.5);
        this->get_parameter_or<double>("adaptive.release_alpha", adaptive_release_alpha, 0.1);
        this->get_parameter_or<double>("adaptive.high_scale_warn_th", adaptive_high_scale_warn_th, 3.0);
        this->get_parameter_or<double>("adaptive.max_downweight_duration_s",
                                       adaptive_max_downweight_duration_s, 3.0);
        if (adaptive_enable && !degeneracy_enable)
        {
            RCLCPP_WARN(this->get_logger(),
                "adaptive.enable=true 但 degeneracy.enable=false: adaptive 降权不会生效 "
                "(matching/geometry 信号来自 degeneracy 门控内的 degen_data)");
        }

        // P4 directional 参数: 经 launch 参数 dict 传入 (唯一参数源, 同 P3 教训)。
        // 默认全关 = 与 p3-final 数值等价 (esekfom projector 默认 identity)。
        this->declare_parameter<bool>("directional.enable", directional_enable);
        this->declare_parameter<double>("directional.beta_t", directional_beta_t);
        this->declare_parameter<double>("directional.beta_r", directional_beta_r);
        this->declare_parameter<bool>("directional.ht_accum_enable", directional_ht_accum_enable);
        this->declare_parameter<double>("directional.ht_accum_alpha", directional_ht_accum_alpha);
        this->declare_parameter<bool>("directional.imu_safety_limiter_enable",
                                      directional_imu_safety_limiter_enable);
        this->declare_parameter<double>("directional.imu_beta_floor", directional_imu_beta_floor);
        this->get_parameter_or<bool>("directional.enable", directional_enable, false);
        this->get_parameter_or<double>("directional.beta_t", directional_beta_t, 0.5);
        this->get_parameter_or<double>("directional.beta_r", directional_beta_r, 1.0);
        this->get_parameter_or<bool>("directional.ht_accum_enable",
                                     directional_ht_accum_enable, false);
        this->get_parameter_or<double>("directional.ht_accum_alpha",
                                       directional_ht_accum_alpha, 0.5);
        this->get_parameter_or<bool>("directional.imu_safety_limiter_enable",
                                     directional_imu_safety_limiter_enable, false);
        this->get_parameter_or<double>("directional.imu_beta_floor",
                                       directional_imu_beta_floor, 0.5);
        if (directional_enable && !degeneracy_enable)
        {
            RCLCPP_WARN(this->get_logger(),
                "directional.enable=true 但 degeneracy.enable=false: directional 不会生效 "
                "(弱方向/ratio 信号来自 degeneracy 门控内的 degen_data)");
        }
        if (directional_enable && adaptive_enable)
        {
            RCLCPP_WARN(this->get_logger(),
                "P4 评审 R29: --adaptive 与 --directional 正式实验必须互斥; 当前同时开启, "
                "结果将无法归因于单一机制");
        }

        // P5 robust gate 参数: 经 launch 参数 dict 传入 (唯一参数源, 同 P3/P4 教训)。
        // 默认全关 = 与 p4-final 数值等价 (esekfom callback 默认 nullptr)。
        this->declare_parameter<bool>("robust_gate.enable", robust_gate_enable);
        this->declare_parameter<double>("robust_gate.num_drop_th", gate_num_drop_th);
        this->declare_parameter<double>("robust_gate.ratio_drop_th", gate_ratio_drop_th);
        this->declare_parameter<double>("robust_gate.p90_rise_th", gate_p90_rise_th);
        this->declare_parameter<double>("robust_gate.base_window_s", gate_base_window_s);
        this->declare_parameter<double>("robust_gate.recover_num_th", gate_recover_num_th);
        this->declare_parameter<double>("robust_gate.recover_ratio_th", gate_recover_ratio_th);
        this->declare_parameter<double>("robust_gate.recover_p90_th", gate_recover_p90_th);
        this->declare_parameter<int>("robust_gate.degraded_after", gate_degraded_after);
        this->declare_parameter<int>("robust_gate.recovery_required_after",
                                     gate_recovery_required_after);
        this->declare_parameter<int>("robust_gate.recover_confirm_frames",
                                     gate_recover_confirm_frames);
        this->get_parameter_or<bool>("robust_gate.enable", robust_gate_enable, false);
        this->get_parameter_or<double>("robust_gate.num_drop_th", gate_num_drop_th, 0.40);
        this->get_parameter_or<double>("robust_gate.ratio_drop_th", gate_ratio_drop_th, 0.30);
        this->get_parameter_or<double>("robust_gate.p90_rise_th", gate_p90_rise_th, 2.0);
        this->get_parameter_or<double>("robust_gate.base_window_s", gate_base_window_s, 30.0);
        this->get_parameter_or<double>("robust_gate.recover_num_th", gate_recover_num_th, 0.55);
        this->get_parameter_or<double>("robust_gate.recover_ratio_th", gate_recover_ratio_th, 0.45);
        this->get_parameter_or<double>("robust_gate.recover_p90_th", gate_recover_p90_th, 1.5);
        this->get_parameter_or<int>("robust_gate.degraded_after", gate_degraded_after, 2);
        this->get_parameter_or<int>("robust_gate.recovery_required_after",
                                    gate_recovery_required_after, 5);
        this->get_parameter_or<int>("robust_gate.recover_confirm_frames",
                                    gate_recover_confirm_frames, 2);
        if (robust_gate_enable && !degeneracy_enable)
        {
            RCLCPP_WARN(this->get_logger(),
                "robust_gate.enable=true 但 degeneracy.enable=false: gate 不会生效 "
                "(matching 信号来自 degeneracy 门控内的 degen_data)");
        }
        if (robust_gate_enable && adaptive_enable)
        {
            RCLCPP_WARN(this->get_logger(),
                "P5: robust_gate 与 adaptive 正式实验必须互斥; 当前同时开启, 结果将无法归因");
        }
        if (robust_gate_enable && directional_enable)
        {
            RCLCPP_WARN(this->get_logger(),
                "P5: robust_gate 与 directional 正式实验必须互斥; 当前同时开启, 结果将无法归因");
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
            // P5: 本帧 gate 状态重置 (callback 在 esekfom 第一次 h_dyn_share 后判定)
            gate_rejected_ = false;
            gate_map_update_skipped_ = false;
            // P3: 用上一帧 detected final_scale 更新本帧 applied covariance (one-frame delay)
            apply_adaptive_lidar_cov();
            // P4: 本帧 directional projector 参数 (来自上一帧 update 后的 dir_cache_, one-frame delay)
            const Eigen::Vector3d *dir_w = nullptr; double dir_beta = 1.0;
            compute_directional_trigger(dir_beta, dir_w);
            // P5: 传入 current-frame gate callback (默认 nullptr = 关闭, 零侵入)。
            // 拒绝时 esekfom 显式恢复 propagated state/P 并置 measurement_rejected=true。
            bool meas_rejected = false;
            kf.update_iterated_dyn_share_modified(lidar_meas_cov_, solve_H_time, dir_w, dir_beta,
                                                  robust_gate_enable ? robust_gate_callback : nullptr,
                                                  nullptr, &meas_rejected);
            gate_rejected_ = meas_rejected;
            // P5: map 门控标记立即设置 (publish_lio_health 需要它在 publish 时已就绪)。
            //   拒绝帧默认不插图 (防污染, 评审 R6/R7); RECOVERY_REQUIRED 时允许传播位姿
            //   插图 (gate_recovery_allow_map_, 防地图冻结导致的永久失配)。
            gate_map_update_skipped_ = meas_rejected && !gate_recovery_allow_map_;
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

            /*** P4: 用本帧 degen_data 填充方向缓存 (供下一帧 projector, one-frame delay) ***/
            if (degeneracy_enable && directional_enable)
            {
                update_directional_cache();
            }

            /*** Publish LIO health (P2, same timestamp as degen score) ***/
            if (degeneracy_enable && health_enable)
            {
                publish_lio_health();
            }

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***
             * P5 (评审 R6/R7): 被拒绝的 LiDAR 帧不得插入地图 —— 该帧位姿只是 IMU
             * 传播预测, 点云在该位姿下可能与真实几何错位, 插图会污染 ikd-Tree,
             * 使下一帧在污染地图上匹配。拒绝帧仍发布 odometry/点云 (调试用),
             * 但跳过 map_incremental()。map_update_skipped 与 measurement_rejected
             * 保持一致, 供事后验证污染防护生效。 ***/
            t3 = omp_get_wtime();
            if (!gate_map_update_skipped_)
            {
                map_incremental();
            }
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
