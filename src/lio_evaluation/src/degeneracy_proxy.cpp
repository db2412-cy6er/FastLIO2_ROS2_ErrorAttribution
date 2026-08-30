// ============================================================================
// External Geometry Proxy Detector (外部几何代理检测器)
// P1: Degeneracy Detection and Evaluation
//
// 作用: 与 FAST-LIO 内嵌精确检测器 (/lio/degeneracy_score) 做旁路对照验证。
//   订阅 FAST-LIO 输出的 /cloud_registered (世界系注册点云) 与 /Odometry,
//   自维护 voxel 局部地图, 通过 kNN + PCA 平面拟合重建几何约束, 近似计算
//   LiDAR geometric Hessian (Ht/Hr/H6), 发布 /lio/degeneracy_score_proxy。
//
// 口径注意: 本节点消费的 /cloud_registered 已经经过 FAST-LIO 当前位姿估计,
//   因此它只验证"内部 Hessian 分数"与"外部点云几何趋势"是否一致,
//   不是独立真值 (ground truth) 检测器。
//
// 与内嵌检测器的口径差异 (M4c 一致性分析需记录):
//   - 局部地图维护方式不同 (voxel + 时间剪枝 vs ikd-tree 增量地图)
//   - kNN 数量 / PCA 平面拟合参数 / 中心化方式不同
//   - 转动 Jacobian 使用帧质心中心化 (消除大平移偏置)
// ============================================================================

#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <lio_interfaces/msg/degeneracy_score.hpp>

namespace lio_evaluation
{

class DegeneracyProxyNode : public rclcpp::Node
{
public:
  explicit DegeneracyProxyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("degeneracy_proxy", options)
  {
    // ---- 话题参数 ----
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry");
    score_topic_ = this->declare_parameter<std::string>("score_topic", "/lio/degeneracy_score_proxy");

    // ---- 几何重建参数 ----
    scan_voxel_ = this->declare_parameter<double>("scan_voxel", 0.5);
    map_voxel_ = this->declare_parameter<double>("map_voxel", 0.5);
    k_neighbors_ = this->declare_parameter<int>("k_neighbors", 5);
    max_nn_dist_ = this->declare_parameter<double>("max_nn_dist", 2.0);
    plane_ratio_th_ = this->declare_parameter<double>("plane_ratio_th", 0.2);
    map_lifetime_ = this->declare_parameter<double>("map_lifetime", 30.0);

    // ---- 退化判定参数 (与内嵌检测器同口径) ----
    thresh_t_ = this->declare_parameter<double>("threshold_t", 0.02);
    thresh_r_ = this->declare_parameter<double>("threshold_r", 0.02);
    ema_alpha_ = this->declare_parameter<double>("ema_alpha", 0.5);
    score_enter_ = this->declare_parameter<double>("score_enter", 0.4);
    score_exit_ = this->declare_parameter<double>("score_exit", 0.2);
    min_enter_ = this->declare_parameter<int>("min_enter_frames", 3);
    min_exit_ = this->declare_parameter<int>("min_exit_frames", 5);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DegeneracyProxyNode::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DegeneracyProxyNode::odomCallback, this, std::placeholders::_1));
    score_pub_ = this->create_publisher<lio_interfaces::msg::DegeneracyScore>(score_topic_, 20);

    scan_filter_.setLeafSize(scan_voxel_, scan_voxel_, scan_voxel_);

    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    scan_ds_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    last_pos_ = Eigen::Vector3d::Zero();

    RCLCPP_INFO(this->get_logger(),
      "degeneracy_proxy started: %s + %s -> %s (k=%d, scan_voxel=%.2f, map_voxel=%.2f, lifetime=%.1fs)",
      cloud_topic_.c_str(), odom_topic_.c_str(), score_topic_.c_str(),
      k_neighbors_, scan_voxel_, map_voxel_, map_lifetime_);
  }

private:
  struct ProxyData
  {
    bool valid = false;
    double trans_ratio = 1.0;
    double rot_ratio = 1.0;
    double normal_concentration = 1.0 / 3.0;
    double condition_number = 1.0;
    Eigen::Matrix<double, 6, 1> eigenvalues6 = Eigen::Matrix<double, 6, 1>::Ones();
    Eigen::Matrix<double, 6, 1> weak_direction = Eigen::Matrix<double, 6, 1>::Zero();
    int effective_feature_num = 0;
    double mean_residual = 0.0;
    double ema_score = 0.0;
    bool is_degenerate = false;
  };

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    last_odom_stamp_ = msg->header.stamp;
    last_pos_ << msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);
    if (scan->empty()) return;

    last_cloud_stamp_ = msg->header.stamp;
    const double t_sec = rclcpp::Time(msg->header.stamp).seconds();

    // 1) 当前帧降采样
    scan_filter_.setInputCloud(scan);
    scan_ds_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    scan_filter_.filter(*scan_ds_);
    if (scan_ds_->empty()) return;

    // 2) 更新局部地图 (追加 + 按年龄剪枝)
    updateMap(*scan_ds_, t_sec);

    // 3) 计算几何约束 + 退化分数
    computeScore();

    // 4) 发布
    publishScore();
  }

  void updateMap(const pcl::PointCloud<pcl::PointXYZ> & add, double t_sec)
  {
    map_cloud_->insert(map_cloud_->end(), add.begin(), add.end());
    map_times_.insert(map_times_.end(), add.size(), t_sec);

    size_t kept = 0;
    for (size_t i = 0; i < map_cloud_->size(); i++)
    {
      if (t_sec - map_times_[i] <= map_lifetime_)
      {
        (*map_cloud_)[kept] = (*map_cloud_)[i];
        map_times_[kept] = map_times_[i];
        kept++;
      }
    }
    map_cloud_->resize(kept);
    map_times_.resize(kept);
  }

  void computeScore()
  {
    ProxyData d;
    if (map_cloud_->empty() || scan_ds_->empty())
    {
      data_ = d;
      return;
    }

    kdtree_.setInputCloud(map_cloud_);

    const int n = static_cast<int>(scan_ds_->size());
    const int k = std::max(3, k_neighbors_);

    // 帧质心 (转动 Jacobian 中心化, 消除大平移偏置)
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; i++)
    {
      centroid += Eigen::Vector3d((*scan_ds_)[i].x, (*scan_ds_)[i].y, (*scan_ds_)[i].z);
    }
    centroid /= static_cast<double>(n);

    Eigen::Matrix3d Ht = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Hr = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 6, 6> H6 = Eigen::Matrix<double, 6, 6>::Zero();

    int eff = 0;
    double res_sum = 0.0;

    std::vector<int> idx(k);
    std::vector<float> dist(k);
    const float nn_dist2 = static_cast<float>(max_nn_dist_ * max_nn_dist_);

    for (int i = 0; i < n; i++)
    {
      const pcl::PointXYZ & p = (*scan_ds_)[i];
      if (kdtree_.nearestKSearch(p, k, idx, dist) < k) continue;
      if (dist[k - 1] > nn_dist2) continue;

      // PCA 平面拟合 (邻域点)
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (int j = 0; j < k; j++)
      {
        const pcl::PointXYZ & q = (*map_cloud_)[idx[j]];
        mean += Eigen::Vector3d(q.x, q.y, q.z);
      }
      mean /= static_cast<double>(k);

      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (int j = 0; j < k; j++)
      {
        const pcl::PointXYZ & q = (*map_cloud_)[idx[j]];
        Eigen::Vector3d dvec = Eigen::Vector3d(q.x, q.y, q.z) - mean;
        cov.noalias() += dvec * dvec.transpose();
      }
      cov /= static_cast<double>(k);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      Eigen::Vector3d w = es.eigenvalues(); // ascending
      // 平面性判据: 最小特征值相对前两个足够小
      if ((w(1) + w(2)) > 1e-12 && w(0) > plane_ratio_th_ * (w(1) + w(2))) continue;

      Eigen::Vector3d nvec = es.eigenvectors().col(0).normalized();
      Eigen::Vector3d r = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
      Eigen::Vector3d a = r.cross(nvec);

      Ht.noalias() += nvec * nvec.transpose();
      Hr.noalias() += a * a.transpose();
      Eigen::Matrix<double, 6, 1> j;
      j << nvec, a;
      H6.noalias() += j * j.transpose();

      res_sum += std::fabs(nvec.dot(Eigen::Vector3d(p.x, p.y, p.z) - mean));
      eff++;
    }

    if (eff < 3) { data_ = d; return; }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_t(Ht);
    Eigen::Vector3d wt = es_t.eigenvalues();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_r(Hr);
    Eigen::Vector3d wr = es_r.eigenvalues();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es6(H6);
    Eigen::Matrix<double, 6, 1> w6 = es6.eigenvalues();

    d.valid = true;
    d.trans_ratio = (wt(2) > 1e-12) ? wt(0) / wt(2) : 0.0;
    d.rot_ratio = (wr(2) > 1e-12) ? wr(0) / wr(2) : 0.0;
    d.normal_concentration = (wt.sum() > 1e-12) ? wt(2) / wt.sum() : 0.0;
    d.condition_number = (w6(0) > 1e-12) ? w6(5) / w6(0) : std::numeric_limits<double>::max();
    d.eigenvalues6 = w6;
    d.weak_direction = es6.eigenvectors().col(0);
    d.effective_feature_num = eff;
    d.mean_residual = res_sum / static_cast<double>(eff);

    // ---- 平滑状态机 (与内嵌检测器同口径: EMA + hysteresis) ----
    double raw = std::max(1.0 - d.trans_ratio / thresh_t_, 1.0 - d.rot_ratio / thresh_r_);
    raw = std::min(1.0, std::max(0.0, raw));
    if (!ema_inited_) { ema_score_ = raw; ema_inited_ = true; }
    else { ema_score_ = ema_alpha_ * raw + (1.0 - ema_alpha_) * ema_score_; }

    if (!is_degenerate_)
    {
      if (ema_score_ > score_enter_)
      {
        if (++enter_cnt_ >= min_enter_) { is_degenerate_ = true; enter_cnt_ = 0; }
      }
      else { enter_cnt_ = 0; }
    }
    else
    {
      if (ema_score_ < score_exit_)
      {
        if (++exit_cnt_ >= min_exit_) { is_degenerate_ = false; exit_cnt_ = 0; }
      }
      else { exit_cnt_ = 0; }
    }

    d.ema_score = ema_score_;
    d.is_degenerate = is_degenerate_;
    data_ = d;
  }

  void publishScore()
  {
    if (!data_.valid) return;

    lio_interfaces::msg::DegeneracyScore msg;
    msg.header.stamp = last_cloud_stamp_;
    msg.header.frame_id = "camera_init";
    msg.score = data_.ema_score;
    msg.is_degenerate = data_.is_degenerate;
    msg.trans_ratio = data_.trans_ratio;
    msg.rot_ratio = data_.rot_ratio;
    msg.condition_number = data_.condition_number;
    msg.normal_concentration = data_.normal_concentration;
    for (int i = 0; i < 6; i++)
    {
      msg.eigenvalues[i] = data_.eigenvalues6(i);
      msg.weak_direction[i] = data_.weak_direction(i);
    }
    msg.effective_feature_num = data_.effective_feature_num;
    msg.mean_residual = data_.mean_residual;
    msg.pos_x = last_pos_(0);
    msg.pos_y = last_pos_(1);
    msg.pos_z = last_pos_(2);
    score_pub_->publish(msg);
  }

  // ---- topics / params ----
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string score_topic_;
  double scan_voxel_;
  double map_voxel_;
  int k_neighbors_;
  double max_nn_dist_;
  double plane_ratio_th_;
  double map_lifetime_;
  double thresh_t_;
  double thresh_r_;
  double ema_alpha_;
  double score_enter_;
  double score_exit_;
  int min_enter_;
  int min_exit_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<lio_interfaces::msg::DegeneracyScore>::SharedPtr score_pub_;

  // ---- state ----
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  std::vector<double> map_times_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ds_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  pcl::VoxelGrid<pcl::PointXYZ> scan_filter_;

  rclcpp::Time last_cloud_stamp_;
  rclcpp::Time last_odom_stamp_;
  Eigen::Vector3d last_pos_;

  ProxyData data_;
  bool ema_inited_ = false;
  double ema_score_ = 0.0;
  bool is_degenerate_ = false;
  int enter_cnt_ = 0;
  int exit_cnt_ = 0;
};

}  // namespace lio_evaluation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lio_evaluation::DegeneracyProxyNode>());
  rclcpp::shutdown();
  return 0;
}

