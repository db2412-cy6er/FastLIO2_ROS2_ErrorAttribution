// Copyright 2026
//
// Licensed under the MIT License (the "License");
// you may not use this file except in compliance with the License.

#ifndef GROUND_TRUTH_BRIDGE__GROUND_TRUTH_BRIDGE_HPP_
#define GROUND_TRUTH_BRIDGE__GROUND_TRUTH_BRIDGE_HPP_

#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

namespace ground_truth_bridge
{

/**
 * @brief 仿真 Ground Truth 桥接节点。
 *
 * 订阅 Gazebo P3D 插件发布的真值里程计 (/gt_odom)，
 * 累计为 nav_msgs/Path (/gt_path) 用于 RViz 可视化与离线评测，
 * 并可选地将轨迹以 TUM 格式 (t x y z qx qy qz qw) 写入日志文件，
 * 供 evo 工具直接做 ATE/RPE 误差分析。
 *
 * 同时内置健康监测：若长时间未收到真值消息会持续告警，
 * 避免"采集了半天数据才发现真值缺失"的无效实验。
 */
class GroundTruthBridge : public rclcpp::Node
{
public:
  explicit GroundTruthBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void healthTimerCallback();

  // --- 参数 ---
  std::string gt_odom_topic_;
  std::string gt_path_topic_;
  std::string frame_id_;
  double timeout_sec_;
  std::string log_file_;

  // --- 订阅 / 发布 / 定时器 ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  // --- 运行状态 ---
  nav_msgs::msg::Path path_msg_;
  std::ofstream tum_log_;
  size_t msg_count_{0u};
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace ground_truth_bridge

#endif  // GROUND_TRUTH_BRIDGE__GROUND_TRUTH_BRIDGE_HPP_