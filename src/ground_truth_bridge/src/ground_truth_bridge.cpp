// Copyright 2026
//
// Licensed under the MIT License (the "License");
// you may not use this file except in compliance with the License.

#include "ground_truth_bridge/ground_truth_bridge.hpp"

namespace ground_truth_bridge
{

GroundTruthBridge::GroundTruthBridge(const rclcpp::NodeOptions & options)
: Node("ground_truth_bridge", options)
{
  gt_odom_topic_ = this->declare_parameter<std::string>("gt_odom_topic", "/gt_odom");
  gt_path_topic_ = this->declare_parameter<std::string>("gt_path_topic", "/gt_path");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
  timeout_sec_ = this->declare_parameter<double>("timeout_sec", 5.0);
  log_file_ = this->declare_parameter<std::string>("log_file", "");

  // 真值消息使用 best-effort QoS 即可，与雷达/里程计一致
  auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    gt_odom_topic_, qos,
    std::bind(&GroundTruthBridge::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(gt_path_topic_, 10);
  path_msg_.header.frame_id = frame_id_;

  if (!log_file_.empty()) {
    tum_log_.open(log_file_, std::ios::app);
    if (tum_log_.is_open()) {
      RCLCPP_INFO(this->get_logger(), "TUM 轨迹日志将追加写入: %s", log_file_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "无法打开轨迹日志文件: %s", log_file_.c_str());
    }
  }

  health_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&GroundTruthBridge::healthTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "ground_truth_bridge 启动: 订阅 %s, 发布 %s",
    gt_odom_topic_.c_str(), gt_path_topic_.c_str());
}

void GroundTruthBridge::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_msg_time_ = msg->header.stamp;
  ++msg_count_;

  // 1) 累计路径
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  path_msg_.poses.push_back(pose);
  path_msg_.header.stamp = msg->header.stamp;
  path_pub_->publish(path_msg_);

  // 2) 可选: TUM 格式轨迹日志 (供 evo 直接使用)
  if (tum_log_.is_open()) {
    const auto & p = msg->pose.pose.position;
    const auto & o = msg->pose.pose.orientation;
    const double t = static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    tum_log_ << std::fixed << t << " "
             << p.x << " " << p.y << " " << p.z << " "
             << o.x << " " << o.y << " " << o.z << " " << o.w << "\n";
  }
}

void GroundTruthBridge::healthTimerCallback()
{
  if (msg_count_ == 0u) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "尚未收到任何 %s 消息，请确认 Gazebo 真值插件 (libgazebo_ros_p3d) 已加载",
      gt_odom_topic_.c_str());
    return;
  }

  const rclcpp::Time now = this->now();
  if (now.seconds() > 0.0 &&
    last_msg_time_.seconds() > 0.0 &&
    (now - last_msg_time_).seconds() > timeout_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "真值消息已停止更新超过 %.1f 秒 (最近 %s)，请检查仿真是否暂停或机器人是否丢失",
      timeout_sec_, last_msg_time_.seconds() > 0.0 ?
      "收到过消息" : "从未收到消息");
  }
}

}  // namespace ground_truth_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ground_truth_bridge::GroundTruthBridge>());
  rclcpp::shutdown();
  return 0;
}