#ifndef _PREAPPROACH_HPP_
#define _PREAPPROACH_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

class PreApproach : public rclcpp::Node {
private:
  // parameter variables
  double obstacle;
  int degrees;

  // member variables
  double yaw;
  double min_dist;

  // flag variables
  bool dist_done;
  bool angl_done;

  // ros objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_robot_control;

  void
  subscriber_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_robot_control_callback();

public:
  explicit PreApproach(const rclcpp::NodeOptions &options);
};

} // namespace my_components

#endif