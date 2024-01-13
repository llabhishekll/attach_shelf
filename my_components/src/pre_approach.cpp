#include "my_components/pre_approach.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("rb1_control_node", options), obstacle{0.3}, degrees{-90},
      dist_done{false}, angl_done{false} {

  // ros objects
  this->subscriber_scan =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 10,
          std::bind(&PreApproach::subscriber_scan_callback, this,
                    std::placeholders::_1));
  this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PreApproach::subscriber_odom_callback, this,
                std::placeholders::_1));
  this->publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);
  this->timer_robot_control = this->create_wall_timer(
      std::chrono::milliseconds(1 / 60),
      std::bind(&PreApproach::timer_robot_control_callback, this));
}

void PreApproach::subscriber_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // select accurate range
  auto f_pos = msg->ranges.begin() + 520;
  auto l_pos = msg->ranges.begin() + 560;

  // find the front distance after reducing noise
  this->min_dist = *std::min_element(f_pos, l_pos);
}

void PreApproach::subscriber_odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // reading current orientation from /odom topic
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  // convert quaternion into euler angles
  double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  this->yaw = (180 / M_PI) * yaw;
}

void PreApproach::timer_robot_control_callback() {
  // define message and clock
  auto clock = this->get_clock();
  auto message = geometry_msgs::msg::Twist();

  // delta calculation
  double delta_dist = this->obstacle - this->min_dist;
  double delta_angl = this->degrees - this->yaw;

  // node feedback
  RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000,
                       "Delta (distance : %f, angle : %f)", delta_dist,
                       delta_angl);

  // control structure
  if ((std::fabs(delta_dist) > 0.25) && !dist_done) {
    message.linear.x = 0.5;
    message.angular.z = 0.0;
  } else if ((std::fabs(delta_angl) > 2) && !angl_done) {
    dist_done = true;
    message.linear.x = 0.0;
    message.angular.z = ((M_PI / 180) * delta_angl) / 2;
  } else {
    angl_done = true;
    message.linear.x = 0.0;
    message.angular.z = 0.0;
  }

  // node feedback
  RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000,
                       "Velocity ( linear : %f, angular : %f)",
                       message.linear.x, message.angular.z);

  // publish velocity
  publisher_cmd_vel->publish(message);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)