#ifndef _ATTACHSERVER_HPP_
#define _ATTACHSERVER_HPP_

#include "attach_shelf_interface/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace my_components {

class AttachServer : public rclcpp::Node {
private:
  // member variables
  double px;
  double py;
  double y_;
  double distance;

  // flag variable
  bool is_approachable;
  bool is_publishable;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Service<attach_shelf_interface::srv::GoToLoading>::SharedPtr service_server;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_elevator_up;
  rclcpp::TimerBase::SharedPtr timer_tf_broadcaster;

  // tf objects
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster;

  void
  subscriber_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void service_callback(
      const std::shared_ptr<attach_shelf_interface::srv::GoToLoading::Request>
          request,
      const std::shared_ptr<attach_shelf_interface::srv::GoToLoading::Response>
          response);
  void publish_cart_frame();
  void move_robot_to_cart();
  void move_robot_under_cart();
  void halt_robot();
  void elevate_cart();

public:
  explicit AttachServer(const rclcpp::NodeOptions &options);
};

} // namespace my_components

#endif