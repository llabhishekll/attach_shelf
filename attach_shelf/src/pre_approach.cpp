#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RB1Control : public rclcpp::Node {
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

  void parameter_handler() {
    this->obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<double>();
    this->degrees =
        this->get_parameter("degrees").get_parameter_value().get<int>();

    // node acknowledgement
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Parameters (obstacle : %f, degrees : %d)", this->obstacle,
                     this->degrees);
  }

  void
  subscriber_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // select accurate range
    auto f_pos = msg->ranges.begin() + 520;
    auto l_pos = msg->ranges.begin() + 560;

    // find the front distance after reducing noise
    this->min_dist = *std::min_element(f_pos, l_pos);
  }

  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // reading current orientation from /odom topic
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // convert quaternion into euler angles
    double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    this->yaw = (180 / M_PI) * yaw;
  }

  void timer_robot_control_callback() {
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

public:
  // constructor
  RB1Control() : Node("rb1_control_node"), dist_done{false}, angl_done{false} {
    this->declare_parameter<double>("obstacle", 0.0);
    this->declare_parameter<int>("degrees", 0);
    parameter_handler();

    // ros objects
    this->subscriber_scan =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&RB1Control::subscriber_scan_callback, this,
                      std::placeholders::_1));
    this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&RB1Control::subscriber_odom_callback, this,
                  std::placeholders::_1));
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    this->timer_robot_control = this->create_wall_timer(
        std::chrono::milliseconds(1 / 60),
        std::bind(&RB1Control::timer_robot_control_callback, this));
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<RB1Control> node = std::make_shared<RB1Control>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}