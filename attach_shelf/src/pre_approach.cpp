#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RB1Control : public rclcpp::Node {
private:
public:
  // constructor
  RB1Control() : Node("rb1_control_node") {
    this->declare_parameter<double>("obstacle", 0.0);
    this->declare_parameter<int>("degrees", 0.0);
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