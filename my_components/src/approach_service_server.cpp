#include "my_components/attach_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options;
  std::shared_ptr<my_components::AttachServer> node =
      std::make_shared<my_components::AttachServer>(options);

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}