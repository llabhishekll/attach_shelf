#ifndef _ATTACHCLIENT_HPP_
#define _ATTACHCLIENT_HPP_

#include "attach_shelf_interface/srv/go_to_loading.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
private:
  // parameter variables
  bool final_approach;

  // ros objects
  rclcpp::Client<attach_shelf_interface::srv::GoToLoading>::SharedPtr
      service_client;
  rclcpp::TimerBase::SharedPtr timer_service_client;

  void timer_service_client_callback();
  void server_response_callback(
      rclcpp::Client<attach_shelf_interface::srv::GoToLoading>::SharedFuture
          future);

public:
  explicit AttachClient(const rclcpp::NodeOptions &options);
};

} // namespace my_components

#endif