#include "my_components/attach_client.hpp"

#include "attach_shelf_interface/srv/go_to_loading.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

// constructor
AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client_node", options), final_approach{true} {
  // ros objects
  this->service_client =
      this->create_client<attach_shelf_interface::srv::GoToLoading>(
          "/approach_shelf");
  this->timer_service_client = this->create_wall_timer(
      std::chrono::seconds(1 / 1),
      std::bind(&AttachClient::timer_service_client_callback, this));
}

void AttachClient::timer_service_client_callback() {
  // node acknowledgement
  RCLCPP_INFO_ONCE(this->get_logger(), "Initiating final approch");

  int counter = 0;
  while (!this->service_client->wait_for_service(std::chrono::seconds(1))) {
    // critical for closing infinite loop
    if (!rclcpp::ok()) {
      // node acknowledgement
      RCLCPP_ERROR(this->get_logger(), "Terminating: interrupt received");
      return;
    }
    counter++;
    // node feedback
    RCLCPP_WARN(this->get_logger(), "Waiting for server /approach_shelf %d",
                counter);
  }

  // define request body
  auto request =
      std::make_shared<attach_shelf_interface::srv::GoToLoading::Request>();
  request->attach_to_shelf = this->final_approach;

  // send request to service server
  auto future = service_client->async_send_request(
      request, std::bind(&AttachClient::server_response_callback, this,
                         std::placeholders::_1));

  // better alternative to flag is behavior tree
  this->timer_service_client.reset();
}

void AttachClient::server_response_callback(
    rclcpp::Client<attach_shelf_interface::srv::GoToLoading>::SharedFuture
        future) {
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    auto response = future.get();
    bool completed = response->complete;
    // node feedback
    RCLCPP_INFO_ONCE(this->get_logger(), "Status: %s",
                     completed ? "true" : "false");
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)