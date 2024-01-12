#include "attach_shelf_interface/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class ApproachService : public rclcpp::Node {
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
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_elevator_up;
  rclcpp::TimerBase::SharedPtr timer_tf_broadcaster;

  // tf objects
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster;

  void
  subscriber_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // find the front distance after reducing noise
    auto first = msg->intensities.begin();
    auto last = msg->intensities.end();

    // lambda function to detect reflector
    auto find_reflector = [](int i) { return i > 7000; };

    // find first leg
    auto it1 = std::find_if(first, last, find_reflector);
    if (it1 == last) {
      this->is_approachable = false;
      return;
    }
    int loc1 = std::distance(first, it1);
    double leg1_angle = (loc1 - 540) * msg->angle_increment;
    double leg1_distance = msg->ranges.at(loc1);

    // find second leg
    auto it2 = std::find_if(first + loc1 + 20, last, find_reflector);
    if (it2 == last) { // bug fix: error `out_of_range` in std::distance
      this->is_approachable = false;
      return;
    }
    int loc2 = std::distance(first, it2);
    double leg2_angle = (loc2 - 540) * msg->angle_increment;
    double leg2_distance = msg->ranges.at(loc2);

    // node feedback
    RCLCPP_INFO_ONCE(this->get_logger(), "loc1 : %d (l1 : %f, a1 %f)", loc1,
                     leg1_distance, leg1_angle);
    RCLCPP_INFO_ONCE(this->get_logger(), "loc2 : %d (l2 : %f, a2 %f)", loc2,
                     leg2_distance, leg2_angle);

    // validate if both legs of shelf are detected
    if ((it1 != last) && (it2 != last)) {
      // calculate (x1, y1), (x2, y2)
      double x1 = leg1_distance * std::cos(leg1_angle);
      double y1 = leg1_distance * std::sin(leg1_angle);
      double x2 = leg2_distance * std::cos(leg2_angle);
      double y2 = leg2_distance * std::sin(leg2_angle);

      // calculate mid point between shelf legs
      this->px = (x1 + x2) / 2.0;
      this->py = (y1 + y2) / 2.0;
      this->is_approachable = true;

      // node feedback
      RCLCPP_INFO_ONCE(this->get_logger(), "(x1 : %f, y1 %f), (x2 : %f, y2 %f)",
                       x1, y1, x2, y2);
    } else {
      this->is_approachable = false;
    }
  }

  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // reading current position from /odom topic
    double y = std::fabs(msg->pose.pose.position.y);
    this->distance = this->distance + (y - this->y_);
    this->y_ = y;
  }

  // member method
  void service_callback(
      const std::shared_ptr<attach_shelf_interface::srv::GoToLoading::Request>
          request,
      const std::shared_ptr<attach_shelf_interface::srv::GoToLoading::Response>
          response) {

    // node feedback
    RCLCPP_INFO(this->get_logger(),
                "/approach_shelf has received a new request.");

    // global control structure
    if (this->is_approachable) {
      // publish transformation
      // publish_cart_frame();
      if (request->attach_to_shelf) {
        // move robot
        move_robot_to_cart();
        move_robot_under_cart();
        // stop robot
        halt_robot();
        // elevate cart
        elevate_cart();
        response->complete = true;
      } else {
        // dont move robot
        response->complete = true;
      }
    } else {
      response->complete = false;
    }
  }

  void publish_cart_frame() {
    // check if frame is publishable and shelf cart is approachable
    if (!this->is_approachable) {
      return;
    }

    // odom->laser transformation
    tf2::Stamped<tf2::Transform> odom2laser;
    geometry_msgs::msg::TransformStamped odom2laser_msg;
    std::string f_ref = "robot_odom";
    std::string f_tar = "robot_front_laser_link";
    try {
      odom2laser_msg =
          this->tf_buffer->lookupTransform(f_ref, f_tar, tf2::TimePointZero);
      tf2::fromMsg(odom2laser_msg, odom2laser);
    } catch (const tf2::TransformException &ex) {
      // node feedback
      RCLCPP_WARN(get_logger(), "Requested transform not found: %s", ex.what());
      return;
    }

    // current time stamp of the system
    // bug fix : transformation not found due to timelag between nodes
    auto timestamp = odom2laser_msg.header.stamp;

    // laser->cart transformation
    tf2::Transform laser2cart;
    laser2cart.setOrigin(tf2::Vector3(this->px, this->py, 0.0));
    laser2cart.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // odom->cart transformation
    tf2::Transform odom2cart = odom2laser * laser2cart;

    // publish odom->cart transformation
    // package issue: https://github.com/ros2/geometry2/issues/176
    // odom2cart_msg.transform = tf2::toMsg(odom2cart);
    geometry_msgs::msg::TransformStamped odom2cart_msg;
    odom2cart_msg.header.stamp = timestamp;
    odom2cart_msg.header.frame_id = "robot_odom";
    odom2cart_msg.child_frame_id = "cart_frame";
    odom2cart_msg.transform.translation.x = odom2cart.getOrigin().getX();
    odom2cart_msg.transform.translation.y = odom2cart.getOrigin().getY();
    odom2cart_msg.transform.translation.z = odom2cart.getOrigin().getZ();
    odom2cart_msg.transform.rotation.x = 0;
    odom2cart_msg.transform.rotation.y = 0;
    odom2cart_msg.transform.rotation.z = 0;
    odom2cart_msg.transform.rotation.w = 1;

    // node feedback
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Publishing `robot_odom` to `cart_frame`");

    // publish transformation
    this->tf_dynamic_broadcaster->sendTransform(odom2cart_msg);
  }

  void move_robot_to_cart() {
    // node feedback
    RCLCPP_INFO_ONCE(this->get_logger(), "Moving robot towards the shelf");

    // parameters
    std::string f_ref = "robot_base_footprint";
    std::string f_tar = "cart_frame";

    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();

    while (rclcpp::ok()) {
      // fetch transformation
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer->lookupTransform(f_ref, f_tar, tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        // node feedback
        RCLCPP_WARN(get_logger(), "Requested transform not found: %s",
                    ex.what());
        return;
      }

      auto x = t.transform.translation.x;
      auto y = t.transform.translation.y;

      // calculate error
      auto error_distance = std::sqrt(x * x + y * y);
      auto error_yaw = std::atan2(y, x);

      // local control structure
      if (error_distance > 0.15) {
        message.linear.x = 0.2;
        message.angular.z = error_yaw / 2;
      } else {
        break;
      }

      // node feedback
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Delta (MTC) (distance : %f, angle : %f)",
                           error_distance, error_yaw);

      // publish velocity
      this->publisher_cmd_vel->publish(message);
    }
  }

  void move_robot_under_cart() {
    // node feedback
    RCLCPP_INFO_ONCE(this->get_logger(), "Moving robot under the shelf");

    // set distance to zero
    this->distance = 0.0;

    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();

    while (rclcpp::ok()) {
      // local control structure
      if (this->distance < 0.45) {
        message.linear.x = 0.1;
        message.angular.z = 0;
      } else {
        break;
      }

      // node feedback
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Delta (MUC) (distance : %f)", this->distance);

      // publish velocity
      this->publisher_cmd_vel->publish(message);
    }
  }

  void halt_robot() {
    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();

    message.linear.x = 0;
    message.angular.z = 0;

    // publish velocity
    this->publisher_cmd_vel->publish(message);

    // bad fix: halt the process for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  void elevate_cart() {
    // node feedback
    RCLCPP_INFO_ONCE(this->get_logger(), "Elevating shelf cart");

    // publishing command to the topic /elevator_up
    auto message = std_msgs::msg::Empty();

    // publish command
    this->publisher_elevator_up->publish(message);
  }

public:
  // constructor
  ApproachService()
      : Node("approach_shelf_service_node"), y_{0.0},
        is_approachable(false), is_publishable{false} {
    // callback groups objects
    callback_g1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_g2 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ros node options
    rclcpp::SubscriptionOptions sub_callback_g1;
    sub_callback_g1.callback_group = callback_g1;

    // ros objects
    this->service_server =
        create_service<attach_shelf_interface::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachService::service_callback, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, callback_g2);
    this->subscriber_scan =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ApproachService::subscriber_scan_callback, this,
                      std::placeholders::_1),
            sub_callback_g1);
    this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ApproachService::subscriber_odom_callback, this,
                  std::placeholders::_1),
        sub_callback_g1);
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    this->publisher_elevator_up =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
    this->timer_tf_broadcaster = this->create_wall_timer(
        std::chrono::seconds(1 / 120),
        std::bind(&ApproachService::publish_cart_frame, this), callback_g1);

    // tf objects
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    this->tf_static_broadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    this->tf_dynamic_broadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // node acknowledgement
    RCLCPP_INFO(this->get_logger(),
                "The service /approach_shelf is available for request.");
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<ApproachService> node = std::make_shared<ApproachService>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}