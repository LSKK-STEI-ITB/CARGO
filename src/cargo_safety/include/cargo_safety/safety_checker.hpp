#include <mavros_msgs/msg/detail/rc_in__struct.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_srvs/srv/detail/empty__struct.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>

#define RC_CHECK 1

// Change to the radio channel
#define RC_STABILIZE 7
#define RC_ESTOP 6

class SafetyChecker : public rclcpp::Node {
public:
  SafetyChecker();
  bool check_cargo_node();
  void call_kill_cargo_srv();
  bool rc_pwm(int val, const std::string &target);
  template <typename MessageT>
  void
  wait_for_message(std::shared_ptr<MessageT> &msg, rclcpp::Node::SharedPtr node,
                   const std::string &topic,
                   std::chrono::nanoseconds timeout = std::chrono::seconds(5));
  int rc_trigger();

  bool call_ready = false;

private:
  const std::string RCL_ESTOP_PWM = "HIGH";
  const std::string RC_STABILIZE_PWM = "LOW";
  const std::string RC_LOITER_PWM = "HIGH";
  std::string name_space;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_kill_cargo;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_kill_mission;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
};
