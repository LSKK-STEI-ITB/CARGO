#include "cargo_safety/safety_checker.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <std_srvs/srv/detail/empty__struct.hpp>

SafetyChecker::SafetyChecker() : Node("safety_node") {
  RCLCPP_INFO(this->get_logger(), "Safety Node Initialized.");

  this->declare_parameter<std::string>("namespace", "rangga");

  this->get_parameter("namespace", name_space);

  if (!name_space.empty() && name_space.front() == '/')
    name_space.erase(0, 1);

  pub = this->create_publisher<std_msgs::msg::Bool>(
      "/" + name_space + "/safety_node/active", 10);
  client_kill_cargo = this->create_client<std_srvs::srv::Empty>(
      "/" + name_space + "/cargo_node/kill_all");
  client_kill_mission = this->create_client<std_srvs::srv::Empty>(
      "/" + name_space + "/mission_node/kill_all");
}

bool SafetyChecker::check_cargo_node() {
  if (!client_kill_cargo->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for service. Exiting...");
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for cargo node...");
    return false;
  }
  return true;
}

void SafetyChecker::call_kill_cargo_srv() {
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = client_kill_cargo->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Service successfully called.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
  }

  auto future2 = client_kill_mission->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Service successfully called.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
  }

  RCLCPP_INFO(this->get_logger(), "Please Re-Run Cargo Core");
}

bool SafetyChecker::rc_pwm(int val, const std::string &target) {
  if (target == "HIGH") {
    if (val > 1999)
      return true;
    else
      return false;
  } else if (target == "LOW") {
    if (val < 1000)
      return true;
    else
      return false;
  } else {
    RCLCPP_FATAL(this->get_logger(), "CHECK rcPWM func!");
    throw std::invalid_argument("CHECK rcPWM func!");
  }
}

template <typename MessageT>
void SafetyChecker::wait_for_message(std::shared_ptr<MessageT> &msg,
                                     rclcpp::Node::SharedPtr node,
                                     const std::string &topic,
                                     std::chrono::nanoseconds timeout) {
  auto sub = this->create_subscription<MessageT>(
      topic, rclcpp::QoS(10),
      [&msg](typename MessageT::SharedPtr m) { msg = m; });

  rclcpp::Time start = this->now();
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && !msg) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (node->now() - start > rclcpp::Duration(timeout)) {
      break;
    }
  }
}

int SafetyChecker::rc_trigger() {

#if RC_CHECK == 1
  std::shared_ptr<mavros_msgs::msg::RCIn> rc_in_ptr;
  wait_for_message<mavros_msgs::msg::RCIn>(rc_in_ptr, this->shared_from_this(),
                                           "/" + name_space + "/rc/in",
                                           std::chrono::seconds(5));

  if (!rc_in_ptr) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Is RC Connected?");
    return 0;
  }

  if (rc_pwm(rc_in_ptr->channels.at(RC_STABILIZE - 1), RC_STABILIZE_PWM)) {
    RCLCPP_ERROR(this->get_logger(), "RC Stabilize Triggered");
    return 2;
  }

  if (rc_pwm(rc_in_ptr->channels.at(RC_ESTOP - 1), RCL_ESTOP_PWM)) {
    RCLCPP_ERROR(this->get_logger(), "RC Emergency Stop Triggered");
    return 2;
  }

#endif
  return 0;
}
