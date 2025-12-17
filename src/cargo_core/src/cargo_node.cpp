// This is basic node that will be started following by other nodes
#include "cargo_core/cargo_node.hpp"

CargoNode::CargoNode() : Node("cargo_node") {
  // Kill node when service called
  RCLCPP_INFO(this->get_logger(), "Cargo Node Initialized.");

  name_space = declare_parameter("namespace", "rangga");

  cargo_service = this->create_service<std_srvs::srv::Empty>(
      "/" + name_space + "/cargo_node/kill_all",
      std::bind(&CargoNode::kill_node_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Task Started.");
}
void CargoNode::kill_node_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_WARN(this->get_logger(), "--- Cargo Node will be Killed ---");
  rclcpp::shutdown();
}
