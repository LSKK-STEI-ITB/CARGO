#include "cargo_mission/mission_node.hpp"
#include "cargo_core/cargo_control.hpp"
#include "cargo_core/command_registry.hpp"
#include <fstream>
#include <sstream>

MissionNode::MissionNode() : Node("cargo_mission") {
  cargo_core::register_all_commands();

  std::string mission_file = declare_parameter("mission_file", "mission.txt");

  loadMission(mission_file);

  this->declare_parameter<std::string>("namespace", "rangga");

  // Now read the actual value passed from launch/YAML
  std::string name_space;
  this->get_parameter("namespace", name_space);

  mission_service = this->create_service<std_srvs::srv::Empty>(
      "/" + name_space + "/mission_node/kill_all",
      std::bind(&MissionNode::kill_node_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Mission Node Initialized.");

  RCLCPP_INFO(this->get_logger(), "Task Started.");
}

void MissionNode::kill_node_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_WARN(this->get_logger(), "--- Mission Node will be Killed ---");
  rclcpp::shutdown();
}

void MissionNode::loadMission(const std::string &file) {
  std::ifstream in(file);
  if (!in.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open mission file");
    return;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    MissionStep step;

    ss >> step.name;

    std::string token;
    while (ss >> token) {
      auto pos = token.find('=');
      std::string key = token.substr(0, pos);
      double value = std::stod(token.substr(pos + 1));
      step.params[key] = value;
    }

    steps_.push_back(step);
  }
}

void MissionNode::runMission() {
  for (auto &step : steps_) {
    try {
      cargo_core::execute_command(step.name, shared_from_this(), step.params);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
    }
  }

  RCLCPP_INFO(this->get_logger(), "--- Mission Completed ---");
}
