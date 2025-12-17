#pragma once
#include "cargo_core/command_registry.hpp"
#include "std_srvs/srv/empty.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/detail/empty__struct.hpp>
#include <string>
#include <unordered_map>
#include <vector>

struct MissionStep {
  std::string name;
  cargo_core::ParamMap params;
};

class MissionNode : public rclcpp::Node {
public:
  MissionNode();
  void runMission();

private:
  void loadMission(const std::string &file);
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mission_service;
  std::string name_space;

  void kill_node_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  std::vector<MissionStep> steps_;
};
