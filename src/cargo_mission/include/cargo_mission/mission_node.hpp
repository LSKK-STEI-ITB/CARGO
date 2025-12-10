#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include "cargo_core/command_registry.hpp"

struct MissionStep {
    std::string name;
    cargo_core::ParamMap params;
};

class MissionNode : public rclcpp::Node{
public:
    MissionNode();
    void runMission();
private:
    void loadMission(const std::string &file);

    std::vector<MissionStep> steps_;
};
