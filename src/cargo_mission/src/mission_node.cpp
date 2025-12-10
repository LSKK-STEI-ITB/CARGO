#include "cargo_mission/mission_node.hpp"
#include <fstream>
#include <sstream>
#include "cargo_core/cargo_control.hpp"
#include "cargo_core/command_registry.hpp"

MissionNode::MissionNode()
    : Node("cargo_mission")
{
    cargo_core::register_all_commands();

    std::string mission_file =
        declare_parameter("mission_file", "mission.txt");

    loadMission(mission_file);
}

void MissionNode::loadMission(const std::string &file)
{
    std::ifstream in(file);
    if (!in.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open mission file");
        return;
    }

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;

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

void MissionNode::runMission()
{
    for (auto &step : steps_) {
        try {
            cargo_core::execute_command(step.name, shared_from_this(), step.params);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "--- Mission Completed ---"); 

}
