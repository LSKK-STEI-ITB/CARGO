#include "cargo_mission/mission_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>();
    node->runMission();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
