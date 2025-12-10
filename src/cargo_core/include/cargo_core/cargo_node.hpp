#pragma once 
#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include "std_srvs/srv/empty.hpp"
#include <std_srvs/srv/detail/empty__struct.hpp>

enum ReturnInfo{
    OK, 
    WARNING, 
    ERROR,  // Error exist in code 
    CRITICAL, // High level warning based on the return (weird value, etc)
    FAILED, // Somehow it not working (not error) perhaps like return nothing etc 
};

class CargoNode: public rclcpp::Node{
public: 
    CargoNode(); 

private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cargo_service; 

    void kill_node_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response); 



};
