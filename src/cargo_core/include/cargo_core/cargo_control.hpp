// Basic movement and simple task (wait, init, etc)

#include "cargo_core/cargo_node.hpp"

#include "cargo_core/command_registry.hpp"
#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace cargo_core {

// TODO: somehow remove passing node (or node pointer) from parameter (bad code)

void initialize(rclcpp::Node::SharedPtr nh, const ParamMap &params);
void arm_drone(rclcpp::Node::SharedPtr nh, const ParamMap &params);
void takeoff(rclcpp::Node::SharedPtr nh, const ParamMap &params);
void switch_mode(rclcpp::Node::SharedPtr nh, const ParamMap &params);
void move_local(rclcpp::Node::SharedPtr nh, const ParamMap &params);
void wait(rclcpp::Node::SharedPtr nh, const ParamMap &params);

// NOTE: Function without params need to add paramMap to fit the desc
void land(rclcpp::Node::SharedPtr nh, const ParamMap &params);

void get_alt(rclcpp::Node::SharedPtr nh, double &alt, std::string source);

template <class T>
bool get_topic_val(rclcpp::Node::SharedPtr nh, T &return_val,
                   const std::string &topic_name,
                   std::chrono::seconds retry_timeout);
void register_all_commands();
}; // namespace cargo_core
