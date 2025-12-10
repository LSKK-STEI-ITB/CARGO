#include "cargo_safety/safety_checker.hpp"

// TODO: Add failsafe and stuff needed


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto nh = std::make_shared<SafetyChecker>();

  while (rclcpp::ok()) {

    // Create the executor to handle spinning (do this only once)
    rclcpp::executors::SingleThreadedExecutor executor;

    // Now we can call the checkPilot function to ensure the service is
    // available
    if (!nh->check_cargo_node()) {
      // callKillPilotService(); // Call the service if available
      RCLCPP_ERROR(nh->get_logger(), "Service is not available.");
      continue;
    }

    if (!nh->call_ready) {
      RCLCPP_INFO(nh->get_logger(), "--- Safety Node Ready ---");
      nh->call_ready = true;
    }

    int triggeredRC = nh->rc_trigger();
    if (triggeredRC) {
      RCLCPP_FATAL_STREAM(nh->get_logger(), "Initiating failsafe");
      if (nh->check_cargo_node()) {
        RCLCPP_ERROR(nh->get_logger(), "Pilot will be killed");
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        nh->call_kill_cargo_srv(); // Call the service if available
        rclcpp::sleep_for(std::chrono::seconds(5));
        rclcpp::spin_some(nh); // Update service availability
        nh->call_ready = false;
      }
    }
  }

  rclcpp::spin(nh);

  rclcpp::shutdown(); // Shutdown after spinning
}
