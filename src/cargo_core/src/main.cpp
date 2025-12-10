#include "cargo_core/cargo_node.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CargoNode>();
    rclcpp::executors::MultiThreadedExecutor executor; 

    executor.add_node(node); 

    // Spin the executor in a separate thread
    std::thread executor_thread([&executor]() { executor.spin(); });

    // Wait for the executor thread to finish
    executor_thread.join();

    rclcpp::shutdown(); 
    return 0; 
}
