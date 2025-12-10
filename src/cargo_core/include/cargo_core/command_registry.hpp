#pragma once
#include <string>
#include <unordered_map>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace cargo_core {

using ParamValue = std::variant<double, std::string>;
using ParamMap   = std::unordered_map<std::string, ParamValue>;
using CommandFunction = std::function<void(rclcpp::Node::SharedPtr, const ParamMap&)>;

class CommandRegistry {
public:
    static CommandRegistry& instance()
    {
        static CommandRegistry registry;
        return registry;
    }

    void register_command(const std::string &name, CommandFunction fn)
    {
        registry_[name] = fn;
    }

    bool exists(const std::string &name) const
    {
        return registry_.count(name) > 0;
    }

    void execute(const std::string &name, rclcpp::Node::SharedPtr nh, const ParamMap &params) {
        registry_.at(name)(nh, params); 
    }

private:
    std::unordered_map<std::string, CommandFunction> registry_;
};

inline void register_command(const std::string &name, CommandFunction fn)
{
    CommandRegistry::instance().register_command(name, fn);
}

inline void execute_command(const std::string &name, rclcpp::Node::SharedPtr nh, const ParamMap &params)
{
    CommandRegistry::instance().execute(name, nh, params);  
}

} // namespace cargo_core

