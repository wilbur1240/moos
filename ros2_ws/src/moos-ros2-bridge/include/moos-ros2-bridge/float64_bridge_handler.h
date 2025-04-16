#ifndef FLOAT64_BRIDGE_HANDLER_H
#define FLOAT64_BRIDGE_HANDLER_H

#include "bridge_handler.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class Float64BridgeHandler : public BridgeHandler {
public:
    Float64BridgeHandler(
        rclcpp::Node::SharedPtr node,
        const std::string& topic,
        const std::string& moos_var);

    void publishFromMOOS(const std::string& data_str) override;
    void setupROSSubscriber(CMOOSCommClient* comms) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};

#endif