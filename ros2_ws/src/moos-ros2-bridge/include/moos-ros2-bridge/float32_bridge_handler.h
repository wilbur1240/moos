#ifndef FLOAT32_BRIDGE_HANDLER_H
#define FLOAT32_BRIDGE_HANDLER_H

#include "bridge_handler.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class Float32BridgeHandler : public BridgeHandler {
public:
    Float32BridgeHandler(rclcpp::Node::SharedPtr node, const std::string& topic, const std::string& moos_var);

    void publicFromMOOS(const std::string& data_str) override;
    void setupROSSubscriber(CMOOSCommClient* comms) override;

private:
    rclcpp::Node::SharePtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharePtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharePtr sub_;
};

#endif