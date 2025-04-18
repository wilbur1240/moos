#ifndef INT32_BRIDGE_HANDLER_H
#define INT32_BRIDGE_HANDLER_H

#include "moos-ros2-bridge/bridge_handler.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class Int32BridgeHandler : public BridgeHandler {
public:
    Int32BridgeHandler(rclcpp::Node::SharedPtr node, const std::string& topic, const std::string& moos_var);

    void publishFromMOOS(const std::string& data_str) override;
    void setupROSSubscriber(MOOSNode* moos_node) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

#endif