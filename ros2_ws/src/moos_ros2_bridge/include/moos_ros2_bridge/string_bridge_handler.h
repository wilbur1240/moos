#ifndef STRING_BRIDGE_HANDLER_H
#define STRING_BRIDGE_HANDLER_H

#include "moos_ros2_bridge/bridge_handler.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class StringBridgeHandler : public BridgeHandler {
public:
    StringBridgeHandler(rclcpp::Node::SharedPtr node, const std::string& topic, const std::string& moos_var);

    void publishFromMOOS(const std::string& data_str) override;
    void setupROSSubscriber(MOOSNode* moos_node) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

#endif