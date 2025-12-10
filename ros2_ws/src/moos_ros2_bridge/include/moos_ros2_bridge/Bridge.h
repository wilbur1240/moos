#ifndef BRIDGE_H
#define BRIDGE_H

#include "moos_ros2_bridge/bridge_handler.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

std::vector<std::shared_ptr<BridgeHandler>> parseBridgeConfig(
    const std::string& config_path,
    rclcpp::Node::SharedPtr node,
    MOOSNode* moos_node);

#endif