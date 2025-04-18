#ifndef BRIDGE_FACTORY_H
#define BRIDGE_FACTORY_H

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "moos-ros2-bridge/bridge_handler.h"
#include "moos-ros2-bridge/float64_bridge_handler.h"
#include "moos-ros2-bridge/float32_bridge_handler.h"
#include "moos-ros2-bridge/int32_bridge_handler.h"
#include "moos-ros2-bridge/string_bridge_handler.h"
//add more handlers here

std::shared_ptr<BridgeHandler> createBridgeHandler(
    const std::string& ros_type,
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    const std::string& moos_var);

#endif