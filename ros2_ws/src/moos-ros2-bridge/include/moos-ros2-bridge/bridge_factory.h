#ifndef BRIDGE_FACTORY_H
#define BRIDGE_FACTORY_H

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "bridge_handler.h"
#include "float64_bridge_handler.h"
//add more handlers here

std::shared_ptr<BridgeHandler> createBridgeHandler(
    const std::string& ros_type,
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    const std::string& moos_var);

#endif