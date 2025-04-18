#include "moos-ros2-bridge/bridge_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>

using namespace rapidxml;

std::vector<std::shared_ptr<BridgeHandler>> parseBridgeConfig(
    const std::string& config_path,
    rclcpp::Node::SharedPtr node,
    MOOSNode* moos_node)
{
    std::vector<std::shared_ptr<BridgeHandler>> bridge_handlers;

    std::ifstream file(config_path);
    if (!file){
        throw std::runtime_error("Failed to open config file: " + config_path);
    }

    std::string xml_string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    std::vector<char> xml_data(xml_string.begin(), xml_string.end());
    xml_data.push_back('\0'); // Null-terminate the string

    xml_document<> doc;
    doc.parse<0>(&xml_data[0]);

    xml_node<>* root = doc.first_node("moosrosconfig");
    if (!root){
        throw std::runtime_error("Invalid XML format: Missing root node");
    }

    for (xml_node<>* topic = root->first_node("message"); topic; topic = topic->next_sibling("message")){
        std::string direction = topic->first_node("direction")->value();
        std::string moos_name = topic->first_node("moosname")->value();
        std::string ros_name = topic->first_node("rosname")->value();
        std::string ros_type = topic->first_node("rostype")->value();
        std::string moos_type = topic->first_node("moostype")->value();

        auto handler = createBridgeHandler(ros_type, node, ros_name, moos_name);
        if (!handler){
            RCLCPP_WARN(node->get_logger(), "Unsupported message type: %s", ros_type.c_str());
            continue;
        }

        handler->moosType = moos_type;
        handler->rosType = ros_type;

        if (direction == "toROS"){
            handler->direction = BridgeDirection::toROS;
            RCLCPP_INFO(node->get_logger(), "Creating publisher for MOOS->ROS: %s -> %s", moos_name.c_str(), ros_name.c_str());
        } else if (direction == "toMOOS"){
            handler->direction = BridgeDirection::toMOOS;
            handler->setupROSSubscriber(moos_node);
            RCLCPP_INFO(node->get_logger(), "Creating subscriber for ROS->MOOS: %s -> %s", ros_name.c_str(), moos_name.c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "Unsupported direction: %s", direction.c_str());
            continue;
        }
        bridge_handlers.push_back(handler);
    }
    return bridge_handlers;
}