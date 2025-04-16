#include "string_bridge_handler.h"
#include <iostream>

StringBridgeHandler::StringBridgeHandler(rclcpp::Node::SharedPtr node, const std::string& topic, const std::string& moos_var) : node_(node) {
    rosName = topic;
    moosName = moos_var;
    pub_ = node_->create_publisher<std_msgs::msg::String>(rosName, 10);
}

void StringBridgeHandler::publishFromMOOS(const std::string& data_str) {
    auto msg = std_msgs::msg::String();
    msg.data = data_str;
    pub_->publish(msg);
}

void StringBridgeHandler::setupROSSubscriber(CMOOSCommClient* comms) {
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        rosName, 10,
        [this, comms](std_msgs::msg::String::SharedPtr msg) {
            std::cout << "[ROS->MOOS] Notifying " << moosName << " = " << msg->data << std::endl;
            comms->Notify(moosName, msg->data);
        });
}