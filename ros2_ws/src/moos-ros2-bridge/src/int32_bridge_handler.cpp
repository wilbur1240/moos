#include "int32_bridge_handler.h"
#include <iostream>

Int32BridgeHandler::Int32BridgeHandler(rclcpp::Node::SharedPtr node, const std::string& topic, const std::string& moos_var) : node_(node){
    rosName = topic;
    moosName = moos_var;
    pub_ = node_->create_publisher<std_msgs::msg::Int32>(rosName, 10);
}

void Int32BridgeHandler::publishFromMOOS(const std::string& data_str) {
    auto msg = std_msgs::msg::Int32();
    msg.data = std::stoi(data_str);
    pub_->publish(msg);
}

void Int32BridgeHandler::setupROSSubscriber(CMOOSCommClient* comms) {
    sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        rosName, 10,
        [this, comms](std_msgs::msg::Int32::SharedPtr msg) {
            std::cout << "[ROS->MOOS] Notifying " << moosName << " = " << msg->data << std::endl;
            comms->Notify(moosName, msg->data);
        });
}