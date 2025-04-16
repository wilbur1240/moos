#include "float32_bridge_handler.h"
#include <iostream>

Float32BridgeHandler::Float32BridgeHandler(rclcpp::Node::SharedPtr node, const std::string& topic, const std::string& moos_var) : node_(node){
    rosName = topic;
    moosName = moos_var;
    pub_ = node_->create_publisher<std_msgs::msg::Float32>(rosName, 10);
}

void Float32BridgeHandler::publishFromMOOS(const std::string& data_str) {
    auto msg = std_msgs::msg::Float32();
    msg.data = std::stof(data_str);
    pub_->publish(msg);
}

void Float32BridgeHandler::setupROSSubscriber(CMOOSCommClient* comms) {
    sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        rosName, 10,
        [this, comms](std_msgs::msg::Float32::SharedPtr msg) {
            std::cout << "[ROS->MOOS] Notifying " << moosName << " = " << msg->data << std::endl;
            comms->Notify(moosName, msg->data);
        });
}