#include "moos-ros2-bridge/int32_bridge_handler.h"
#include "moos-ros2-bridge/MOOSNode.h"
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

void Int32BridgeHandler::setupROSSubscriber(MOOSNode* moos_node) {
    moosNode_ = moos_node;
    sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        rosName, 10,
        [this](std_msgs::msg::Int32::SharedPtr msg) {
            if(moosNode_){
                // std::cout << "[ROS->MOOS] Notifying " << moosName << " = " << msg->data << std::endl;
                moosNode_->NotifyFromROS(moosName, msg->data);
            }
        });
}