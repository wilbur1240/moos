#include "moos_ros2_bridge/float32_bridge_handler.h"
#include "moos_ros2_bridge/MOOSNode.h"
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

void Float32BridgeHandler::setupROSSubscriber(MOOSNode* moos_node) {
    moosNode_ = moos_node;
    sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        rosName, 10,
        [this](std_msgs::msg::Float32::SharedPtr msg) {
            if (moosNode_){
                // std::cout << "[ROS->MOOS] Notifying " << moosName << " = " << msg->data << std::endl;
                moosNode_->NotifyFromROS(moosName, msg->data);
            }
        });
}