#include "bridge_factory.h"

std::shared_ptr<BridgeHandler> createBridgeHandler(
    const std::string& ros_type,
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    const std::string& moos_var)
{
    if (ros_type == "std_msgs/Float64") {
        return std::make_shared<Float64BridgeHandler>(node, topic, moos_var);
    }

    // Add more types here
    return nullptr;
}