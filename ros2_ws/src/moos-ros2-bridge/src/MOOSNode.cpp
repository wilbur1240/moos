#include "moos-ros2-bridge/MOOSNode.h"
#include <iostream>

MOOSNode::MOOSNode() {
    // You can initialize default AppTick or CommsTick if desired
    appTick = 10.0;  // Default app tick rate
    commsTick = 10.0;  // Default comms tick rate
}

void MOOSNode::AssignBridgeHandlers(std::vector<std::shared_ptr<BridgeHandler>>* handlers) {
    bridgeHandlers_ = handlers;
}

bool MOOSNode::NotifyFromROS(const std::string& var, const std::string& val) {
    return m_Comms.Notify(var, val);
}

bool MOOSNode::NotifyFromROS(const std::string& var, double val) {
    return m_Comms.Notify(var, val);
}

bool MOOSNode::OnStartUp() {
    std::cout << "[MOOSNode] OnStartUp()" << std::endl;
    DoRegisterations();
    return true;
}

bool MOOSNode::OnConnectToServer() {
    std::cout << "[MOOSNode] Connected to MOOSDB" << std::endl;
    DoRegisterations();
    return true;
}

bool MOOSNode::OnNewMail(MOOSMSG_LIST &NewMail) {
    for (auto& msg : NewMail) {
        std::string key = msg.GetKey();
        
        if (bridgeHandlers_) {
            for (auto& handler : *bridgeHandlers_){
                if (handler->moosName == key) {
                    if (msg.IsString()) {
                        std::string val = msg.GetString();
                        // std::cout << "[MOOS->ROS] Received " << key << " (string) = " << val << std::endl;
                        handler->publishFromMOOS(val);
                    }else if (msg.IsDouble()){
                        double val = msg.GetDouble();
                        // std::cout << "[MOOS->ROS] Received " << key << " (double) = " << val << std::endl;
                        handler->publishFromMOOS(std::to_string(val));
                    }else {
                        std::cout << "[MOOS->ROS] Unsupported type for: " << key << std::endl;
                    }
                }
            }
        }
    }

    return true;
}

bool MOOSNode::Iterate() {
    // Called at m_AppFreq
    // Can be used for periodic tasks (e.g. publishing time or status to ROS2)

    return true;
}

void MOOSNode::DoRegisterations() {
    if (bridgeHandlers_) {
        for (auto& handler : *bridgeHandlers_) {
            Register(handler->moosName, 0);  // 0 = no minimum time between updates
            std::cout << "[MOOSNode] Registered for: " << handler->moosName << std::endl;
        }
    }
}