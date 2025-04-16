#include "MOOSNode.h"
#include <iostream>

MOOSNode::MOOSNode() {
    // You can initialize default AppTick or CommsTick if desired
    m_AppFreq = 10.0;  // Hz
    m_CommsFreq = 10.0;
}

void MOOSNode::AssignBridgeHandlers(std::vector<std::shared_ptr<BridgeHandler>>* handlers) {
    bridgeHandlers_ = handlers;
}

bool MOOSNode::OnStartUp() {
    std::cout << "[MOOSNode] OnStartUp()" << std::endl;
    DoRegistrations();
    return true;
}

bool MOOSNode::OnConnectToServer() {
    std::cout << "[MOOSNode] Connected to MOOSDB" << std::endl;
    DoRegistrations();
    return true;
}

bool MOOSNode::OnNewMail(MOOSMSG_LIST &NewMail) {
    for (auto& msg : NewMail) {
        std::string key = msg.GetKey();
        std::string val = msg.GetString();

        std::cout << "[MOOS->ROS] Received: " << key << " = " << val << std::endl;

        if (bridgeHandlers_) {
            for (auto& handler : *bridgeHandlers_) {
                if (handler->moosName == key) {
                    handler->publishFromMOOS(val);
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

void MOOSNode::DoRegistrations() {
    if (bridgeHandlers_) {
        for (auto& handler : *bridgeHandlers_) {
            Register(handler->moosName, 0);  // 0 = no minimum time between updates
            std::cout << "[MOOSNode] Registered for: " << handler->moosName << std::endl;
        }
    }
}