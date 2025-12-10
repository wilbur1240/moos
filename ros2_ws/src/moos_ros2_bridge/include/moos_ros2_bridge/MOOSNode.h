#ifndef MOOS_NODE_H
#define MOOS_NODE_H

#include "MOOS/libMOOS/App/MOOSApp.h"
#include <vector>
#include <memory>
#include <string>

#include "moos_ros2_bridge/bridge_handler.h"

class MOOSNode : public CMOOSApp {
public:
    MOOSNode();
    virtual ~MOOSNode() = default;

    //Assign handler vector from bridge config
    void AssignBridgeHandlers(std::vector<std::shared_ptr<BridgeHandler>>* handlers);

    // MOOS overrides
    bool OnNewMail(MOOSMSG_LIST &NewMail) override;
    bool Iterate() override;
    bool OnConnectToServer() override;
    bool OnStartUp() override;

    // ROS->MOOS Notify
    bool NotifyFromROS(const std::string& var, const std::string& val);
    bool NotifyFromROS(const std::string& var, double val);

protected:
    double appTick;
    double commsTick;

private:
    void DoRegisterations();

    std::vector<std::shared_ptr<BridgeHandler>>* bridgeHandlers_ = nullptr;
};

#endif 