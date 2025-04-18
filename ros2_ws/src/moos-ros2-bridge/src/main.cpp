#include "moos-ros2-bridge/Bridge.h"
#include "moos-ros2-bridge/MOOSNode.h"
#include <rclcpp/rclcpp.hpp>
#include <MOOS/libMOOS/App/MOOSApp.h>
#include <thread>
#include <memory>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

extern CMOOSApp* initMOOSApp();

std::shared_ptr<CMOOSApp> moos_app;

void run_moos(const std::string& mission_file){
    moos_app->Run("MOOS_ROS_Bridge", mission_file.c_str());
}

CMOOSApp* initMOOSApp() {
    return new MOOSNode();
}

int main(int argc, char* argv[]){
    if (argc < 3) {
        std::cerr << "Usage: ros2 run moos_ros2_bridge main <bridge_config.xml> <mission.moos>" << std::endl;
        return 1;
    }

    std::string config_file = argv[1];
    std::string moos_file = argv[2];

    struct stat buffer;
    if (stat(config_file.c_str(), &buffer) != 0) {
        std::cerr << "Bridge config not found: " << config_file << std::endl;
        return 1;
    }
    if (stat(moos_file.c_str(), &buffer) != 0) {
        std::cerr << "MOOS mission file not found: " << moos_file << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moos_ros2_bridge");

    // Init MOOS app
    moos_app.reset(initMOOSApp());

    // Start MOOS in background thread
    std::thread moos_thread(run_moos, moos_file);

    // get moos_node
    MOOSNode* moos_node_ptr = dynamic_cast<MOOSNode*>(moos_app.get());

    // Parse bridge config and set up handlers
    auto handlers = parseBridgeConfig(config_file, node, moos_node_ptr);

    // Provide handlers to MOOSApp
    moos_node_ptr->AssignBridgeHandlers(&handlers);

    rclcpp::spin(node);

    moos_thread.join();
    rclcpp::shutdown();
    return 0;
}