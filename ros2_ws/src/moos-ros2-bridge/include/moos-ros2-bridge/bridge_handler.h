#ifndef BRIDGE_HANDLER_H
#define BRIDGE_HANDLER_H

#include <string>

enum class BridgeDirection {
    toROS,
    toMOOS
};

class BridgeHandler {
public:
    virtual ~BridgeHandler() = default;

    virtual void publishFromMOOS(const std::string& data_str) = 0;
    virtual void setupROSSubscriber() = 0;

    std::string moosName;
    std::string rosName;
    std::string moosType;
    std::string rosType;

    BridgeDirection direction;
};

#endif