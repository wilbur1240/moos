// The MOOS ROS2 bridge
// Auther: Wilbur Lai, NYCU
// Date: April, 2025
// inspired from https://github.com/SyllogismRXS/moos-ros-bridge/tree/master

#ifndef _MOOS_NODE_H_ 
#define _MOOS_NODE_H_

#include <MOOS/libMOOS/App/MOOSApp.h>
#include "rclcpp/crlcpp.hpp"
#include <vector>
#include "MsgContainer.h"
#include <string.h>

class MOOSNode : public CMOOSApp { 
public:
     //standard construction and destruction
	MOOSNode();
	virtual ~MOOSNode();

	bool toMOOS(std::string moosName, double data);
	bool toMOOS(std::string moosName, std::string myString);
    bool toMOOSBinaryString(std::string moosName, std::string myString);
	void AssignPublisher(std::vector<MsgContainer> *msgVec);

protected:
	double appTick;
	double commsTick;

	//where we handle new mail
	bool OnNewMail(MOOSMSG_LIST &NewMail);

	//where we do the work
	bool Iterate();

	//called when we connect to the server
	bool OnConnectToServer();

	//called when we are starting up..
	bool OnStartUp();

void DoRegistrations();

private:
	vector<MsgContainer> *msgVec;

};
#endif