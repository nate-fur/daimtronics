#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block myROScomm/Subscribe
extern SimulinkSubscriber<semi_truck::Teensy_Sensors, SL_Bus_myROScomm_semi_truck_Teensy_Sensors> Sub_myROScomm_1;

// For Block myROScomm/Publish
extern SimulinkPublisher<semi_truck::Teensy_Sensors, SL_Bus_myROScomm_semi_truck_Teensy_Sensors> Pub_myROScomm_9;

void slros_node_init(int argc, char** argv);

#endif
