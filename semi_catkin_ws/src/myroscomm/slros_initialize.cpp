#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "myROScomm";

// For Block myROScomm/Subscribe
SimulinkSubscriber<semi_truck::Teensy_Sensors, SL_Bus_myROScomm_semi_truck_Teensy_Sensors> Sub_myROScomm_1;

// For Block myROScomm/Publish
SimulinkPublisher<semi_truck::Teensy_Sensors, SL_Bus_myROScomm_semi_truck_Teensy_Sensors> Pub_myROScomm_9;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

