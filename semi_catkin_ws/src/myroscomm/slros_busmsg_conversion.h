#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <semi_truck/Teensy_Sensors.h>
#include "myROScomm_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(semi_truck::Teensy_Sensors* msgPtr, SL_Bus_myROScomm_semi_truck_Teensy_Sensors const* busPtr);
void convertToBus(SL_Bus_myROScomm_semi_truck_Teensy_Sensors* busPtr, semi_truck::Teensy_Sensors const* msgPtr);


#endif
