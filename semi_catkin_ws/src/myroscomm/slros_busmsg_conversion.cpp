#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_myROScomm_semi_truck_Teensy_Sensors and semi_truck::Teensy_Sensors

void convertFromBus(semi_truck::Teensy_Sensors* msgPtr, SL_Bus_myROScomm_semi_truck_Teensy_Sensors const* busPtr)
{
  const std::string rosMessageType("semi_truck/Teensy_Sensors");

  msgPtr->imu_angle =  busPtr->ImuAngle;
  msgPtr->left_URF =  busPtr->LeftURF;
  msgPtr->right_URF =  busPtr->RightURF;
  msgPtr->wheel_speed =  busPtr->WheelSpeed;
}

void convertToBus(SL_Bus_myROScomm_semi_truck_Teensy_Sensors* busPtr, semi_truck::Teensy_Sensors const* msgPtr)
{
  const std::string rosMessageType("semi_truck/Teensy_Sensors");

  busPtr->ImuAngle =  msgPtr->imu_angle;
  busPtr->LeftURF =  msgPtr->left_URF;
  busPtr->RightURF =  msgPtr->right_URF;
  busPtr->WheelSpeed =  msgPtr->wheel_speed;
}

