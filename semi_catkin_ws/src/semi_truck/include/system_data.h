//
// Created by nate on 2/20/19.
//

#ifndef DAIMTRONICS_SYSTEM_DATA_H
#define DAIMTRONICS_SYSTEM_DATA_H
#include <stdint.h>

/**
 * @brief a data structure that holds all of the information of what state the
 * semi-truck is in at any given time.
 * @var wheel_speed speed that the wheel speed sensor is recording
 * @var imu_angle euler angle read by the BNO055 IMU in degrees
 * @var right_URF distance of nearest object for the right URF
 * @var left_URF distance of nearest object for the left URF
 * @var rear_URF distance of nearest object for the rear URF
 * @var motor_output the output sent to the motor driver
 * @var steer_output output sent to the steering servo
 * @var fifth_output desired state of the 5th wheel (either locked or unlocked)
 */
struct system_data_t {
   int16_t  wheel_speed;    // speed that the wheel speed sensor is recording
   float    imu_angle;      // euler angle read by the BNO055 IMU (degrees)
   int16_t  right_URF;      // distance of nearest object for the right URF
   int16_t  left_URF;       // distance of nearest object for the left URF
   int16_t  read_URF;       // distance of nearest object for the rear URF
   int16_t  motor_output;   // output sent to the motor driver
   int16_t  steer_output;   // output sent to the steering servo
   int16_t  fifth_output;     // actual state of the 5th wheel
};

#endif //DAIMTRONICS_SYSTEM_DATA_H
