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
 * @var int16_t distance of nearest object for the right TOF
 * @var left_TOF distance of nearest object for the left TOF
 * @var rear_TOF distance of nearest object for the rear TOF
 * @var motor_output the output sent to the motor driver
 * @var steer_output output sent to the steering servo
 * @var fifth_output desired state of the 5th wheel (either locked or unlocked)
 */
struct system_data_t {
   int16_t   wheel_speed;    // speed that the wheel speed sensor is recording
   int16_t   imu_angle;      // euler angle read by the BNO055 IMU (degrees)
   uint16_t  right_TOF;     // distance of nearest object for the right TOF
   uint16_t  left_TOF;      // distance of nearest object for the left TOF
   uint16_t  rear_TOF;      // distance of nearest object for the rear TOF
   uint16_t  drive_mode_1;
   uint16_t  drive_mode_2;
   int16_t   motor_output;   // output sent to the motor driver
   int16_t   steer_output;   // output sent to the steering servo
   uint16_t  fifth_output;  // actual state of the 5th wheel
};

#endif //DAIMTRONICS_SYSTEM_DATA_H
