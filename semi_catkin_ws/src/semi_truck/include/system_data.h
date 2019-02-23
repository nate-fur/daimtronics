//
// Created by nate on 2/20/19.
//

#ifndef DAIMTRONICS_SYSTEM_DATA_H
#define DAIMTRONICS_SYSTEM_DATA_H

/**
 * @brief a data structure that holds all of the information of what state the
 * semi-truck is in at any given time.
 * @var motor_output the output sent to the motor driver
 * @var steer_output output sent to the steering servo
 * @var wheel_speed speed that the wheel speed sensor is recording
 * @var imu_angle euler angle read by the BNO055 IMU
 * @var actual_gear desired gear level (set by the remote control device)
 * @var desired_5th desired state of the 5th wheel (either locked or unlocked)
 * @var actual_5th actual state of the 5th wheel
 */
struct system_data_t {
	int32_t  motor_output;   // output sent to the motor driver
    int32_t  steer_output;   // output sent to the steering servo
    int32_t  wheel_speed;    // speed that the wheel speed sensor is recording
    int32_t  imu_angle;     // euler angle read by the BNO055 IMU (degrees)
    int32_t  right_URF;     // distance of nearest object for the right URF
	int32_t  left_URF;      // distance of nearest object for the left URF
    int32_t  desired_5th;    // desired state for 5th wheel (locked or unlocked)
    int32_t  actual_5th;     // actual state of the 5th wheel
};

#endif //DAIMTRONICS_SYSTEM_DATA_H
