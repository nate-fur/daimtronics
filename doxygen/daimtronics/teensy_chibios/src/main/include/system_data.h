#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdint.h>

typedef struct sensor_data_t {
   int16_t imu_angle;
   int16_t wheel_speed;
   int16_t right_TOF;
   int16_t left_TOF;
   int16_t rear_TOF;
} sensor_data_t;


typedef struct actuator_data_t {
   int16_t motor_output;
   int16_t steer_output;
   int16_t fifth_output;
} actuator_data_t;


typedef struct system_data_t {
   bool updated;
   int16_t deadman;
   int16_t drive_mode;
   sensor_data_t sensors;
   actuator_data_t actuators;
} system_data_t;

#endif
