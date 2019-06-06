#ifndef TOF_LIDAR_H
#define TOF_LIDAR_H

#include <Adafruit_VL53L0X.h>

int16_t tof_left_loop_fn();

int16_t tof_right_loop_fn();

void tof_lidar_setup();

#endif //TOF_LIDAR_H
