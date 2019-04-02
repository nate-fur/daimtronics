#ifndef TEENSY_SERIAL_H
#define TEENSY_SERIAL_H

#include "system_data.h"

#define HWSERIAL Serial1
#define OFFSET 48

void teensy_serial_loop_fn(float imu_angle);

void teensy_serial_setup();

void set_sensor_msg(int user_input, sensor_data_t *data_ptr);

void read_from_pi(sensor_data_t *data_ptr);

void print_sensor_msg(sensor_data_t *data_ptr);

#endif
