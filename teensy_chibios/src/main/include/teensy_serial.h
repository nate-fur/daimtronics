#ifndef TEENSY_SERIAL_H
#define TEENSY_SERIAL_H

#include "system_data.h"

#define HWSERIAL Serial1

void teensy_serial_loop_fn(system_data_t *system_data);

void teensy_serial_setup();

void set_sensor_msg(int user_input, sensor_data_t *data_ptr);

void read_from_pi(actuator_data_t *actuators_ptr);

void print_sensor_msg(sensor_data_t *sensors_ptr);

void print_actuator_msg(actuator_data_t *actuators_ptr);

#endif
