#ifndef PI_COMM_NODE_H
#define PI_COMM_NODE_H

#include "semi_truck/Teensy_Sensors.h"
#include "semi_truck/Teensy_Actuators.h"


// Functions for serial communication over UART
short read_sensor_msg(int serial, char num_bytes);

void read_from_teensy(int serial, semi_truck::Teensy_Sensors &sensors);

void write_sensor_msg(int serial, short sensor_val, char num_bytes);

void write_to_teensy(int serial, const semi_truck::Teensy_Actuators &actuators);

void update_sensors(semi_truck::Teensy_Sensors &sensors);

bool pi_sync();

void print_sensors(const semi_truck::Teensy_Sensors &sensors);

void print_actuators(const semi_truck::Teensy_Actuators &actuators);

void sensor_cb(const semi_truck::Teensy_Sensors &msg);

void actuator_cb(const semi_truck::Teensy_Actuators &msg);

#endif
