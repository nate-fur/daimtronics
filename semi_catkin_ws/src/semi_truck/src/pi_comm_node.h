#ifndef TEENSY_COMM_NODE_H
#define TEENSY_COMM_NODE_H

#include "semi_truck/Teensy_Sensors.h"
#include "semi_truck/Teensy_Actuators.h"


// Functions for serial communication over UART
short read_sensor_msg(int serial);

void read_from_teensy(int serial, semi_truck::Teensy_Sensors &sensors);

void write_sensor_msg(int serial, short sensor_val);

void write_to_teensy(int serial, const semi_truck::Teensy_Sensors &sensors);

void update_sensors(semi_truck::Teensy_Sensors &sensors);

void print_sensors(const semi_truck::Teensy_Sensors &sensors);

#endif
