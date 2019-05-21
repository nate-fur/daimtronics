#include "../include/semi_truck_api.h"

void set_motor_output(semi_truck::Teensy_Actuators &actuators, int16_t motor_output){
   actuators.motor_output = motor_output;
}

void set_steer_output(semi_truck::Teensy_Actuators &actuators, int16_t steer_output) {
   actuators.steer_output = steer_output;
}

void set_fifth_output(semi_truck::Teensy_Actuators &actuators, uint16_t fifth_output) {
   actuators.fifth_output = fifth_output;
}

int16_t get_wheel_speed(semi_truck::Teensy_Sensors &sensors) {
   return sensors.wheel_speed;
}

int16_t get_imu_angle(semi_truck::Teensy_Sensors &sensors) {
   return sensors.imu_angle;
}

int16_t get_right_TOF(semi_truck::Teensy_Sensors &sensors) {
   return sensors.right_TOF;
}

int16_t get_left_TOF(semi_truck::Teensy_Sensors &sensors) {
   return sensors.left_TOF;
}

int16_t get_rear_TOF(semi_truck::Teensy_Sensors &sensors) {
   return sensors.rear_TOF;
}

