#include "../include/semi_truck_api.h"

void set_motor_output(TeensyActuators &actuators, int16_t motor_output){
   actuators.motor_output = motor_output;
}

void set_steer_output(TeensyActuators &actuators, int16_t steer_output) {
   actuators.steer_output = steer_output;
}

void set_fifth_output(TeensyActuators &actuators, uint16_t fifth_output) {
   actuators.fifth_output = fifth_output;
}

int16_t get_wheel_speed(TeensySensors &sensors) {
   return sensors.wheel_speed;
}

int16_t get_imu_angle(TeensySensors &sensors) {
   return sensors.imu_angle;
}

int16_t get_right_TOF(TeensySensors &sensors) {
   return sensors.right_TOF;
}

int16_t get_left_TOF(TeensySensors &sensors) {
   return sensors.left_TOF;
}

int16_t get_rear_TOF(TeensySensors &sensors) {
   return sensors.rear_TOF;
}

