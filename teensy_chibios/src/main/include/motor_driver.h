#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
void motor_driver_loop_fn(int16_t motor_output);

void motor_driver_setup(short motor_pin);

int16_t stop_motor(int16_t wheel_speed, int16_t time_step);
#endif
