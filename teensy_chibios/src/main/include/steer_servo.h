#ifndef STEER_SERVO_H
#define STEER_SERVO_H

#include <stdint.h>
void steer_servo_loop_fn(int16_t steer_output);

void steer_servo_setup(short servo_pin);

#endif