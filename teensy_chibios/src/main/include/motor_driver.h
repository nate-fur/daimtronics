#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
void motor_driver_loop_fn(int16_t motor_output);

void motor_driver_setup();

#endif
