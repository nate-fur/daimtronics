#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>
int32_t wheel_speed_loop_fn(short PhaseB_pin, short PhaseC_pin);

void wheel_speed_setup(short PhaseA_pin, short PhaseB_pin, short PhaseC_pin);

#endif