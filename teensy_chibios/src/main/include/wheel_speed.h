#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>
int16_t wheel_speed_loop_fn(short PhaseB_pin);

void wheel_speed_setup();

#endif