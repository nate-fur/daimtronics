#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>
int16_t wheel_speed_loop_fn(int16_t ticks);

void wheel_speed_setup();

#endif