#ifndef FIFTH_WHEEL_H
#define FIFTH_WHEEL_H

#include <stdint.h>

#define LOCKED 1
#define UNLOCKED 0

void fifth_wheel_loop_fn(int16_t fifth_output);

void fifth_wheel_setup(short fifth_wheel_pin);

#endif //FIFTH_WHEEL_H
