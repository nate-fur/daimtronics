#ifndef RANGE_FINDER_H
#define RANGE_FINDER_H

#include <stdint.h>
long range_finder_loop_fn(short urf_echo_pin);

void range_finder_ping(short urf_trig_pin);

void range_finder_setup(short urf_trig_pin);

#endif
