//
// Created by Fernando Mondragon-Cardenas on 2019-05-20.
//

#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <stdint.h>
int16_t hall_sensor_loop_fn(short PhaseB_pin, short PhaseC_pin);

void hall_sensor_setup(short PhaseA_pin, short PhaseB_pin, short PhaseC_pin);

#endif //HALL_SENSOR_H
