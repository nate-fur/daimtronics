//
// Created by nate on 4/2/19.
//
#include <Arduino.h>
#include <ChRt.h>

#include "include/wheel_speed.h"

int16_t speed;
int16_t prev_ticks=0;
int16_t scale = 1;
int16_t Current_tick;
int16_t current_time = chVTGetSystemTime();
int16_t prev_time;
/**
 * This is the primary function controlling the wheel speed sensor. It reads an
 * IR sensor mounted so that the sensor reads an alternating black and white
 * strip of tape on the inside of the wheel rims. The frequency that the IR
 * sensor detects a change in reflection of the tape determines the speed of
 * the vehicle.
 *
 * @return the speed that the wheel speed sensor is detecting
 */
int16_t wheel_speed_loop_fn(int16_t ticks) {
    current_time = chVTGetSystemTime();
    Current_tick = ticks;
    speed = scale*(Current_tick - prev_ticks);

    prev_ticks = Current_tick;
    prev_time = current_time;

    return speed;
}


void wheel_speed_setup() {

}