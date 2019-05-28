#include <Arduino.h>
#include <ChRt.h>

#include "include/wheel_speed.h"

#define SCALE 1
#define MAX_CHANGE 32768

int16_t speed;
int16_t prev_ticks=0;
uint16_t prev_time = chVTGetSystemTime();
/**
 * @brief This function reads the motor ticks that have been determined by
 * the hall sensor and converts this value into a speed for the truck
 *
 * @param ticks The number of ticks that is kept track of  by the hall_sensor
 * task.
 * @return the speed of the truck
 */
int16_t wheel_speed_loop_fn(int16_t ticks) {
    uint16_t current_time = chVTGetSystemTime();
    int16_t Current_tick = ticks;
    int16_t delta_t = current_time - prev_time;
    int16_t delta_ticks = Current_tick-prev_ticks;
    if (delta_t < 0){
        delta_t +=65535;
    }
    if ((delta_ticks)> MAX_CHANGE) {
        speed = SCALE * ((delta_ticks) + 65534);//delta_t;
    }else if ((delta_ticks)< -MAX_CHANGE){
        speed = SCALE * ((delta_ticks)-65536);//delta_t;
    }else{
        speed = SCALE*(delta_ticks);//delta_t;
    }
    prev_ticks = Current_tick;
    prev_time = current_time;

    return speed;
}


void wheel_speed_setup() {

}