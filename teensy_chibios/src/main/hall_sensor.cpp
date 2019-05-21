//
// Created by Fernando Mondragon-Cardenas on 2019-05-20.
//

#include "include/hall_sensor.h"
#include <Arduino.h>
#include <ChRt.h>


int16_t ticks = 0;
/**
 * This is the primary function controlling the wheel speed sensor. It reads an
 * IR sensor mounted so that the sensor reads an alternating black and white
 * strip of tape on the inside of the wheel rims. The frequency that the IR
 * sensor detects a change in reflection of the tape determines the speed of
 * the vehicle.
 *
 * @return the speed that the wheel speed sensor is detecting
 */
int16_t hall_sensor_loop_fn(short PhaseB_pin, short PhaseC_pin) {
    //int time = chVTGetSystemTime();
    //Serial.println(time);
    //Serial.print(digitalRead(PhaseB_pin));

    if((digitalRead(PhaseC_pin))){
        //FORWARD
        //Tick forward
        ticks +=1;
#ifdef DEBUG
        Serial.print(" FORWARDS ");
#endif
    }
    if((digitalRead(PhaseB_pin))){
        //REVERSE
        //Tick Reverse
#ifdef DEBUG
        Serial.print("REVERSE ");
#endif
        ticks -=1;
    }
    if (digitalRead(PhaseB_pin) && digitalRead(PhaseC_pin)) {
        //INVALID STATE
#ifdef DEBUG
        Serial.print("Both high ");
#endif
    }
    if (!digitalRead(PhaseB_pin) && !digitalRead(PhaseC_pin)) {
        //INVALID STATE
#ifdef DEBUG
        Serial.print("Both low ");
#endif
    }

#ifdef DEBUG
    Serial.print("Ticks = ");
    Serial.println(ticks);
#endif
    return ticks;
}


void hall_sensor_setup(short PhaseA_pin, short PhaseB_pin, short PhaseC_pin) {
    pinMode(PhaseA_pin, INPUT);
    pinMode(PhaseB_pin, INPUT);
    pinMode(PhaseC_pin, INPUT);
    delay(10);
}