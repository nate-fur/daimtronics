//
// Created by nate on 4/2/19.
//
#include <Arduino.h>
#include "include/wheel_speed.h"

unsigned long prev_time = 0;
int32_t speed;

/**
 * This is the primary function controlling the wheel speed sensor. It reads an
 * IR sensor mounted so that the sensor reads an alternating black and white
 * strip of tape on the inside of the wheel rims. The frequency that the IR
 * sensor detects a change in reflection of the tape determines the speed of
 * the vehicle.
 *
 * @return the speed that the wheel speed sensor is detecting
 */
int32_t wheel_speed_loop_fn(short PhaseB_pin, short PhaseC_pin) {
   unsigned long time = micros();
   uint32_t t_time = time-prev_time;
   //Serial.println("Times = ");
   //Serial.println(prev_time);
   //Serial.println(t_time);
   //Serial.println(time);
   bool phaseB_val = digitalRead(PhaseB_pin);
   bool phaseC_val = digitalRead(PhaseC_pin);
   Serial.print("phaseB_val = ");
   Serial.println(phaseB_val);
   Serial.print("phaseC_val = ");
   Serial.println(phaseC_val);

    if(phaseB_val){
       //Going Forward
       speed = t_time;//5000/t_time;
       //Serial.print("Wheel_Speed = ");
       //Serial.println(speed);

   } else if(not(phaseB_val)){
       //Going Backward
       speed = -t_time;//-(5000/t_time);
       //Serial.print("Wheel_Speed_neg = ");
       //Serial.println(speed);
   }
   else {
       speed = 0;
   }
   //Serial.print("Speed = ");
   //Serial.println(speed);
   prev_time = time;
   return speed; // placeholder for compilation
}

void wheel_speed_setup(short PhaseA_pin, short PhaseB_pin, short PhaseC_pin) {
    pinMode(PhaseA_pin, INPUT_PULLUP);
    pinMode(PhaseB_pin, INPUT_PULLUP);
    pinMode(PhaseC_pin, INPUT_PULLUP);
    delay(10);
}