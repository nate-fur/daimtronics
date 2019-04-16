//
// Created by nate on 4/2/19.
//

#include "include/fifth_wheel.h"
#include <Arduino.h>
#include <Servo.h>

#define FIFTH_WHEEL_PIN 13
#define LOCKED_ANGLE 0
#define UNLOCKED_ANGLE 180

/**
 * This is a Servo object to control the fifth wheel. It relies on code from
 * Servo.h which is built into the Arduino IDE.
 */
static Servo fifth_wheel_servo;

/**
 * The is the primary function controlling the fifth wheel. It reads the
 * fifth_output value form the system data and writes to the Servo for actuating
 * between the two different angles.
 *
 * @param fifth_output the output to the fifth wheel, which will be one of
 * two values, either locked or unlocked
 */
void fifth_wheel_loop_fn(short fifth_output) {

#ifdef DEBUG
   Serial.print("outputting to fifth wheel : ");
   Serial.println(fifth_output);
#endif

   if (fifth_output == UNLOCKED && fifth_wheel_servo.read() != UNLOCKED_ANGLE){
      fifth_wheel_servo.write(UNLOCKED_ANGLE);
   }
   else if (fifth_output == LOCKED && fifth_wheel_servo.read() != LOCKED_ANGLE){
      fifth_wheel_servo.write(LOCKED_ANGLE);
   }
   else {
      // error state?
   }
}

void fifth_wheel_setup() {
   fifth_wheel_servo.attach(FIFTH_WHEEL_PIN);
   //delay(15);
   fifth_wheel_servo.write(LOCKED_ANGLE);
}

