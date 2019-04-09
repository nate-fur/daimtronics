//
// Created by nate on 4/2/19.
//

#include "include/steer_servo.h"
#include <Servo.h>
#include <Arduino.h>

#define STEER_SERVO_PIN 13
#define STRAIGHT 90

/**
 * This is a Servo object to control the fifth wheel. It relies on code from
 * Servo.h which is built into the Arduino IDE.
 */
static Servo steer_servo;

/**
 * The is the primary function controlling the steering servo wheel. It reads a
 * value form the system data and controls the steering servo based
 * on what the system data contains.
 *
 * @param steer_output the output to the steering servo that controls angle
 */
void steer_servo_loop_fn(short steer_output) {
   // if output is out of the 0-180 degree range, drive straight
   Serial.print("outputting to steer servo: ");
   Serial.println(steer_output);
   if (steer_output > 180 || steer_output < 0) {
      steer_servo.write(STRAIGHT);
   }
   else if (steer_output != steer_servo.read()) {
      steer_servo.write(steer_output);
   }
}

void steer_servo_setup() {
   steer_servo.attach(STEER_SERVO_PIN);
   // delay(15);
   steer_servo.write(STRAIGHT);

}
