//
// Created by nate on 4/2/19.
//

#include "include/range_finder.h"
#include <Arduino.h>
#include <Servo.h>

#define MOTOR_PIN 13
#define STOP 0
/**
 * This is a Servo object to control the steering servo. It relies on code from
 * Servo.h which is built into the Arduino IDE.
 */
static Servo motor;

/**
 * This is the primary function controlling the motor. It reads the
 * motor output value from the system data and controls the motor with this
 * value.
 *
 * @param motor_output the output to the motor
 */
void motor_driver_loop_fn(short motor_output) {
   // if output is out of the 0-180 range, stop the motor
   Serial.print("outputting to motor: ");
   Serial.println(motor_output);

   if (motor_output > 180 || motor_output < 0) {
      motor.write(STOP);
   }
   else if (motor_output != motor.read()) {
      motor.write(motor_output);
   }
}

void motor_driver_setup() {
   motor.attach(MOTOR_PIN);
   // delay(15);
   motor.write(STOP);
}

