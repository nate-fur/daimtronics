//
// Created by nate on 4/2/19.
//

#include "include/motor_driver.h"
#include <Arduino.h>
#include <Servo.h>

#define WHEEL_SPEED_STOP 0 // wheel speed of 0 is no velocity
#define MOTOR_STOP 90 // motor output of 90 is no torque
#define FORWARDS 120 // motor output of 90 is no torque
#define INIT_VALUE 68// this output is the same as the receiver outputs idly
#define KP 1
#define KI 0.05f
#define SAT_ERROR 1000 // max error sum that can accumulate for integral control
#define MAX_TIME_STEP 500 // in millis
#define WHEEL_SPEED_RANGE 1000 // max error from wheel speed
#define MOTOR_RANGE 180 // max error from wheel speed
#define FULL_REVERSE 1200 // Time in microseconds of pulse width corresponding to full reverse
#define FULL_FORWARD 1660 // Time in microseconds of pulse width corresponding to full forward
//#define DEBUG

/**
 * This is a Servo object to control the steering servo. It relies on code from
 * Servo.h which is built into the Arduino IDE.
 */
static Servo motor;
static int16_t error_sum = 0;

/**
 * This is the primary function controlling the motor. It reads the
 * motor output value from the system data and controls the motor with this
 * value.
 *
 * @param motor_output the output to the motor
 */
void motor_driver_loop_fn(int16_t motor_output) {
   // if output is out of the 0-180 range, stop the motor

#ifdef DEBUG
   Serial.print("outputting to motor: ");
   Serial.println(motor_output);
#endif
   Serial.print("motor output: ");
   Serial.println(INIT_VALUE);
   if (motor_output > 180 || motor_output < 0) {
      motor.write(motor_output);
   }
   else /*if (motor_output != motor.read()) */ {
      //motor.write(motor_output);
      motor.write(motor_output);
   }
   /*
   else {
      Serial.print("what ");
   }
    */
}

void motor_driver_setup(short motor_pin) {
   motor.attach(motor_pin, FULL_REVERSE, FULL_FORWARD);
   // delay(15);
   motor.write(INIT_VALUE);
}

/**
 * Runs a control loop to stop the motor based on the reported wheel speed,
 * and returns a value to be output to the motor
 *
 * @param wheel_speed speed of the truck read by the wheel speed sensor
 * @param time_step number of millis since the last time this task ran; used
 * in integral control
 * @return the output to the motor
 */
int16_t stop_motor(int16_t wheel_speed, int16_t time_step) {
   int16_t motor_output;
   int16_t error_range = (KI * SAT_ERROR) + (KP * WHEEL_SPEED_RANGE);
   int16_t error = WHEEL_SPEED_STOP - wheel_speed;

   if (error_sum < SAT_ERROR && (time_step > 0 && time_step < MAX_TIME_STEP)) {
      error_sum += time_step * error;
      error_sum = error_sum > SAT_ERROR ? SAT_ERROR : error_sum;
   }

   motor_output = (((KP * error) + (KI * error_sum)) * (MOTOR_RANGE /
    error_range)) + MOTOR_STOP;

   //return motor_output;
   return INIT_VALUE;
}

