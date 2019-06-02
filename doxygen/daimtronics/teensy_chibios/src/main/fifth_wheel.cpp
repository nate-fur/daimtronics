#include "include/fifth_wheel.h"
#include "include/system_data.h"
#include <Arduino.h>
#include <Servo.h>

#define LOCKED_ANGLE 0
#define UNLOCKED_ANGLE 180

/**
 * @brief This is a Servo object to control the fifth wheel. It relies on code
 * from Servo.h which is a prebuilt library for the Arduino IDE.
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
void fifth_wheel_loop_fn(int16_t fifth_output) {

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

/**
 * @brief Set up the fifth wheel task to write to the pin attached to
 * the fifth wheel, and to be in the locked position.
 *
 * @param fifth_wheel_pin The pin that signals a PWM to the fifth wheel servo.
 */
void fifth_wheel_setup(short fifth_wheel_pin) {
   fifth_wheel_servo.attach(fifth_wheel_pin);
   //delay(15);
   fifth_wheel_servo.write(LOCKED_ANGLE);
}

