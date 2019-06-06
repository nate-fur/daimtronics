#include "include/hall_sensor.h"
#include <Arduino.h>
#include <ChRt.h>


int16_t ticks = 0;
/**
 * @brief This is the primary function controlling the wheel speed sensor. It
 * reads a Hall sensor that is built into the Tekin R8 Motor. This is a three
 * phased motor (PhaseA, PhaseB, PhaseC). Each phase is offset 120 degrees
 * from each other. Ticks will be incremented if the motor rotates forwards
 * one full revolution, and decremented if it rotates backwards one
 * revolution.
*
 * @return the current number of ticks that the sensor reads.
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


/**
 * @brief Set up the wheel speed task to read high or low values from the three
 * pins attached to the Hall sensor (one for each phase).
 *
 * @param PhaseA_pin An interrupt is triggered every time the Teensy reads a
 * leading edge for this phase.
 * @param PhaseB_pin If this pin is high when the interrupt is triggered, the
 * truck is going in reverse.
 * @param PhaseC_pin If this pin is high when the interrupt is triggered, the
 * truck is going forwards.
 */
void hall_sensor_setup(short PhaseA_pin, short PhaseB_pin, short PhaseC_pin) {
    pinMode(PhaseA_pin, INPUT);
    pinMode(PhaseB_pin, INPUT);
    pinMode(PhaseC_pin, INPUT);
    delay(10);
}