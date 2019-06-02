#include "include/range_finder.h"
#include "Arduino.h"

int val = 0;
unsigned long high_time = 0;
long distance = 0;
unsigned long time = 0;

/**
 * This is the primary function controlling the URFs. It reads a value
 * representing a distance to an object from the URF sensor and returns that
 * value
 *
 * @return distance to object
 */
long range_finder_loop_fn(short urf_echo_pin) {

    val = digitalRead(urf_echo_pin);

    if(val==HIGH){
        time = micros();

    } else if(val==LOW){
        long f_time = micros();
        high_time = f_time - time;
        distance = high_time/58;
#ifdef DEBUG
        //Serial.print("Distance: ");
        //Serial.print(distance);
#endif
    }
    return 0; // placeholder for compilation
}

void range_finder_ping(short urf_trig_pin){

    digitalWrite(urf_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(urf_trig_pin, LOW);

}

void range_finder_setup(short urf_trig_pin) {
    pinMode(urf_trig_pin, OUTPUT);
    digitalWrite(urf_trig_pin,LOW);

}
