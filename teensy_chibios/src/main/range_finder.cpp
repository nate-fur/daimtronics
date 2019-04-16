//
// Created by nate on 4/2/19.
//
#include "include/range_finder.h"
#include "Arduino.h"

int val = 0;
long high_time = 0;
long distance = 0;
long time = 0;

/**
 * This is the primary function controlling the URFs. It reads a value
 * representing a distance to an object from the URF sensor and returns that
 * value
 *
 * @return distance to object
 */
long range_finder_loop_fn() {

    val = digitalRead(24);

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

void range_finder_ping(){

    digitalWrite(27, HIGH);
    delayMicroseconds(10);
    digitalWrite(27, LOW);

}

void range_finder_setup() {
    pinMode(27, OUTPUT);
    digitalWrite(27,LOW);

}
