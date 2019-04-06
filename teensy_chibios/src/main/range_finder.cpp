//
// Created by nate on 4/2/19.
//
#include "include/range_finder.h"
#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

#define URF1_TRIG 4      // Sensor 1 trigger pin
#define URF1_ECHO 5      // Sensor 1 echo pin
#define URF2_TRIG 6      // Sensor 2 trigger pin
#define URF2_ECHO 7      // Sensor 2 echo pin
#define URF3_TRIG 8      // Sensor 3 trigger pin
#define URF3_ECHO 9      // Sensor 3 echo pin

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
        NewPing(URF1_TRIG, URF1_ECHO, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
        NewPing(URF2_TRIG, URF2_ECHO, MAX_DISTANCE),
        NewPing(URF3_TRIG, URF3_ECHO, MAX_DISTANCE)
};
/**
 * This is the primary function controlling the URFs. It reads a value
 * representing a distance to an object from the URF sensor and returns that
 * value
 *
 * @return distance to object
 */
short range_finder_loop_fn(short *URF_array) {

    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
        Serial.print(i);
        Serial.print("=");
        URF_array[i] = (short) sonar[i].ping_cm();
        Serial.print(URF_array[i]);
        Serial.print("cm ");
    }
    Serial.println();

    return 0; // placeholder for compilation
}


void range_finder_setup() {


}
