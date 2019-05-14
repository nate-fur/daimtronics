#include "include/tof_lidar.h"

/**
 * @brief A global variable that sets up the sensor to be used here
 */
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


/**
 * @brief This is the primary function controlling the ToF Lidar for reading
 * distance measurements on the sensors.
 *
 * The Adafruit_VL53L0X library does most of the work and the function here
 * calls the measure.RangeMilliMeter instruction and stores the distance here.
 * @return an integer representing distance the sensor detected in millimeters
 */
int16_t tof_loop_fn(){

    VL53L0X_RangingMeasurementData_t measure;
    int16_t dist;

    //Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        dist = measure.RangeMilliMeter;
        //Serial.print("Distance (mm): "); Serial.println(dist);
    } else {
        dist = dist;
        //Serial.println(" out of range ");
    }
    return dist;
}


/**
 * @brief Initializes the VL53L0X sensor.
 *
 */
void tof_lidar_setup() {
    Serial.println("Adafruit VL53L0X test");

    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        //while(1);
    }
}