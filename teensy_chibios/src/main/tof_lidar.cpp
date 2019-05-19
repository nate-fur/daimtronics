#include "include/tof_lidar.h"

#define TCAADDR 0x70

static bool initialized = false;

/**
 * @brief A global variable that sets up the sensors to be used here
 */

Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();

void tcaselect(uint8_t i) {
    if (i > 7) return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

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
    sensor1.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        dist = measure.RangeMilliMeter;
        //Serial.print("Distance (mm): "); Serial.println(dist);
    } else {
        dist = dist;
        //Serial.println(" out of range ");
    }

    if (initialized) {
        //Serial.print("Reading a measurement... ");
        sensor1.rangingTest(&measure,
                        false); // pass in 'true' to get debug data printout!
        if (measure.RangeStatus != 4) {  // phase failures have incorrect data
            dist = measure.RangeMilliMeter;
            //Serial.print("Distance (mm): "); Serial.println(dist);
        } else {
            dist = dist;
            //Serial.println(" out of range ");
        }
        return dist;
    }
    else {
        return 0;

    }
}

/**
 * @brief Initializes the VL53L0X sensor.
 *
 */
void tof_lidar_setup() {
    Serial.println("Adafruit VL53L0X 1 test");
    Wire.begin();
    tcaselect(0);

    if (!sensor1.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        //while(1);
    }
    tcaselect(1);
    if (!sensor2.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        //while(1);
    }
    tcaselect(2);
    if (!sensor3.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        //while(1);
    }
    else {
        initialized = true;
    }
}