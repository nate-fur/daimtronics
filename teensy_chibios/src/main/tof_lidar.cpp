#include "include/tof_lidar.h"

/**
 * @brief A global variable that sets up the Sensor 1 to be used here
 */
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
/**
 * @brief A global variable that sets up the Sensor 2 to be used here
 */
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();



/**
 * @brief A variable that is used to determine if Sensor 1 is initialized
 */
static bool sens1_initialized = false;
/**
 * @brief A variable that is used to determine if Sensor 2 is initialized
 */
static bool sens2_initialized = false;



/**
 * @brief This is the primary function controlling the left ToF Lidar for reading
 * distance measurements on the sensors.
 *
 * The Adafruit_VL53L0X library does most of the work and the function here
 * calls the measure.RangeMilliMeter instruction and stores the distance here.
 * @return an integer representing distance the sensor detected in millimeters
 */
int16_t tof_left_loop_fn(){
    VL53L0X_RangingMeasurementData_t measure_left;
    int16_t dist;

    if (sens1_initialized) {
        //Serial.print("Reading a measurement... ");
        sensor1.rangingTest(&measure_left, false); // pass in 'true' to get debug data
        if (measure_left.RangeStatus != 4) {  // phase failures have incorrect data
            dist = measure_left.RangeMilliMeter;
            //Serial.print("Distance (mm): "); Serial.println(dist);
        } else {
            //Serial.println(" out of range ");
        }
        return dist;
    }
    else {
        return 0;
    }
}

/**
 * @brief This is the primary function controlling the right ToF Lidar for reading
 * distance measurements on the sensors.
 *
 * The Adafruit_VL53L0X library does most of the work and the function here
 * calls the measure.RangeMilliMeter instruction and stores the distance here.
 * @return an integer representing distance the sensor detected in millimeters
 */
int16_t tof_right_loop_fn(){
    VL53L0X_RangingMeasurementData_t measure_right;
    int16_t dist;

    if (sens2_initialized) {
        //Serial.print("Reading a measurement... ");
        sensor1.rangingTest(&measure_right, false); // pass in 'true' to get debug data
        if (measure_right.RangeStatus != 4) {  // phase failures have incorrect data
            dist = measure_right.RangeMilliMeter;
            //Serial.print("Distance (mm): "); Serial.println(dist);
        } else {
            //Serial.println(" out of range ");
        }
        return dist;
    }
    else {
        return 0;
    }
}

/**
 * @brief Initializes the VL53L0X sensors.
 *
 */
void tof_lidar_setup() {
    Serial.println("Adafruit VL53L0X 1 test");
    Wire.begin();

    if (!sensor1.begin(0x29, false, &Wire1)) {
        Serial.println(F("Failed to boot left VL53L0X"));
        //while(1);
    }
    else {
        sens1_initialized = true;
    }
    if (!sensor2.begin(0x29, false, &Wire2)) {
        Serial.println(F("Failed to boot right VL53L0X"));
        //while(1);
    }
    else {
        sens2_initialized = true;
    }
}