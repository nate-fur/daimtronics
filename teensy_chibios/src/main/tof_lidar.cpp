#include "include/tof_lidar.h"
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void tof_lidar_setup() {
    Serial.println("Adafruit VL53L0X test");

    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        //while(1);
    }
}