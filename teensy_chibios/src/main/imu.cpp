#include "include/imu.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);


float imu_loop_fn() {
   /* Get a new sensor event */
   sensors_event_t event;
   bno.getEvent(&event);

   /* Display the floating point data */
   Serial.print("X: ");
   Serial.print(event.orientation.x, 4);
   Serial.print("\tY: ");
   Serial.print(event.orientation.y, 4);
   Serial.print("\tZ: ");
   Serial.print(event.orientation.z, 4);
   Serial.println("");

   return event.orientation.z;
}


void imu_setup() {
   Serial.println("Orientation Sensor Test"); Serial.println("");

   /* Initialise the sensor */
   if (!bno.begin())
   {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      //while(1);
   }

   delay(1000);

   bno.setExtCrystalUse(true);
}
