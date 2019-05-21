#include "include/imu.h"

/**
 * @brief A global variable, for the only bno object in the system
 */
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**
 * @brief The primary function for the IMU that reads and returns heading
 * data of the BNO055.
 *
 * The Adafruit_BNO055 library does most of the work here. This function
 * simply sets up a sets up an "event" variable to hold the IMU data, calls
 * @return an integer representing the heading angle in degrees
 */
int16_t imu_loop_fn() {
   /* Get a new sensor event */
   sensors_event_t event;
   bno.getEvent(&event);

   /* Display the floating point data */
   //print_imu_data(&event);

   return (int16_t) (event.orientation.x + 0.5); // rounds to nearest integer
}


/**
 * @brief Initializes the BNO055 sensor.
 *
 */
void imu_setup() {
   Serial.println("Orientation Sensor Test"); Serial.println("");

   /* Initialise the sensor */
   if (!bno.begin())
   {
      Serial.println("Ooops, no BNO055 detected ... Check your wiring or "
                     "I2C ADDR!");
      //while(1);
   }

   delay(1000);

   bno.setExtCrystalUse(true);
}

/**
 * @brief A debugging function used to print out IMU orientation to the USB
 * serial device (usually the Arduino Serial Monitor).
 */
void print_imu_data(sensors_event_t *event) {
   Serial.print("X: ");
   Serial.print(event->orientation.x, 4);
   Serial.print("\tY: ");
   Serial.print(event->orientation.y, 4);
   Serial.print("\tZ: ");
   Serial.print(event->orientation.z, 4);
   Serial.println("");
}
