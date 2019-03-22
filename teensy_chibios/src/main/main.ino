/**
 * @file This file holds the main loop that that runs on the Teensy. It
 * creates a ChibiOS thread for each task in the system and assigns
 * priorities and stack sizes. Within the while loop for each thread, the
 * "primary" function for a task is called. The project is organized so that
 * the implementation of primary a function for a task is held in the task's
 * respective .cpp file (which is also within the same "main" folder that
 * main.ino is in). Header files for each cpp file are found within the
 * "include" folder that is in the "main" folder, and these header files must
 * be included here.
 *
 * @author Daimtronics
 */

#include "include/system_data.h"
#include "include/teensy_serial.h"
#include "include/imu.h"

#include <ChRt.h>

#define HWSERIAL Serial1

/**
 * @brief The data for the entire system. Synchronization will be achieved
 * through the use of ChibiOS's mutex library.
 *
 * Tasks that control a sensor will call the primary function to read that
 * sensor, obtain the mutex to for the system data, write the sensor data to
 * the system_data, and then release the mutex on the system_data.
 *
 * Tasks that control actuators will read from the system_data (no mutex
 * needed as the system_data is read-only for actuator tasks) to receive the
 * specific data that corresponds to their task and then run their primary
 * functions to control the actuators.
 */
static system_data_t system_data = {0};
MUTEX_DECL(sysMtx);

/**
 * @brief Heartbeat Thread: blinks the LED periodically so that the user is
 * sure that the Teensy program is still running and has not crashed.
 */
static THD_WORKING_AREA(heartbeat_wa, 64);

static THD_FUNCTION(heartbeat_thread, arg) {
   (void)arg;
   pinMode(LED_BUILTIN, OUTPUT);
   while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      chThdSleepMilliseconds(100);
      digitalWrite(LED_BUILTIN, LOW);
      chThdSleepMilliseconds(1500);
   }
}


/**
 * @brief IMU Thread: Reads euler angles from the BNO055 IMU and writes the
 * data to the system_data after obtaining the system_data mutex.
 *
 * This thread calls imu_loop_fn() which is the primary function for the IMU
 * and whose implementation is found in imu.cpp.
 */
static THD_WORKING_AREA(imu_wa, 2048);

static THD_FUNCTION(imu_thread, arg) {
   float imu_angle;

   while (true) {
      Serial.println("*****************************************************");
      imu_angle = imu_loop_fn();


      chMtxLock(&sysMtx);
      system_data.sensors.imu_angle = imu_angle;
      chMtxUnlock(&sysMtx);

      chThdSleepMilliseconds(100);
   }
}



/**
 * @brief Teensy Serial Thread: Communicates over the serial (UART) port to
 * relay system data between the Teensy and the Raspberry Pi.
 *
 * This thread calls serial_loop_fn() which is the primary function for the
 * serial communication and whose implementation is found in teensy_serial.cpp.
 */
static THD_WORKING_AREA(teensy_serial_wa, 2048);

static THD_FUNCTION(teensy_serial_thread, arg) {

   while (true) {

      chMtxLock(&sysMtx);
      serial_loop_fn(system_data.sensors.imu_angle);
      chMtxUnlock(&sysMtx);

      chThdSleepMilliseconds(100);
   }
}



/**
 * @brief URF Thread: Controls the HC-SR04 URF where nearby object distance
 * is calculated and written to the system data.
 *
 * @todo still need to implement this task (likely going to use interrupts)
 * and put the code in a URF.cpp file.
 */
static THD_WORKING_AREA(urf_wa, 64);

static THD_FUNCTION(urf_thread, arg) {
   (void)arg;
   Serial.print("starting up URF driver");

   int URF_reading;
   while (true) {
      chSysLock();
      digitalWrite(27, HIGH);
      delayMicroseconds(10);
      digitalWrite(27, LOW);

      Serial.print("in serial reader");

      HWSERIAL.print("URF received:");


      chThdSleepMilliseconds(100);
   }
}



/**
 * @brief Creates the threads to be run by assigning the thread function,
 * working space, priority and any parameters that the thread needs.
 *
 * While the static thread definitions are written before this function, none
 * of them are used until chThdCreateStatic(...) is called.
 */
void chSetup() {

   chThdCreateStatic(heartbeat_wa, sizeof(heartbeat_wa),
   NORMALPRIO, heartbeat_thread, NULL);

   chThdCreateStatic(imu_wa, sizeof(imu_wa),
   NORMALPRIO, imu_thread, NULL);

   chThdCreateStatic(teensy_serial_wa, sizeof(teensy_serial_wa),
   NORMALPRIO, teensy_serial_thread, NULL);

   /*
   chThdCreateStatic(urf_wa, sizeof(urf_wa),
   NORMALPRIO, urf_thread, NULL);
   */
}


/**
 * @brief Initializes the semi-truck system so that it is ready to run in an
 * RTOS environment.
 *
 * The setup function is default to Arduino sketches, and holds all of the
 * code that must be run before the main loop can be run. In this project,
 * each task will have a setup function in it's .cpp file that is called here.
 * Finally, ChibiOS setup is called to start initialize threads and start the
 * thread scheduling that is built in to ChibiOS.
 */
void setup() {
   // Setup the serial ports -- both the hardware (UART) and console (USB)
   teensy_serial_setup();
   // Setup the IMU to make sure it is connected and reading
   imu_setup();

   chBegin(chSetup);
   // chBegin() resets stacks and should never return.
   while (true) {}
}



//------------------------------------------------------------------------------
// loop() is the main thread.  Not used in this example.
void loop() {
}



//ISR FUNCTION CODE
