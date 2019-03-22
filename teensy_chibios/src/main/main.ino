#include "include/system_data.h"
#include "include/teensy_serial.h"
#include "include/imu.h"

#include <ChRt.h>

#define HWSERIAL Serial1
#define NUM_BUFFERS 1
#define BUFFERS_SIZE sizeof(system_data_t)

// This is the data for the entire system. Synchronization will be achieved
// through the use of mutexes
static system_data_t system_data = {0};

MUTEX_DECL(sysMtx);

//------------------------------------------------------------------------------
// Heartbeat Thread
//
// Blinks the LED to verify that teensy is still running
//
// 64 byte stack beyond task switch and interrupt needs.
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



//------------------------------------------------------------------------------
// IMU Thread
//
// Reads euler angle from the BNO055 IMU and relays the data to the serial
// thread for communication with the Pi
//
// 64 byte stack beyond task switch and interrupt needs.
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



//------------------------------------------------------------------------------
// Teensy Serial Thread
//
// Communicates over the serial (UART) port on the teensy to the Raspberry Pi
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(teensy_serial_wa, 2048);

static THD_FUNCTION(teensy_serial_thread, arg) {

   while (true) {

      chMtxLock(&sysMtx);
      serial_loop_fn(system_data.sensors.imu_angle);
      chMtxUnlock(&sysMtx);

      chThdSleepMilliseconds(100);
   }
}



//------------------------------------------------------------------------------
// Thread 5, Read from URF
//
/* 64 byte stack beyond task switch and interrupt needs.*/
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



//------------------------------------------------------------------------------
// continue setup() after chBegin().
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



//------------------------------------------------------------------------------
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
