#include "include/teensy_serial.h"
#include "include/imu.h"

#include <ChRt.h>

#define HWSERIAL Serial1
#define NUM_BUFFERS 1
#define BUFFERS_SIZE 256

// Declare a semaphore with an inital counter value of zero.
SEMAPHORE_DECL(sem, 0);


//------------------------------------------------------------------------------
// Thread 1, turn the LED off when signalled by thread 2.
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread1, 64);

static THD_FUNCTION(Thread1, arg) {
   (void)arg;
   while (!chThdShouldTerminateX()) {
      // Wait for signal from thread 2.
      chSemWait(&sem);

      // Turn LED off.
      digitalWrite(LED_BUILTIN, LOW);
   }
}



//------------------------------------------------------------------------------
// Thread 2, turn the LED on and signal thread 1 to turn the LED off.
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread2, 64);

static THD_FUNCTION(Thread2, arg) {
   (void)arg;
   pinMode(LED_BUILTIN, OUTPUT);
   while (true) {
      digitalWrite(LED_BUILTIN, HIGH);

      // Sleep for 200 milliseconds.
      chThdSleepMilliseconds(200);

      // Signal thread 1 to turn LED off.
      chSemSignal(&sem);

      // Sleep for 200 milliseconds.
      chThdSleepMilliseconds(200);
   }
}



//------------------------------------------------------------------------------
// Thread 3, IMU
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread3, 2048);

static THD_FUNCTION(Thread3, arg) {
   mailbox_t *imu_mbox = (mailbox_t*)arg;
   short imu_angle = 0;

   while (true) {
      imu_angle = imu_loop_fn();
      chMBPost(imu_mbox, imu_angle, TIME_INFINITE);
      chThdSleepMilliseconds(500);
   }
}



//------------------------------------------------------------------------------
// Thread 4, Communicate via serial (UART)
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread4, 2048);

static THD_FUNCTION(Thread4, arg) {
   mailbox_t *imu_mbox = (mailbox_t*)arg;
   short imu_angle = 0;

   while (true) {
      chMBFetch(imu_mbox, (msg_t*)&imu_angle, TIME_INFINITE);
      serial_loop_fn(imu_angle);
      chThdSleepMilliseconds(100);
   }
}



//------------------------------------------------------------------------------
// Thread 5, Read from URF
//
/* 64 byte stack beyond task switch and interrupt needs.*/
static THD_WORKING_AREA(waThread5, 64);

static THD_FUNCTION(Thread5, arg) {
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
    mailbox_t imu_mbox;
    msg_t mbox_buffer[BUFFERS_SIZE];

    chMBObjectInit(&imu_mbox, mbox_buffer, BUFFERS_SIZE);

   // Start threads.
   chThdCreateStatic(waThread1, sizeof(waThread1),
   NORMALPRIO + 2, Thread1, NULL);

   chThdCreateStatic(waThread2, sizeof(waThread2),
   NORMALPRIO + 1, Thread2, NULL);

   chThdCreateStatic(waThread3, sizeof(waThread3),
   NORMALPRIO + 1, Thread3, (void*)&imu_mbox);

   chThdCreateStatic(waThread4, sizeof(waThread4),
   NORMALPRIO + 1, Thread4, (void*)&imu_mbox);
}



//------------------------------------------------------------------------------
void setup() {
   // Initialize OS and then call chSetup.
   teensy_serial_setup();
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
