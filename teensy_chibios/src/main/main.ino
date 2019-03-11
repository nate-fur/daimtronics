// Example to demonstrate thread definition, semaphores, and thread sleep.
#include <ChRt.h>

// Adafruit IMU Sensor Includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define HWSERIAL Serial1

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// LED_BUILTIN pin on Arduino is usually pin 13.

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
static THD_WORKING_AREA(waThread3, 256);

static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  while (true) {
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

    chThdSleepMilliseconds(100);
  }
}



//------------------------------------------------------------------------------
// Thread 4, Read from UART
//
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waThread4, 64);

static THD_FUNCTION(Thread4, arg) {
  (void)arg;
  Serial.print("starting up serial reader");

  int incomingByte;
  while (true) {
    Serial.print("in serial reader");
    if (HWSERIAL.available() > 0) {
      incomingByte = HWSERIAL.read();
      Serial.print("UART received: ");
      Serial.println(incomingByte, DEC);
      HWSERIAL.print("UART received:");
      HWSERIAL.println(incomingByte, DEC);
    }

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
  // Start threads.
  chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO + 2, Thread1, NULL);

  chThdCreateStatic(waThread2, sizeof(waThread2),
                    NORMALPRIO + 1, Thread2, NULL);

  chThdCreateStatic(waThread3, sizeof(waThread3),
                    NORMALPRIO + 1, Thread3, NULL);

  /*chThdCreateStatic(waThread4, sizeof(waThread4),
                    NORMALPRIO + 1, Thread4, NULL);*/
}



//------------------------------------------------------------------------------
void setup() {
  // Initialize OS and then call chSetup.
  Serial.begin(9600);
  HWSERIAL.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  chBegin(chSetup);
  // chBegin() resets stacks and should never return.
  while (true) {}
}



//------------------------------------------------------------------------------
// loop() is the main thread.  Not used in this example.
void loop() {
}




//ISR FUNCTION CODE
