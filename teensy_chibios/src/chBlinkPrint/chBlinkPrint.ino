#include "ChRt.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55);

// data structures and stack for our threads, along with their pointers
static THD_WORKING_AREA(waThread1, 64);
thread_t *heartbeat_thd_ptr;

static THD_WORKING_AREA(waThread2, 64);
thread_t *imu_thd_ptr;

static THD_WORKING_AREA(waThread3, 64);
thread_t *intercomm_thd_ptr;



//------------------------------------------------------------------------------
// thread 1 - low priority heartbeat LED thread.
// 64 byte stack beyond task switch and interrupt needs.
static THD_FUNCTION(heartbeat_fn , arg) {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("starting heartbeat fn");

  // Flash led every 200 ms.
  while (true) {
    // Turn LED on.
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("in heartbeat fn");

    // Sleep for 50 milliseconds.
    chThdSleepMilliseconds(50);

    // Turn LED off.
    digitalWrite(LED_BUILTIN, LOW);

    // Sleep for 150 milliseconds.
    chThdSleepMilliseconds(150);
  }
}



//------------------------------------------------------------------------------
// thread 2 - read from IMU and send message to serial thread
// 64 byte stack beyond task switch and interrupt needs.
static THD_FUNCTION(imu_fn, arg) {
  sensors_event_t imu_data; 
  Serial.print("starting IMU fn");
  while (true) {
    
    bno.getEvent(&imu_data);

    chMsgSend(intercomm_thd_ptr, (msg_t)&imu_data);
    
    Serial.print("X: ");
    Serial.print(imu_data.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(imu_data.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(imu_data.orientation.z, 4);
    Serial.println("");
    
    chThdSleepMilliseconds(100);
  }
}



//------------------------------------------------------------------------------
// thread 3 - receieve message from IMU message and output to serial
// 64 byte stack beyond task switch and interrupt needs.
static THD_FUNCTION(intercomm_fn, arg) {
  sensors_event_t *imu_data;
  Serial.print("starting serial fn");

  while (true) {
    chMsgWait();
    
    imu_data = (sensors_event_t*)chMsgGet(imu_thd_ptr);
    chMsgGet(imu_thd_ptr);

    Serial.print("X: ");
    Serial.print(imu_data->orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(imu_data->orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(imu_data->orientation.z, 4);
    Serial.println("");
    
    chMsgRelease(imu_thd_ptr, (msg_t)&imu_data);
    chThdSleepMilliseconds(100);
  }
}



//------------------------------------------------------------------------------
// Name chSetup is not special - must be same as used in chBegin() call.
void chSetup() {
  Serial.print("in chsetup");
  // Start heartbeat thread that verifies that the system is running
  heartbeat_thd_ptr = chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO - 1, heartbeat_fn, NULL);
                    
  // Start IMU thread to read data from the BNO055 and send msg to intercomm
  imu_thd_ptr = chThdCreateStatic(waThread2, sizeof(waThread2),
                    NORMALPRIO + 1, imu_fn, NULL);
                    
  // Start Serial thread to receive IMU msgs and send to Pi                  
  intercomm_thd_ptr = chThdCreateStatic(waThread3, sizeof(waThread3),
                    NORMALPRIO + 1, intercomm_fn, NULL);
  
  Serial.print("in chsetup");

}



//------------------------------------------------------------------------------
void setup() {
  // Serial Setup
  Serial.begin(9600);
  while (!Serial) {}
  
  Serial.print("X: ");
  Serial.print("\tY: ");
  Serial.print("\tZ: ");
  Serial.println("");
  Serial.print("what the fuck is going on");

  /* IMU Setup
  if(!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  */
  
  Serial.print("after sleep");

  chThdSleepMilliseconds(1000);;  
  bno.setExtCrystalUse(true);
  Serial.print("after sleep");
  chBegin(chSetup);
  // chBegin() resets stacks and should never return.
  while (true) {}
}



//------------------------------------------------------------------------------
// Runs at NORMALPRIO.
void loop() {
  // Sleep for one second.
  //chThdSleepMilliseconds(1000);
  Serial.print("in main loop\n");
  // Print count for previous second.
  /*
  Serial.print(F("Count: "));
  Serial.print(count);
  
  // Zero count.
  count = 0;

  // Print unused stack space in bytes.
  Serial.print(F(", Unused Stack: "));
  Serial.print(chUnusedThreadStack(waThread1, sizeof(waThread1)));
  Serial.print(' ');
  Serial.print(chUnusedThreadStack(waThread2, sizeof(waThread2)));
  Serial.print(' ');
  Serial.print(chUnusedMainStack());
#ifdef __arm__
  // ARM has separate stack for ISR interrupts. 
  Serial.print(' ');
  Serial.print(chUnusedHandlerStack());
#endif  // __arm__
  Serial.println();
  */
}
