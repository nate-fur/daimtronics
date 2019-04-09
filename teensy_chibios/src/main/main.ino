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
#include "include/fifth_wheel.h"
#include "include/imu.h"
#include "include/motor_driver.h"
#include "include/range_finder.h"
#include "include/RC_receiver.h"
#include "include/steer_servo.h"
#include "include/teensy_serial.h"
#include "include/wheel_speed.h"

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
 * @brief Fifth Wheel Thread: Reads desired state of the fifth wheel from the
 * system_data and outputs a servo angle corresponding to locked or unlocked.
 *
 * This thread calls fifth_wheel_loop_fn() which is the primary function for the
 * fifth wheel and whose implementation is found in fifth_wheel.cpp.
 */
static THD_WORKING_AREA(fifth_wheel_wa, 64);

static THD_FUNCTION(fifth_wheel_thread, arg) {
   short fifth_output;

   while (true) {
      Serial.println("fifth wheel");
      fifth_output = system_data.actuators.fifth_output;

      fifth_wheel_loop_fn(fifth_output);

      chThdSleepMilliseconds(100);
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
      Serial.println("imu");
      imu_angle = imu_loop_fn() * 1000;


      chMtxLock(&sysMtx);
      system_data.sensors.imu_angle = imu_angle;
      system_data.updated = true;
      chMtxUnlock(&sysMtx);

      chThdSleepMilliseconds(100);
   }
}



/**
 * @brief Motor Driver Thread: Reads motor output levels from the system data
 * and controls the motor based on this value.
 *
 * This thread calls motor_driver_loop_fn which is the primary function for
 * the motor and whose implementation is found in motor_driver.cpp.
 */
static THD_WORKING_AREA(motor_driver_wa, 64);

static THD_FUNCTION(motor_driver_thread, arg) {
   short motor_output;

   while (true) {
      Serial.println("motor");
      motor_output = system_data.actuators.motor_output;

      motor_driver_loop_fn(motor_output);

      chThdSleepMilliseconds(100);
   }
}


/**
 * @brief Range Finder Thread: Controls the HC-SR04 URF where nearby object
 * distance is calculated and written to the system data.
 *
 * @todo still need to implement this task (likely going to use interrupts)
 * and put the code in a URF.cpp file.
 */
static THD_WORKING_AREA(range_finder_wa, 64);

static THD_FUNCTION(range_finder_thread, arg) {
   (void)arg;
   Serial.println("starting up URF driver");

   while (true) {

      chSysLock();

      range_finder_ping();

      chSysUnlock();

      chThdSleepMilliseconds(100);
   }
}

BSEMAPHORE_DECL(urf_isrSem, true);

void urf_ISR_Fcn() {
    CH_IRQ_PROLOGUE();
    // IRQ handling code, preemptable if the architecture supports it.

    chSysLockFromISR();
    // Invocation of some I-Class system APIs, never preemptable.

    // Signal handler task.
    chBSemSignalI(&urf_isrSem);
    chSysUnlockFromISR();

    // More IRQ handling code, again preemptable.

    // Perform rescheduling if required.
    CH_IRQ_EPILOGUE();
}

// Handler task for interrupt.
static THD_WORKING_AREA(isr_wa_thd, 128);

static THD_FUNCTION(handler, arg) {
    (void)arg;
    long urf_dist;
    while (true) {
        // wait for event
        chBSemWait(&urf_isrSem);

        urf_dist = range_finder_loop_fn();

    }
}

/**
 * @brief RC Receiver Thread: Reads auxiliary signals from the RC receiver
 * for determining what drive mode the semi-truck is in.
 *
 * This thread calls RC_receiver_loop_fn which is the primary function for
 * the RC receiver and whose implementation is found in RC_receiver.cpp
 */
static THD_WORKING_AREA(RC_receiver_wa, 64);

static THD_FUNCTION(RC_receiver_thread, arg) {
   short drive_mode;

   while (true) {
      Serial.println("rc");
      drive_mode = RC_receiver_loop_fn();

      chMtxLock(&sysMtx);
      system_data.drive_mode = drive_mode;
      chMtxUnlock(&sysMtx);

      chThdSleepMilliseconds(100);
   }
}



/**
 * @brief Steer Servo Thread: Controls the servo that dictates the driving
 * angle of the semi truck
 *
 * This thread calls steer_servo_loop_fn which is the primary function for
 * the steering servo and whose implementation is found in steer_servo.cpp
 */
static THD_WORKING_AREA(steer_servo_wa, 64);

static THD_FUNCTION(steer_servo_thread, arg) {
   short steer_output;

   while (true) {

      Serial.println("steer");
      steer_output = system_data.actuators.steer_output;

      steer_servo_loop_fn(steer_output);

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

      Serial.println("serial");

      chMtxLock(&sysMtx);
      teensy_serial_loop_fn(&system_data);
      chMtxUnlock(&sysMtx);

      chThdSleepMilliseconds(100);
   }
}



/**
 * @brief Wheel Speed Thread: Controls the digital IR sensor that is mounted on
 * the front axis of the vehicle for determining wheel speed.
 *
 * This thread calls wheel_speed_loop_fn() which is the primary function for the
 * wheel speed sensor and whose implementation is found in wheel_speed.cpp.
 */
static THD_WORKING_AREA(wheel_speed_wa, 64);

static THD_FUNCTION(wheel_speed_thread, arg) {
   short wheel_speed;

   while (true) {

      Serial.println("wheel");
      wheel_speed = wheel_speed_loop_fn();

      chMtxLock(&sysMtx);
      system_data.sensors.wheel_speed = wheel_speed;
      chMtxUnlock(&sysMtx);

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

   chThdCreateStatic(fifth_wheel_wa, sizeof(fifth_wheel_wa),
   NORMALPRIO, fifth_wheel_thread, NULL);

   chThdCreateStatic(imu_wa, sizeof(imu_wa),
   NORMALPRIO, imu_thread, NULL);

   chThdCreateStatic(motor_driver_wa, sizeof(motor_driver_wa),
   NORMALPRIO, motor_driver_thread, NULL);

   chThdCreateStatic(range_finder_wa, sizeof(range_finder_wa),
   NORMALPRIO, range_finder_thread, NULL);

   chThdCreateStatic(RC_receiver_wa, sizeof(RC_receiver_wa),
   NORMALPRIO, RC_receiver_thread, NULL);

   chThdCreateStatic(steer_servo_wa, sizeof(steer_servo_wa),
   NORMALPRIO, steer_servo_thread, NULL);

   chThdCreateStatic(teensy_serial_wa, sizeof(teensy_serial_wa),
   NORMALPRIO, teensy_serial_thread, NULL);

   chThdCreateStatic(wheel_speed_wa, sizeof(wheel_speed_wa),
   NORMALPRIO, wheel_speed_thread, NULL);

   chThdCreateStatic(isr_wa_thd, sizeof(isr_wa_thd), NORMALPRIO + 1, handler, NULL);
   attachInterrupt(digitalPinToInterrupt(24), urf_ISR_Fcn, CHANGE);
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
   // Setup the fifth wheel
   fifth_wheel_setup();
   // Setup the motor driver
   motor_driver_setup();
   // Setup the URFs
   range_finder_setup();
   // Setup the RC receiver
   RC_receiver_setup();
   // Setup the steering servo
   steer_servo_setup();
   // Setup the wheel speed sensors
   wheel_speed_setup();
   // chBegin() resets stacks and should never return.
   chBegin(chSetup);

   while (true) {}
}



// loop() is the main thread.  Not used in this example.
void loop() {
}



//ISR FUNCTION CODE
