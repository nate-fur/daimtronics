/* CHIBIOS */
#include <ChRt.h>

// BNO055
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int led = LED_BUILTIN; // 13
SEMAPHORE_DECL(sem, 0);
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//------------------------------------------------------------------------------
// Thread 1: read data from the IMU
static THD_WORKING_AREA(waThread1, 64);

static THD_FUNCTION(Thread1, arg) {
  sensors_event_t event; 
  bno.getEvent(&event);
  
  // Print the data from the IMU out
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  chThdSleepMilliseconds(200);
}

//------------------------------------------------------------------------------
// Thread 2: print data out to the serial port
static THD_WORKING_AREA(waThread2, 64);

static THD_FUNCTION(Thread2, arg) {

}

//------------------------------------------------------------------------------
// continue setup() after chBegin().
void chSetup() {
  // Start threads.
  chThdCreateStatic(waThread1, sizeof(waThread1),
    NORMALPRIO + 2, Thread1, NULL);

  chThdCreateStatic(waThread2, sizeof(waThread2),
    NORMALPRIO + 1, Thread2, NULL);
}



//------------------------------------------------------------------------------
void setup() {
  // Initialize OS and then call chSetup.
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
  chBegin(chSetup);
  // chBegin() resets stacks and should never return.
  while (true) {}
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
  */
}
