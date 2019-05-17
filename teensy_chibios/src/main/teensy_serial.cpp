#include <Arduino.h>
#include "include/teensy_serial.h"

#define SHORT_SIZE 2

// number of times the teensy should read form serial for every time it writes
// to it. Helps with serial buffer overflow issues
#define READ_CYCLES 2


/**
 * @brief The primary function for communicating between the Teensy and the
 * Pi over the Serial UART port.
 * @param imu_angle
 */
void teensy_serial_loop_fn(system_data_t *system_data) {

   // reading from PC console and outputting to PC console
   if (system_data->updated) {
      Serial.print("Received from imu: ");
      Serial.println(system_data->sensors.imu_angle, DEC);

      if (HWSERIAL.availableForWrite()) {
         print_sensor_msg(&system_data->sensors);
         //Serial.printf("sending bytes: %i\n", sizeof(sensor_data_t));
         HWSERIAL.write((char*)&(system_data->sensors), sizeof(sensor_data_t));
         HWSERIAL.write((char*)&(system_data->drive_mode_1), sizeof(int16_t));
         HWSERIAL.write((char*)&(system_data->drive_mode_2), sizeof(int16_t));
         system_data->updated = false;
      }
   }


   //Serial.print("HW bytes: ");
   //Serial.println(HWSERIAL.available());
   // communicate with Pi and the ROS network
   for (int i = 0; i < READ_CYCLES; i++) {
      if (HWSERIAL.available() > 0) {
         read_from_pi(&(system_data->actuators));
         Serial.println("Received from PI:");
         print_actuator_msg(&system_data->actuators);
      }
   }
}


void teensy_serial_setup(){
   Serial.begin(9600);
   HWSERIAL.begin(9600);
}


void read_from_pi(actuator_data_t *actuators_ptr) {
   byte data_buffer[SHORT_SIZE];

   HWSERIAL.readBytes(data_buffer, SHORT_SIZE);
   actuators_ptr->motor_output = *((int16_t*)data_buffer);
   HWSERIAL.readBytes(data_buffer, SHORT_SIZE);
   actuators_ptr->steer_output = *((int16_t*)data_buffer);
   HWSERIAL.readBytes(data_buffer, SHORT_SIZE);
   actuators_ptr->fifth_output = *((int16_t*)data_buffer);

   print_actuator_msg(actuators_ptr);
}


void print_sensor_msg(sensor_data_t *sensors_ptr) {
   Serial.printf("IMU angle: %i\t", sensors_ptr->imu_angle);
   Serial.printf("Wheel speed: %i\t", sensors_ptr->wheel_speed);
   Serial.printf("Right TOF: %i\t", sensors_ptr->right_TOF);
   Serial.printf("Left TOF: %i\t", sensors_ptr->left_TOF);
   Serial.printf("Rear TOF: %i\n", sensors_ptr->rear_TOF);
}


void print_actuator_msg(actuator_data_t *actuators_ptr) {
   Serial.printf("Motor output: %i\t", actuators_ptr->motor_output);
   Serial.printf("Steer output: %i\t", actuators_ptr->steer_output);
   Serial.printf("Fifth output: %i\n", actuators_ptr->fifth_output);
}
