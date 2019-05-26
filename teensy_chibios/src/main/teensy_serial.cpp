#include <Arduino.h>
#include "include/teensy_serial.h"

#define SHORT_SIZE 2

#define ACT_DATA_SIZE_W_SYNC 8
#define ACT_DATA_SIZE 6
#define READ_CYCLES 2
#define SYNC_VALUE -32000


/**
 * @brief The primary function for communicating between the Teensy and the
 * Pi over the Serial UART port.
 * @param imu_angle
 */
void teensy_serial_loop_fn(system_data_t *system_data) {
   short sync_value = SYNC_VALUE;
   short waiting_bytes;

   if (system_data->updated) {
      Serial.print("Received from imu: ");
      Serial.println(system_data->sensors.imu_angle, DEC);

      if (HWSERIAL.availableForWrite()) {
         print_sensor_msg(&system_data->sensors);
         Serial.printf("sending sync data: %i\n", sync_value);

         HWSERIAL.write((char*)&sync_value, sizeof(short));
         HWSERIAL.write((char*)&(system_data->sensors), sizeof(sensor_data_t));
         HWSERIAL.write((char*)&(system_data->drive_mode_1), sizeof(int16_t));
         system_data->updated = false;
      }
   }

   /*
   // communicate with Pi and the ROS network
   for (int i = 0; i < READ_CYCLES; i++) {
      if (HWSERIAL.available() > ACT_DATA_SIZE) {
         read_from_pi(&(system_data->actuators));
         print_actuator_msg(&system_data->actuators);
      }
   }
    */
   waiting_bytes = HWSERIAL.available();
   Serial.printf("waiting bytes: %i\n", waiting_bytes);
   // communicate with Pi and the ROS network
   for (int i = 0; i < waiting_bytes/ACT_DATA_SIZE_W_SYNC; i++) {

      teensy_sync();
      waiting_bytes = HWSERIAL.available();
      Serial.printf("waiting bytes: %i\n", waiting_bytes);
      if (waiting_bytes >= ACT_DATA_SIZE) {
         read_from_pi(&(system_data->actuators));
         print_actuator_msg(&system_data->actuators);
      }
   }
}


void teensy_serial_setup(){
   Serial.begin(9600);
   HWSERIAL.begin(9600);
}


void clear_buffer() {
   byte data_buffer[64];
   short avail = HWSERIAL.available();
   HWSERIAL.readBytes(data_buffer, avail);
}


void teensy_sync() {
   int16_t data;
   byte data_buffer[2];

   do {
      HWSERIAL.readBytes(data_buffer, 2);
      data = *((int16_t*)data_buffer);
      Serial.printf("sync data: %i\n", data);
   } while (data != SYNC_VALUE);

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
   Serial.printf("Sending to Pi:\t");
   Serial.printf("IMU angle: %i\t", sensors_ptr->imu_angle);
   Serial.printf("Wheel speed: %i\t", sensors_ptr->wheel_speed);
   Serial.printf("Right TOF: %i\t", sensors_ptr->right_TOF);
   Serial.printf("Left TOF: %i\t", sensors_ptr->left_TOF);
}


void print_actuator_msg(actuator_data_t *actuators_ptr) {
   Serial.printf("Received from Pi:\t");
   Serial.printf("Motor output: %i\t", actuators_ptr->motor_output);
   Serial.printf("Steer output: %i\t", actuators_ptr->steer_output);
   Serial.printf("Fifth output: %i\n", actuators_ptr->fifth_output);
}
