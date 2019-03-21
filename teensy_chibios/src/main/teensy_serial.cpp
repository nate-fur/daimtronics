#include <Arduino.h>
#include "include/teensy_serial.h"

#define NUM_MSGS 4
#define SHORT 2
#define FLOAT 2


void serial_loop_fn(short imu_angle) {

   byte user_input;
   sensor_data_t sensor_data_out;
   sensor_data_t sensor_data_in;
   short *short_ptr;

   // reading from PC console and outputting to PC console
   if (Serial.available() > 0) {
      user_input = Serial.read() - OFFSET;
      Serial.read(); // clear the newline
      Serial.print("Received from terminal: ");
      Serial.println(user_input, DEC);
      Serial.print("Received from imu: ");
      Serial.println(imu_angle);
      set_sensor_msg(user_input, &sensor_data_out);

      if (HWSERIAL.availableForWrite()) {
         Serial.println("Sending to PI:");
         print_sensor_msg(&sensor_data_out);
         HWSERIAL.write((char*)&sensor_data_out, sizeof(sensor_data_t));
      }
   }

   // communicate with Pi and the ROS network
   if (HWSERIAL.available() > 0) {
      read_from_pi(&sensor_data_in);
      Serial.println("Received from PI:");
      print_sensor_msg(&sensor_data_in);
   }

}

void teensy_serial_setup(){
   Serial.begin(9600);
   HWSERIAL.begin(9600);
}


void set_sensor_msg(int user_input, sensor_data_t *data_ptr) {
   data_ptr->wheel_speed = user_input;
   data_ptr->imu_angle= user_input+1;
   data_ptr->right_URF = user_input+2;
   data_ptr->left_URF = user_input+3;
}


void read_from_pi(sensor_data_t *data_ptr) {
   byte data_buffer[FLOAT];

   HWSERIAL.readBytes(data_buffer, SHORT);
   data_ptr->wheel_speed = (short) *data_buffer;
   HWSERIAL.readBytes(data_buffer, SHORT); // float data!
   data_ptr->imu_angle = (short) *data_buffer;
   HWSERIAL.readBytes(data_buffer, SHORT);
   data_ptr->right_URF = (short) *data_buffer;
   HWSERIAL.readBytes(data_buffer, SHORT);
   data_ptr->left_URF = (short) *data_buffer;
}


void print_sensor_msg(sensor_data_t *data_ptr) {
   Serial.printf("\t%i", data_ptr->wheel_speed);
   Serial.printf("\t%i", data_ptr->imu_angle);
   Serial.printf("\t%i", data_ptr->right_URF);
   Serial.printf("\t%i\n", data_ptr->left_URF);
}
