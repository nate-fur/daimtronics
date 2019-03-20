/* UART Example, any character received on either the real
   serial port, or USB serial (or emulated serial to the
   Arduino Serial Monitor when using non-serial USB types)
   is printed as a message to both ports.

   This example code is in the public domain.
*/
//#include "serial.h"
#define HWSERIAL Serial1
#define OFFSET 48
#define NUM_MSGS 4
#define MSG_SIZE 2

int ledPin = 13;

typedef struct sensor_data_t {
  short wheel_speed;
  short imu_angle;
  short right_URF;
  short left_URF;
} sensor_data_t;


void set_sensor_msg(int user_input, sensor_data_t *data_ptr) {
  data_ptr->wheel_speed = user_input;
  data_ptr->imu_angle= user_input+1;
  data_ptr->right_URF = user_input+2;
  data_ptr->left_URF = user_input+3;
}


void receive_sensor_msg(sensor_data_t *data_ptr) {
  byte data_buffer[MSG_SIZE];
  
  HWSERIAL.readBytes(data_buffer, MSG_SIZE);
  data_ptr->wheel_speed = (short) *data_buffer;
  HWSERIAL.readBytes(data_buffer, MSG_SIZE);
  data_ptr->imu_angle = (short) *data_buffer;
  HWSERIAL.readBytes(data_buffer, MSG_SIZE);
  data_ptr->right_URF = (short) *data_buffer;
  HWSERIAL.readBytes(data_buffer, MSG_SIZE);
  data_ptr->left_URF = (short) *data_buffer;
}


void print_sensor_msg(sensor_data_t *data_ptr) {
  Serial.printf("\t%i", data_ptr->wheel_speed);  
  Serial.printf("\t%i", data_ptr->imu_angle);   
  Serial.printf("\t%i", data_ptr->right_URF);  
  Serial.printf("\t%i\n", data_ptr->left_URF);  
}


void setup() {
  
  Serial.begin(9600);
  HWSERIAL.begin(9600);
  
}


void loop() {
  
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
    set_sensor_msg(user_input, &sensor_data_out);
    
    if (HWSERIAL.availableForWrite()) {
      short_ptr = (short*)&sensor_data_out;
      Serial.println("Sending to PI:"); 
      Serial.printf("%hu", *short_ptr++);
      Serial.printf("\t%hu", *short_ptr++);
      Serial.printf("\t%hu", *short_ptr++);
      Serial.printf("\t%hu\n", *short_ptr++);
      HWSERIAL.write((char*)&sensor_data_out, 8);
    }
	}

  // communicate with Pi and the ROS network
	if (HWSERIAL.available() > 0) {
		receive_sensor_msg(&sensor_data_in);
    
		Serial.println("Received from PI:");
    print_sensor_msg(&sensor_data_in);
	}
 
}
