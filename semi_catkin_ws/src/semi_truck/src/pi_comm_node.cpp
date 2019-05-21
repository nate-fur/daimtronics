#include "pi_comm_node.h"
#include "system_data.h"
#include "semi_truck/Teensy_Sensors.h"
#include "semi_truck/Teensy_Actuators.h"

#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define RELAY_PIN 7
#define SYNC_VALUE -32000
#define NUM_MSGS 4
#define SHORT_SIZE 2
#define FLOAT_SIZE 4
#define SENSOR_DATA_SIZE 10
#define BUFF_SIZE 50
#define UART "/dev/ttyS0"
#define BAUDRATE 9600
#define DEBUG 

// number of times the pi should read from serial for every time it writes 
// to it. Helps with serial buffer overflow issues
#define READ_CYCLES 2 
// in hz; should match with controller node
#define LOOP_FREQUENCY 10

using namespace std;
static int serial;


int main(int argc, char **argv) {
   char avail;
   short synchronized = false;
   bool updated_values = false;
   short receiveBuffer[BUFF_SIZE] = {0};
	serial = serialOpen(UART, 9600);
   printf("Starting serial communication...\n");

   wiringPiSetup();
   pinMode(RELAY_PIN, OUTPUT);
   ros::init(argc, argv, "pi_comm_node");
   ros::NodeHandle nh("~");
   semi_truck::Teensy_Sensors sensor_data;
   ros::Publisher publisher = nh.advertise<semi_truck::Teensy_Sensors>
    ("teensy_sensor_data", 1024);

   semi_truck::Teensy_Actuators actuator_data;
   ros::Subscriber subscriber = nh.subscribe("teensy_actuator_data", 1,
    actuator_cb);

   ros::WallRate loop_rate(LOOP_FREQUENCY);
   while (ros::ok()) {

      for (int i = 0; i < READ_CYCLES; i++) {

         if (synchronized = pi_sync()) { // avoids data becoming mismatched
            avail = serialDataAvail(serial);
            if (avail >= SENSOR_DATA_SIZE) { // need to have a full set of data
               read_from_teensy(serial, sensor_data);
               print_sensors(sensor_data);
               publisher.publish(sensor_data);
            }
         }

      }

      //printf("writing to pin: %i\n", sensor_data.drive_mode_2);
      digitalWrite(RELAY_PIN, sensor_data.drive_mode_2);
      
      ros::spinOnce();


      //print_sensors(sensor_data);

      loop_rate.sleep();
   }
}

bool pi_sync() {
   short data;

   for (int i; i < 100; i++) {
      if ((data = read_sensor_msg(serial, 2)) == SYNC_VALUE) {
         return true;
      }
   }

   return false;
}


short read_sensor_msg(int serial, char num_bytes) {
   short byte;
   short sensor_msg = 0;

   for (char i = 0; i < num_bytes; i++) {
      byte = serialGetchar(serial);
      byte <<= 8*i; 
      sensor_msg |= byte;
   }

   //printf("sensor message: %x\n", sensor_msg);
   return sensor_msg;
}


void read_from_teensy(int serial, semi_truck::Teensy_Sensors &sensors) {
   short sensor_msg;

   sensors.imu_angle = read_sensor_msg(serial, SHORT_SIZE);
   sensors.wheel_speed = read_sensor_msg(serial, SHORT_SIZE);
   sensors.right_TOF = read_sensor_msg(serial, SHORT_SIZE);
   sensors.left_TOF = read_sensor_msg(serial, SHORT_SIZE);
   sensors.rear_TOF = read_sensor_msg(serial, SHORT_SIZE);
   sensors.drive_mode_1 = read_sensor_msg(serial,SHORT_SIZE);
   sensors.drive_mode_2 = read_sensor_msg(serial,SHORT_SIZE);
}


void write_actuator_msg(int serial, short actuator_val, char num_bytes) {
   char *byte_ptr = (char*)&actuator_val;

   for (char i = 0; i < num_bytes; i++) {
      serialPutchar(serial, *byte_ptr++); // write 1 byte for each byte in val
   }
}


void write_to_teensy(int serial, const semi_truck::Teensy_Actuators &actuators) {
   write_actuator_msg(serial, actuators.motor_output, SHORT_SIZE);
   write_actuator_msg(serial, actuators.steer_output, SHORT_SIZE);
   write_actuator_msg(serial, actuators.fifth_output, SHORT_SIZE);
}


void update_sensors(semi_truck::Teensy_Sensors &sensors) {
   sensors.wheel_speed += 48;
   sensors.imu_angle += 48;
   sensors.right_TOF += 48;
   sensors.left_TOF += 48;
}


void print_sensors(const semi_truck::Teensy_Sensors &sensors) {
   ROS_INFO("imu angle:\t [%i]", sensors.imu_angle);
   ROS_INFO("wheel speed:\t [%i]", sensors.wheel_speed);
   ROS_INFO("right_TOF:\t [%i]", sensors.right_TOF);
   ROS_INFO("left_TOF:\t [%i]", sensors.left_TOF);
   ROS_INFO("rear_TOF:\t [%i]", sensors.rear_TOF);
   ROS_INFO("drive_mode_1:\t [%i]", sensors.drive_mode_1);
   ROS_INFO("drive_mode_2:\t [%i]\n", sensors.drive_mode_2);
}


void print_actuators(const semi_truck::Teensy_Actuators &actuators) {
   ROS_INFO("motor output:\t [%i]", actuators.motor_output);
   ROS_INFO("steer output:\t [%hu]", actuators.steer_output);
   ROS_INFO("fifth output:\t [%hu]\n", actuators.fifth_output);
}


void sensor_cb(const semi_truck::Teensy_Sensors &msg) {
   #ifdef DEBUG
   ROS_INFO("Got Sensor Message!");
   print_sensors(msg);
   #endif
}


void actuator_cb(const semi_truck::Teensy_Actuators &msg) {
   #ifdef DEBUG
   ROS_INFO("Got Actuator Message!");
   print_actuators(msg);
   printf("\n********SENDING MESSAGE*********\n");
   printf("Time: %lf\n", ros::WallTime::now().toSec());
   #endif
   write_to_teensy(serial, msg);
}
