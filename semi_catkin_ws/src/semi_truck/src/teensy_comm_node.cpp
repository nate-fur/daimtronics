#include "teensy_comm_node.h"
#include "system_data.h"
#include "semi_truck/Teensy_Sensors.h"
#include "semi_truck/Teensy_Actuators.h"

#include <wiringSerial.h>
#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define NUM_MSGS 4
#define MSG_SIZE 2
#define BUFF_SIZE 50
#define UART "/dev/ttyS0"
#define BAUDRATE 9600

using namespace std;
static int serial;

void sensor_cb(const semi_truck::Teensy_Sensors &msg);

void actuator_cb(const semi_truck::Teensy_Actuators &msg);


int main(int argc, char **argv) {
   char avail;
   bool updated_values = false;
   short receiveBuffer[BUFF_SIZE] = {0};
	serial = serialOpen(UART, 9600);
   printf("Starting serial communication...\n");

   ros::init(argc, argv, "teensy_comm_node");
   ros::NodeHandle nh("~");

   semi_truck::Teensy_Sensors sensor_data;
   semi_truck::Teensy_Actuators actuator_data;

   ros::Publisher publisher = nh.advertise<semi_truck::Teensy_Sensors>
    ("sensor_data", 1024);

   ros::Subscriber subscriber = nh.subscribe("sensor_data_modified", 1024,
    sensor_cb);

   ros::Rate loop_rate(1);
   while (ros::ok()) {
      ROS_INFO("new loop!");

      if ((avail = serialDataAvail(serial)) > 0) { 
         printf("bytes available: %i\n", avail);
         read_from_teensy(serial, sensor_data);
         //print_buffer(receiveBuffer);
         //update_values(receiveBuffer);
         //updated_values = true;
      }
      

      if (updated_values) {
         write_to_teensy(serial, sensor_data);
         updated_values = false;
      }

      //print_sensors(sensor_data);
      publisher.publish(sensor_data);

      ros::spinOnce();
      loop_rate.sleep();
   }
}


short read_sensor_msg(int serial) {
   short byte;
   short sensor_msg = 0;

   for (char i = 0; i < MSG_SIZE; i++) {
      byte = serialGetchar(serial);
      byte <<= 8*i; 
      sensor_msg |= byte;
   }

   //printf("sensor message: %x\n", sensor_msg);
   return sensor_msg;
}


void read_from_teensy(int serial, semi_truck::Teensy_Sensors &sensors) {
   short sensor_msg;

   sensors.wheel_speed = read_sensor_msg(serial);
   sensors.imu_angle = read_sensor_msg(serial);
   sensors.right_URF = read_sensor_msg(serial);
   sensors.left_URF = read_sensor_msg(serial);
}


void write_sensor_msg(int serial, short sensor_val) {
   char *byte_ptr = (char*)&sensor_val;

   for (char i = 0; i < MSG_SIZE; i++) {
      serialPutchar(serial, *byte_ptr++); // write 1 byte for each byte in val
   }
}


void write_to_teensy(int serial, const semi_truck::Teensy_Sensors &sensors) {
   printf("\n********SENDING MESSAGE*********\n");
   write_sensor_msg(serial, sensors.wheel_speed);
   write_sensor_msg(serial, sensors.imu_angle);
   write_sensor_msg(serial, sensors.right_URF);
   write_sensor_msg(serial, sensors.left_URF);
}


void update_sensors(semi_truck::Teensy_Sensors &sensors) {
   sensors.wheel_speed += 48;
   sensors.imu_angle += 48;
   sensors.right_URF += 48;
   sensors.left_URF += 48;
}


void print_sensors(const semi_truck::Teensy_Sensors &sensors) {
   printf("wheel_speed:\t %hu\n", sensors.wheel_speed);
   printf("imu_angle:\t %hu\n", sensors.imu_angle);
   printf("right_URF:\t %hu\n", sensors.right_URF);
   printf("left_URF:\t %hu\n", sensors.left_URF);
}


void sensor_cb(const semi_truck::Teensy_Sensors &msg) {
   ROS_INFO("Got Message!");
   ROS_INFO("wheel speed:\t [%i]", msg.wheel_speed);
   ROS_INFO("imu angle:\t [%i]", msg.imu_angle);
   ROS_INFO("right_URF:\t [%i]", msg.right_URF);
   ROS_INFO("left_URF:\t [%i]", msg.left_URF);
   print_sensors(msg);
   write_to_teensy(serial, msg);
}


void actuator_cb(const semi_truck::Teensy_Actuators &msg) {
   ROS_INFO("Got Message!");
   ROS_INFO("motor output:\t [%i]", msg.motor_output);
   ROS_INFO("steer output:\t [%i]", msg.steer_output);
   ROS_INFO("fifth wheel:\t [%i]", msg.desired_fifth_wheel);
}
