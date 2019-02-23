#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "system_data.h"
#include "semi_truck/Teensy_Sensors.h"
#include "semi_truck/Teensy_Actuators.h"

using namespace std;

void sensor_cb(const semi_truck::Teensy_Sensors &msg) {
   ROS_INFO("Got Message!");
   ROS_INFO("wheel speed:\t [%i]", msg.wheel_speed);
   ROS_INFO("imu angle:\t [%i]", msg.imu_angle);
   ROS_INFO("right_URF:\t [%i]", msg.right_URF);
   ROS_INFO("left_URF:\t [%i]", msg.left_URF);
}

void actuator_cb(const semi_truck::Teensy_Actuators &msg) {
   ROS_INFO("Got Message!");
   ROS_INFO("motor output:\t [%i]", msg.motor_output);
   ROS_INFO("steer output:\t [%i]", msg.steer_output);
   ROS_INFO("fifth wheel:\t [%i]", msg.desired_fifth_wheel);
}



int main(int argc, char **argv) {
   ros::init(argc, argv, "teensy_comm_node");
   ros::NodeHandle nh("~");

   // Simulate data that might come over the UART from the teensy
   system_data_t system_data = {0};
   system_data.wheel_speed = 1;
   system_data.imu_angle = 2;
   system_data.right_URF = 3;
   system_data.left_URF = 4;

   // Need to place the teensy data into the sensor message to comm with ROS
   semi_truck::Teensy_Sensors sensor_data;
   sensor_data.wheel_speed = system_data.wheel_speed;
   sensor_data.imu_angle = system_data.imu_angle;
   sensor_data.right_URF = system_data.right_URF;
   sensor_data.left_URF = system_data.left_URF;

   // Actuator Data
   semi_truck::Teensy_Actuators actuator_data;

   ros::Publisher publisher = nh.advertise<semi_truck::Teensy_Sensors>
    ("sensor_data", 1024);

   ros::Subscriber subscriber = nh.subscribe("actuator_data", 1024,
    actuator_cb);



   ros::Rate loop_rate(1);

   while (ros::ok()) {

      publisher.publish(sensor_data);

      ros::spinOnce();
      loop_rate.sleep();
   }
}
