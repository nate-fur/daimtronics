/* This is a template file for writing autonomous algorithms in
 * ROS. There are messages throughout this file that suggest how 
 * to add additional code.
 */

#include "system_data.h"
#include "semi_truck/Teensy_Sensors.h"
#include "semi_truck/Teensy_Actuators.h"

#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
    
void rplidar_cb(const sensor_msgs::LaserScan &msg);

void lidar_lite_cb(const sensor_msgs::LaserScan &msg);

void teensy_sensors_cb(const semi_truck::Teensy_Sensors &msg);

/* ANY ADDITIONAL SUBSCRIBER CALLBACKS PROTOTYPES SHOULD BE DEFINED HERE */


int main(int argc, char **argv) {
   ros::init(argc, argv, "truck_template_node");
   ros::NodeHandle nh("~");

   ros::Subscriber rplidar_subscriber = nh.subscribe("rplidar_scane", 1024,
    rplidar_cb);

   ros::Subscriber lidar_lite_subscriber = nh.subscribe("lidar_lite_scan", 1024,
    lidar_lite_cb);

   ros::Subscriber teensy_sensors_subscriber = nh.subscribe("teensy_sensor_data", 1024,
    teensy_sensors_cb);

   /* ANY ADDITIONAL SUBSCRIBERS TO TOPICS SHOULD BE INITIALIZED HERE */

   ros::Rate loop_rate(50);

   while (ros::ok()) {
      
      /* AUTONOMOUS ALGORITHMS SHOULD BE WRITTEN HERE */

      /* spinOnce() will force all of the subscriber callbacks to process their data */
      ros::spinOnce();
      loop_rate.sleep();
   }
}


void rplidar_cb(const sensor_msgs::LaserScan &msg) {

}

void lidar_lite_cb(const sensor_msgs::LaserScan &msg) {

} 

void teensy_sensors_cb(const semi_truck::Teensy_Sensors &msg) {

}

/* ANY ADDITIONAL SUBSCRIBER CALLBACKS SHOULD BE IMPLEMENTED HERE */
