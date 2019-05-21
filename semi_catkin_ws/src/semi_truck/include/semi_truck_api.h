/**
 * @file This is the application layer for interfacing with the semi_truck
 * platform. There are three 'setter' functions for setting values to output to
 * the actuators hooked up to the system. There are also five 'getter'
 * functions that read data from sensors hooked up to the system. These
 * functions are designed to be called in the autonomous algorithms
 * implemented in a ROS node file.
 */

#ifndef DAIMTRONICS_SEMI_TRUCK_API_H
#define DAIMTRONICS_SEMI_TRUCK_API_H

#include "Teensy_Actuators.h"
#include "Teensy_Sensors.h"

/**
 * @brief sets the motor output of the actuators object to the passed in value
 * @param actuators a reference to a TeensyActuators object to alter
 * @param value the value to update the actuators with. This value should
 * range from 0 for full reverse power to 180 for full forwards power.
 */
void set_motor_output(semi_truck::Teensy_Actuators &actuators, int16_t value);

/**
 * @brief sets the steer output of the actuators object to the passed in value
 * @param actuators a reference to a TeensyActuators object to alter
 * @param value the value to update the actuators with. This value should
 * range from 0 for a 20 degree angle left to 180 for a 20 degree angle right.
 */
void set_steer_output(semi_truck::Teensy_Actuators &actuators, int16_t value);

/**
 * @brief sets the fifth wheel of the actuators object to the passed in value
 * @param actuators a reference to a TeensyActuators object to alter
 * @param value the value to update the actuators with. 0 for locked and 1
 * for unlocked.
 */
void set_fifth_output(semi_truck::Teensy_Actuators &actuators, uint16_t value);

/**
 * @brief reads the wheel speed of a TeensySensors object
 * @param sensors a reference to a TeensySensors object to read from.
 */
int16_t get_wheel_speed(semi_truck::Teensy_Sensors &sensors);

/**
 * @brief reads the imu angle (degrees) of a TeensySensors object
 * @param sensors a reference to a TeensySensors object to read from.
 */
int16_t get_imu_angle(semi_truck::Teensy_Sensors &sensors);

/**
 * @brief reads the right TOF distance (cm) of a TeensySensors object
 * @param sensors a reference to a TeensySensors object to read from.
 */
int16_t get_right_TOF(semi_truck::Teensy_Sensors &sensors);

/**
 * @brief reads the left TOF distance (cm) of a TeensySensors object
 * @param sensors a reference to a TeensySensors object to read from.
 */
int16_t get_left_TOF(semi_truck::Teensy_Sensors &sensors);

/**
 * @brief reads the rear TOF distance (cm) of a TeensySensors object
 * @param sensors a reference to a TeensySensors object to read from.
 */
int16_t get_rear_TOF(semi_truck::Teensy_Sensors &sensors);


#endif //DAIMTRONICS_SEMI_TRUCK_API_H
