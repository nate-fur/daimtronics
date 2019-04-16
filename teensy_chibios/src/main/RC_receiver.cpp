//
// Created by nate on 4/2/19.
//

#include <Arduino.h>
#include "include/RC_receiver.h"

/**
 * @brief This is the primary function controlling the RC receiver. It reads a
 * PWM signal that the RC receiver receives from the RC controller. Based on
 * the specific timing of the PWM, a drive mode (either manual or one of the
 * autonomous algorithms on the Pi) is selected to control the vehicle.
 *
 * @return the driving mode of the semi-truck based on RC receiver signal
 */
int16_t RC_receiver_loop_fn() {

   return 0;
}

void RC_receiver_setup() {

}

