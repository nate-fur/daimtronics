//
// Created by nate on 4/2/19.
//

#include <Arduino.h>
#include "include/RC_receiver.h"

//#define DEBUG

float SW1_high_time = 0;
volatile unsigned long SW1_time = 0;
short SW1_mode = 0;
/**
 * @brief This is the primary function reading Switch 1 on the receiver. It reads a
 * PWM signal that the RC receiver receives from the RC controller. Based on the
 * specific timing of the PWM, a deadman mode (either deadman switch pressed or
 * not pressed) is selected to send to the rest of the platform.
 *
 * @return the deadman mode of the semi-truck based on RC receiver signal
 */
int16_t RC_receiver_SW1_fn(short PWM_PIN) {

    short val = digitalRead(PWM_PIN);

    if(val==HIGH){
        SW1_time = micros();
    } else if(val==LOW){
        volatile unsigned long f_time = micros();
        SW1_high_time = (float)(f_time - SW1_time)/1000;
    }
    if (SW1_high_time >= 1.35 && SW1_high_time <= 1.45){
        SW1_mode = 0;
    }
    else if(SW1_high_time >= 1.05 && SW1_high_time <= 1.15){
        SW1_mode = 1;
    }
    else{
        //mode remains unchanged if high time does not fall in region
    }
#ifdef DEBUG
    Serial.print("SW1 mode = ");
    Serial.println(SW1_mode);
#endif
    return SW1_mode;
}

float SW2_high_time = 0;
volatile unsigned long SW2_time = 0;
short SW2_mode = 0;
/**
 * @brief This is the primary function controlling the RC receiver. It reads a
 * PWM signal that the RC receiver receives from the RC controller. Based on
 * the specific timing of the PWM, a drive mode (either manual or one of the
 * autonomous algorithms on the Pi) is selected to control the vehicle.
 *
 * @return the driving mode of the semi-truck based on RC receiver signal
 */
int16_t RC_receiver_SW2_fn(short PWM_PIN) {

    short val = digitalRead(PWM_PIN);

    if(val==HIGH){
        SW2_time = micros();

    } else{
        unsigned long f_time = micros();
        SW2_high_time = (float)(f_time - SW2_time)/1000;
    }
    if (SW2_high_time >= 1.45 && SW2_high_time <= 1.55){
        SW2_mode = 0;
    }
    else if(SW2_high_time >= 1.05 && SW2_high_time <= 1.15){
        SW2_mode = 1;
    }
    else{
        //mode remains unchanged if high time does not fall in region
    }
#ifdef DEBUG
    Serial.print("SW2 mode = ");
    Serial.println(SW2_mode);
#endif
    return SW2_mode;
}


float SW3_high_time = 0;
volatile unsigned long SW3_time = 0;
short SW3_mode = 0;
/**
 * @brief This is the primary function controlling the RC receiver. It reads a
 * PWM signal that the RC receiver receives from the RC controller. Based on
 * the specific timing of the PWM, a drive mode (either manual or one of the
 * autonomous algorithms on the Pi) is selected to control the vehicle.
 *
 * @return the driving mode of the semi-truck based on RC receiver signal
 */
int16_t RC_receiver_SW3_fn(short PWM_PIN) {

    short val = digitalRead(PWM_PIN);

    if(val==HIGH){
        SW3_time = micros();

    } else{
        unsigned long f_time = micros();
        SW3_high_time = (float)(f_time - SW3_time)/1000;
    }
    if (SW3_high_time >= 1.45 && SW3_high_time <= 1.55){
        SW3_mode = 1;
    }
    else if(SW3_high_time >= 1.85 && SW3_high_time <= 1.95){
        SW3_mode = 2;
    }
    else if(SW3_high_time >= 1.05 && SW3_high_time <= 1.15){
        SW3_mode = 3;
    }
    else{
        //mode remains unchanged if high time does not fall in region
    }
#ifdef DEBUG
    Serial.print("SW3 mode = ");
    Serial.println(SW3_mode);
#endif

    return SW3_mode;
}

void RC_receiver_setup() {

}

