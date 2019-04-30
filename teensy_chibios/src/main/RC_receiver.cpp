//
// Created by nate on 4/2/19.
//

#include <Arduino.h>
#include "include/RC_receiver.h"

float SW1_high_time = 0;
volatile unsigned long SW1_time = 0;

/**
 * @brief This is the primary function reading Switch 1 on the receiver. It reads a
 * PWM signal that the RC receiver receives from the RC controller. Based on the
 * specific timing of the PWM, a deadman mode (either deadman switch pressed or
 * not pressed) is selected to send to the rest of the platform.
 *
 * @return the deadman mode of the semi-truck based on RC receiver signal
 */
int16_t RC_receiver_SW1_fn(short PWM_PIN) {

    short mode;
    short val = digitalRead(PWM_PIN);

    if(val==HIGH){
        SW1_time = micros();
        //Serial.print("SW1_time =");
        //Serial.print(SW1_time);
    } else if(val==LOW){
        volatile unsigned long f_time = micros();
        //Serial.print(" SW1_f_time = ");
        //Serial.print(f_time);
        SW1_high_time = (float)(f_time - SW1_time)/1000;
        //Serial.print(" SW1_high_time = ");
        //Serial.print(SW1_high_time);
    }
    if (SW1_high_time >= 1.35 && SW1_high_time <= 1.45){
        mode = 1;
    }
    else if(SW1_high_time >= 1.05 && SW1_high_time <= 1.15){
        mode = 2;
    }
    else{
        mode = 0;
    }
#ifdef DEBUG
    Serial.print("SW1 mode = ");
    Serial.print(mode);
#endif
    return mode;
}

float SW2_high_time = 0;
volatile unsigned long SW2_time = 0;
/**
 * @brief This is the primary function controlling the RC receiver. It reads a
 * PWM signal that the RC receiver receives from the RC controller. Based on
 * the specific timing of the PWM, a drive mode (either manual or one of the
 * autonomous algorithms on the Pi) is selected to control the vehicle.
 *
 * @return the driving mode of the semi-truck based on RC receiver signal
 */
int16_t RC_receiver_SW2_fn(short PWM_PIN) {

    short mode;
    short val = digitalRead(PWM_PIN);

    if(val==HIGH){
        SW2_time = micros();

    } else{
        unsigned long f_time = micros();
        SW2_high_time = (float)(f_time - SW2_time)/1000;
    }
    if (SW2_high_time >= 1.45 && SW2_high_time <= 1.55){
        mode = 1;
    }
    else if(SW2_high_time >= 1.05 && SW2_high_time <= 1.15){
        mode = 2;
    }
    else{
        mode = 0;
    }
#ifdef DEBUG
    Serial.print("SW2 mode = ");
    Serial.print(mode);
#endif
    return mode;
}


float SW3_high_time = 0;
volatile unsigned long SW3_time = 0;
/**
 * @brief This is the primary function controlling the RC receiver. It reads a
 * PWM signal that the RC receiver receives from the RC controller. Based on
 * the specific timing of the PWM, a drive mode (either manual or one of the
 * autonomous algorithms on the Pi) is selected to control the vehicle.
 *
 * @return the driving mode of the semi-truck based on RC receiver signal
 */
int16_t RC_receiver_SW3_fn(short PWM_PIN) {

    short mode;
    short val = digitalRead(PWM_PIN);

    if(val==HIGH){
        SW3_time = micros();

    } else{
        unsigned long f_time = micros();
        SW3_high_time = (float)(f_time - SW3_time)/1000;
    }
    if (SW3_high_time >= 1.45 && SW3_high_time <= 1.55){
        mode = 1;
    }
    else if(SW3_high_time >= 1.85 && SW3_high_time <= 1.95){
        mode = 2;
    }
    else if(SW3_high_time >= 1.05 && SW3_high_time <= 1.15){
        mode = 3;
    }
    else{
        mode = 0;
    }
#ifdef DEBUG
    Serial.print("SW3 mode = ");
    Serial.print(mode);
#endif
    return mode;
}

void RC_receiver_setup() {

}

