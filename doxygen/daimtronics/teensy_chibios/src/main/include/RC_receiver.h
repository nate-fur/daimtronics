#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <stdint.h>
int16_t RC_receiver_SW1_fn(short PWM_PIN);

int16_t RC_receiver_SW2_fn(short PWM_PIN);

int16_t RC_receiver_SW3_fn(short PWM_PIN);

void RC_receiver_setup();

#endif