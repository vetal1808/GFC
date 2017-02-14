#ifndef __RADIO_CONTROL_H__
#define __RADIO_CONTROL_H__

#include "stdint.h"
#include "helpers.h"
#include "quaternion.h"
//RX mnemonic
#define RC_ANGLES 0
#define RC_PITCH 0
#define RC_ROLL 1
#define RC_YAW 2

#define PID_CONFIG 3

#define THRUST 9

#define MOTOR_MASK 10
#define TX_MASK 11
#define TX_MASK2 12
#define LOST_CONNECTION 13



void RC_update();
void get_RC_state(Quaternion * RC_quaternion_, Vector3 * RC_spin_, int16_t * thrust_, int8_t * last_connect_counter_);
void RC_state_update();
#endif
