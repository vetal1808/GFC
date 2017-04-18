#ifndef __RADIO_CONTROL_H__
#define __RADIO_CONTROL_H__

#include "stdint.h"
#include "helpers.h"
#include "quaternion.h"
//RX mnemonic
#define RC_ANGLES		0
#define RC_PITCH		0
#define RC_ROLL			1
#define RC_YAW			2
#define TRUST			3
#define PID_CONFIG		4
#define MOTOR_MASK		5
#define LOST_CONNECTION	6
#define COMMAND 		7
#define FLIGHT_MODE 	9
#define TX_MASK 		10
#define TX_MASK2		(TX_MASK + 1)
#define TX_MASK3		(TX_MASK + 2)
#define TX_MASK4		(TX_MASK + 3)

#define RC_DIRECT_VELOCITY		0
#define RC_SIDE_VELOCITY		1
#define RC_HEADING				2
#define RC_ALTITUDE_VELOCITY	3


void RC_update();
int16_t RC_getFlightMode();
int16_t RC_getLoseConnectionState();
void RC_getManualState(Quaternion *RC_quaternion_, Vector3 *RC_spin_, uint16_t thrust_);
void RC_getAltitudeHoldState(Quaternion *RC_quaternion_, Vector3 *RC_spin_, float altitude, float alttitude_velocity);
void RC_getPositionHoldState(Vector3 *RC_position_, Vector3 *RC_position_velocity_, float yaw, float yaw_derivative);
void get_RC_state(Quaternion * RC_quaternion_, Vector3 * RC_spin_, int16_t * thrust_, int8_t * last_connect_counter_);
void RC_state_update();
#endif
