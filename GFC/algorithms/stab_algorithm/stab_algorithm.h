#ifndef _STAB_ALGORITHM_H_
#define _STAB_ALGORITHM_H_

#include "helpers.h"
#include "quaternion.h"
#include <math.h>

#define integration_trottle 250
#define low_trottle 150



void manual_stab(Quaternion * real_quaternion, Vector3 * gyro, Quaternion * RC_quaternion, Vector3 * RC_gyro, uint16_t thrust, Rotor4 * rotor4_thrust);
void update_PID_config(uint16_t raw);
void defaultPIDinit ();
void getAnglesPidSummands(int16_t * array);
void loadPidsTelemetry();
#endif
