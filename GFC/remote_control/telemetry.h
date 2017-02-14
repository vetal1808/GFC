#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "radio_channal.h"
#include "helpers.h"
#include "quaternion.h"


//TX mnemonic
#define ANGLES 0
#define PITCH 0
#define ROLL 1
#define YAW 2

#define ANGLES_PIDS 3
#define PICTH_PID 3
#define PROP_PITCH 3
#define INTEGR_PITCH 4
#define DIFF_PITCH 5

#define ROLL_PID 6
#define PROP_ROLL 6
#define INTEGR_ROLL 7
#define DIFF_ROLL 8

#define YAW_PID 9
#define PROP_YAW 9
#define INTEGR_YAW 10
#define DIFF_YAW 11

#define LOOP_TIME 12
#define ALTITUDE 13
#define ALTITUDE_VELOCITY 14

#define RESERVED_CHANNAL0 15
#define RESERVED_CHANNAL1 (RESERVED_CHANNAL0 + 1)
#define RESERVED_CHANNAL2 (RESERVED_CHANNAL0 + 2)
#define RESERVED_CHANNAL3 (RESERVED_CHANNAL0 + 3)
#define RESERVED_CHANNAL4 (RESERVED_CHANNAL0 + 4)
#define RESERVED_CHANNAL5 (RESERVED_CHANNAL0 + 5)


void telemetry_update();
void load_axis_errors(Quaternion * quaternion);
void load_euclid_angles(Quaternion * quaternion);
void load_euclid_angles_derivative(Quaternion * q, Vector3 local_rotation);
#endif
