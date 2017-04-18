#ifndef __MOTOR_ALGORITHM_H__
#define __MOTOR_ALGORITHM_H__
#include "types.h"
void MOTORS_InitESC();
void MOTORS_SetMask(uint8_t mask);
void MOTORS_UpdateThrust(Vector3 * torque_of_axis_float, int16_t average_thrust);


#endif
