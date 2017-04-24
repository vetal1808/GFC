#ifndef __ALTITUDE_ALORITHM_H__
#define __ALTITUDE_ALORITHM_H__
#include "types.h"

void Altitude_Init();
void Altitude_AlgorithmUpdate(float accel_z);
void Altitude_GetVerticalState(float * _altitude, float * _vertical_velocity, float * _altitude_acceleration);
void Altitude_GetVerticalState1(float * _altitude, float * _vertical_velocity, float * _altitude_acceleration);
uint8_t Altitude_GetCurrentSensor();

#endif
