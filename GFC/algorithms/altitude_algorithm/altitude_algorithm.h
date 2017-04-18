#ifndef __ALTITUDE_ALORITHM_H__
#define __ALTITUDE_ALORITHM_H__
#include "types.h"

void altitudeSensorInit();
void altitudeAlgorithmUpdate(Vector3 global_accel);
void getVerticalState(int32_t * altitude, int32_t * vertical_velocity);
uint8_t Altitude_GetCurrentSensor();

#endif
