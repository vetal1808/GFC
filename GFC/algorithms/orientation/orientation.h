#ifndef __ORIENTATION_H__
#define __ORIENTATION_H__

#include "types.h"
#include "quaternion.h"

#define MAGNET_SKIP 10
#define COMPASS_USE 1
#define COMPASS_DO_NOT_USE 0

void Orientation_InitSensors(uint8_t _use_compass);
void Orientation_Update ();
void Orientation_getGlobalAccel(Vector3 *_global_accel);
void Orientation_getLocalGyro(Vector3 *_local_gyro);
void Orientation_getQuaternion(Quaternion * q);


#endif
