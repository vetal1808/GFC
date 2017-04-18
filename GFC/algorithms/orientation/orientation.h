#ifndef __ORIENTATION_H__
#define __ORIENTATION_H__

#include "mpu6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"

#define MAGNET_SKIP 10
#define COMPASS_USE 1
#define COMPASS_DO_NOT_USE 0

void Orientation_InitSensors(uint8_t _use_compass);
void Orientation_Update ();
void Orientation_getGlobalAccel(Vector3 *_global_accel);
void Orientation_getLocalGyro(Vector3 *_local_gyro);
#define Orientation_getQuaternion(q) (GetOffsetedMadgwickAHRSQuaternion(q))

#endif
