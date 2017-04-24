#include "orientation.h"
#include "mpu6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"

//variables definition
Vector3 global_accel;
Vector3 local_gyro;
Vector3 local_magnet;
uint8_t use_compass;

/*
 * redefine functions from other libs. If needed some convertation, use functions
 */
#define getAccelAndGyro(accel, gyro)(MPU6050_getFloatMotion6(accel, gyro))
#define getMagnet(magnet)(HMC_getCalibratedData(magnet))

void Orientation_InitSensors(uint8_t _use_compass){
	use_compass = _use_compass;
	//MPU6050 init
	MPU6050_initialize();
	MPU6050_setDLPFMode(MPU6050_DLPF_BW_42);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

	int delay_var = 0;
	for (delay_var = 0; delay_var < 2000000; ++delay_var);

	MPU6050_accumulateGyroOffset(3000);
	MPU6050_setSampleRateDiv(3);
	//HMC8553L init
	if (_use_compass == 1)
		HMC_init();
}
void Orientation_Update (){
	static uint8_t magnet_skip_counter = MAGNET_SKIP;
	Vector3 local_accel;
	getAccelAndGyro(&local_accel, &local_gyro);
	if (magnet_skip_counter < MAGNET_SKIP) {
		magnet_skip_counter++;
	} else {
		getMagnet(&local_magnet);
	}
	if (use_compass == 1) {
		MadgwickAHRSupdate(local_gyro.x, local_gyro.y, local_gyro.z,
								local_accel.x, local_accel.y, local_accel.z,
								local_magnet.x, local_magnet.y, local_magnet.z);
	} else {
		MadgwickAHRSupdateIMU(local_gyro.x, local_gyro.y, local_gyro.z,
								local_accel.x, local_accel.y, local_accel.z);
	}
	Quaternion global_quaternion;
	GetOffsetedMadgwickAHRSQuaternion(&global_quaternion);
	global_accel = local_accel;
	rotateVector3ByQuatern(&global_quaternion, &global_accel);

};
void Orientation_getGlobalAccel(Vector3 *_global_accel){
	*_global_accel = global_accel;
}
void Orientation_getLocalGyro(Vector3 *_local_gyro){
	*_local_gyro = local_gyro;
}
void Orientation_getQuaternion(Quaternion * q){
	GetOffsetedMadgwickAHRSQuaternion(q);
}
