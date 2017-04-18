#ifndef __QUATERNION_H__
#define __QUATERNION_H__
#include "types.h"


typedef struct{
   float q0, q1, q2, q3;
} Quaternion;


#define RAD_TO_ARC_MINUTE 3437.746770784939f
#define ARC_MINUTE_TO_RAD 2.908882086e-4f
typedef struct{
   float pitch, roll, yaw;
} EuclidAngles;


void quaternionMultiplication(Quaternion * leftQ, Quaternion * rightQ, Quaternion * result);
void convertEuclidAnglesToQuaterion(EuclidAngles euclid_angles, Quaternion * quaternion);
void convertQuaternionToEuclidAngles(Quaternion * q, EuclidAngles * euclid_angles);
void quaternionDecomposition(Quaternion * quaternion, Vector3  * rotationAngles);
void quaternionComposition(EuclidAngles *rotationAngles, Quaternion * quaternion);
void normalaseQuaternion(Quaternion * q);

void convertRadiansToIntArcMinutes(EuclidAngles * euclid_angles, Vector3_int16 * arc_min);
void convertIntArcMinutesToRadians(Vector3_int16 * arc_min, EuclidAngles * euclid_angles);
float invSqrt(float x);
void radianNormalaize(float * angle);
void computeEuclidAnglesDerivative(EuclidAngles * euclid_angles, Vector3 * local_rotation);
void rotateVector3ByQuatern(Quaternion * q, Vector3 * return_vector);
float projectionOfNormalVectorToGlobalZ(Quaternion * q);
#endif
