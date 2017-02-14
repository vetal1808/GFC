#include "quaternion.h"
#include "math.h"

void quaternionMultiplication(Quaternion * leftQ, Quaternion * rightQ, Quaternion * result) {
	result->q0 = leftQ->q0*rightQ->q0 - leftQ->q1*rightQ->q1 - leftQ->q2*rightQ->q2 - leftQ->q3*rightQ->q3;
	result->q1 = leftQ->q2*rightQ->q3 - leftQ->q3*rightQ->q2 + leftQ->q0*rightQ->q1 + leftQ->q1*rightQ->q0;
	result->q2 = leftQ->q3*rightQ->q1 - leftQ->q1*rightQ->q3 + leftQ->q0*rightQ->q2 + leftQ->q2*rightQ->q0;
	result->q3 = leftQ->q1*rightQ->q2 - leftQ->q2*rightQ->q1 + leftQ->q0*rightQ->q3 + leftQ->q3*rightQ->q0;
}
void convertEuclidAnglesToQuaterion(EuclidAngles euclid_angles, Quaternion * quaternion){
	euclid_angles.pitch*=0.5;
	euclid_angles.roll*=0.5;
	euclid_angles.yaw*=0.5;
	float sin_fi = sinf(euclid_angles.pitch);
	float sin_teta = sinf(euclid_angles.roll);
	float sin_psi = sinf(euclid_angles.yaw);
	float cos_fi = cosf(euclid_angles.pitch);
	float cos_teta = cosf(euclid_angles.roll);
	float cos_psi = cosf(euclid_angles.yaw);
	float cos_fi_cos_teta = cos_fi * cos_teta;
	float sin_fi_cos_teta = sin_fi * cos_teta;
	float cos_fi_sin_teta = cos_fi * sin_teta;
	float sin_fi_sin_teta = sin_fi * sin_teta;
	quaternion->q0 = - cos_fi_cos_teta * cos_psi - sin_fi_sin_teta * sin_psi;
	quaternion->q1 = sin_fi_cos_teta * cos_psi - cos_fi_sin_teta * sin_psi;
	quaternion->q2 = cos_fi_sin_teta * cos_psi + sin_fi_cos_teta * sin_psi;
	quaternion->q3 = cos_fi_cos_teta * sin_psi - sin_fi_sin_teta * cos_psi;
}
void convertQuaternionToEuclidAngles(Quaternion * q, EuclidAngles * euclid_angles)
{
	euclid_angles->pitch = atan2f ((q->q0*q->q1+q->q2*q->q3),0.5f-(q->q1*q->q1+q->q2*q->q2));
	euclid_angles->roll = -asinf (2.0f*(q->q0*q->q2-q->q3*q->q1));
	euclid_angles->yaw = -atan2f ((q->q0*q->q3+q->q1*q->q2),0.5f-(q->q2*q->q2+q->q3*q->q3));
}
void quaternionDecomposition(Quaternion * quaternion, Vector3  * rotationAngles){

	if (quaternion->q0 <0.0f) {
		quaternion->q0 = -quaternion->q0;
		quaternion->q1 = -quaternion->q1;
		quaternion->q2 = -quaternion->q2;
		quaternion->q3 = -quaternion->q3;
	}
	float angle = acosf(quaternion->q0);//!TODO maybe possible replace arccos to simpler and faster function
	if (angle < 0.05) {
		rotationAngles->x = quaternion->q1;
		rotationAngles->y = quaternion->q2;
		rotationAngles->z = quaternion->q3;
	} else {
		float tmp = invSqrt(1 - quaternion->q0 * quaternion->q0);
		rotationAngles->x = angle*quaternion->q1 * tmp;
		rotationAngles->y = angle*quaternion->q2 * tmp;
		rotationAngles->z = angle*quaternion->q3 * tmp;
	}


}
void quaternionComposition(EuclidAngles rotationAngles, Quaternion * quaternion){

	radianNormalaize(&rotationAngles.pitch);
	radianNormalaize(&rotationAngles.roll);
	radianNormalaize(&rotationAngles.yaw);

	quaternion->q0 = 0.0f;
	quaternion->q1 = rotationAngles.pitch;
	quaternion->q2 = rotationAngles.roll;
	quaternion->q3 = rotationAngles.yaw;
	normalaseQuaternion(quaternion);
	float angle = sqrt(rotationAngles.pitch * rotationAngles.pitch + rotationAngles.roll * rotationAngles.roll + rotationAngles.yaw * rotationAngles.yaw);
	angle *= 0.5f;
	quaternion->q0 = -cosf(angle);
	float sin_half_angle = sinf(angle);
	quaternion->q1 *= sin_half_angle;
	quaternion->q2 *= sin_half_angle;
	quaternion->q3 *= sin_half_angle;
	normalaseQuaternion(quaternion);
}
void convertRadiansToIntArcMinutes(EuclidAngles * euclid_angles, Vector3_int16 * arc_min){
	arc_min->x = (int16_t)(euclid_angles->pitch * rad_to_minuteArc);
	arc_min->y = (int16_t)(euclid_angles->roll * rad_to_minuteArc);
	arc_min->z = (int16_t)(euclid_angles->yaw * rad_to_minuteArc);
}
void convertIntArcMinutesToRadians(Vector3_int16 * arc_min, EuclidAngles * euclid_angles){
	euclid_angles->pitch = (float) arc_min->x * minuteArc_to_rad;
	euclid_angles->roll = (float) arc_min->y * minuteArc_to_rad;
	euclid_angles->yaw = (float) arc_min->z * minuteArc_to_rad;
}
void normalaseQuaternion(Quaternion * q){

	float tmp = q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3;
	float recipNorm = invSqrt(tmp);
	q->q0 *= recipNorm;
	q->q1 *= recipNorm;
	q->q2 *= recipNorm;
	q->q3 *= recipNorm;
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void radianNormalaize(float * angle){
	if (fabsf(*angle) > M_PI) {
		int8_t full_periods = ((int8_t)(*angle * M_1_PI));
		*angle -= ((2.0f * M_PI) * (float)full_periods);
	}
}
void computeEuclidAnglesDerivative(EuclidAngles * euclid_angles, Vector3 * local_rotation){
	Vector3 tmp;
	float sin_yaw = sinf(euclid_angles->yaw);
	float sin_pitch = sinf(euclid_angles->pitch);
	float cos_yaw = cosf(euclid_angles->yaw);
	float cos_pitch = cosf(euclid_angles->pitch);
	float _1_cos_pitch = 1.0f / cos_pitch;
	tmp.x = (cos_yaw*local_rotation->x + sin_yaw * local_rotation->y)*_1_cos_pitch;
	tmp.y = (-sin_yaw*cos_pitch*local_rotation->x + cos_yaw*cos_pitch*local_rotation->y)*_1_cos_pitch;
	tmp.z = (cos_yaw*sin_pitch*local_rotation->x+sin_yaw*sin_pitch*local_rotation->y - cos_pitch*local_rotation->z)*_1_cos_pitch;
	*local_rotation = tmp;
}
void rotateVector3ByQuatern(Quaternion * q, Vector3 * return_vector)
{
	float q1q1, q2q2, q3q3,
	q0q1,q0q2,q0q3,
	q1q2,q1q3,
	q2q3;
	q0q1 = q->q0 * q->q1;
	q0q2 = q->q0 * q->q2;
	q0q3 = q->q0 * q->q3;
	q1q1 = q->q1 * q->q1;
	q1q2 = q->q1 * q->q2;
	q1q3 = q->q1 * q->q3;
	q2q2 = q->q2 * q->q2;
	q2q3 = q->q2 * q->q3;
	q3q3 = q->q3 * q->q3;
	float _x = return_vector->x, _y = return_vector->y, _z = return_vector->z;
	return_vector->x = 2.0f * ((0.5f - q2q2 - q3q3)*_x + (q1q2 - q0q3)*(_y)		 + (q1q3 + q0q2)*(_z));
	return_vector->y = 2.0f * ((q1q2 + q0q3)*_x 		 + (0.5f - q1q1 - q3q3)*(_y) + (q2q3 - q0q1)*(_z));
	return_vector->z = 2.0f * ((q1q3 - q0q2)*_x 		 + (q2q3 + q0q1)*(_y) 	     + (0.5f - q1q1 - q2q2)*(_z));
}
