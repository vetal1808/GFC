#include "telemetry.h"

void telemetry_update(){
	static uint8_t skip_counter = 0;
	const uint8_t skip_num = 3;
	if(skip_counter<skip_num){
		skip_counter++;
		return;
	}
	skip_counter = 0;
	transmit_masked_channal();
}
void load_axis_errors(Quaternion * quaternion){
	EuclidAngles axis_error;
	Vector3_int16 axis_error_arc_min;
	quaternionDecomposition(quaternion, (Vector3 *)&axis_error);
	convertRadiansToIntArcMinutes(&axis_error, &axis_error_arc_min);
	set_tx_channals((int16_t *)&axis_error_arc_min,RESERVED_CHANNAL0,3);
}
void load_euclid_angles_derivative(Quaternion * q, Vector3 local_rotation){
	EuclidAngles euclid_angles;
	convertQuaternionToEuclidAngles(q, &euclid_angles);
	computeEuclidAnglesDerivative(&euclid_angles, &local_rotation);
	Vector3_int16 lol;
	lol.x = (int16_t)(local_rotation.x*60.0f);
	lol.y = (int16_t)(local_rotation.y*60.0f);
	lol.z = (int16_t)(local_rotation.z*60.0f);
	set_tx_channals((int16_t *)&lol,RESERVED_CHANNAL0,3);
}
void load_euclid_angles(Quaternion * quaternion){
	EuclidAngles euclid_angles;
	Vector3_int16 euclid_angles_arc_min;
	convertQuaternionToEuclidAngles(quaternion, &euclid_angles);
	convertRadiansToIntArcMinutes(&euclid_angles, &euclid_angles_arc_min);
	set_tx_channals((int16_t *)&euclid_angles_arc_min,ANGLES,3);
}
