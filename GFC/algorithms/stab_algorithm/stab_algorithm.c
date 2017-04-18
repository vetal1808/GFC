/*
 * stab_alg.c
 *
 *  Created on: 02.11.2016
 *      Author: vetal
 */
#include "stab_algorithm.h"
#include "stab_algorithm_config.h"
#include "telemetry.h"
#include "q_config.h"
#include "PID.h"
#define loop_time update_period_in_sec



//mnemonic
#define pitch_PID 0
#define roll_PID 1
#define yaw_PID 2

#define Oz_PID 3
#define Oy_PID 4
#define Ox_PID 5


PID_struct PIDs_array[3];

void angle_stab_algorithm(Quaternion * quaternion, Vector3 * gyro, uint8_t integral_switcher, Vector3 * torque);
void calc_rotor4_thrust(Vector3 torque_of_axis, int16_t average_thrust, Rotor4 * rotors4_thrust);
void angle_stab_algorithm2(Quaternion * quaternion, Vector3 * gyro, uint8_t integral_switcher, Vector3 * torque);

/*
 function calc_rotor4_thrust calculate thrust for X-copter by 3 torque of axis and thrust
*/
void calc_rotor4_thrust(Vector3 torque_of_axis, int16_t average_thrust, Rotor4 * rotors4_thrust){
	rotors4_thrust->LFW = (uint16_t)(average_thrust - (int16_t)((torque_of_axis.x + torque_of_axis.y + torque_of_axis.z)));
	rotors4_thrust->RFC = (uint16_t)(average_thrust - (int16_t)((torque_of_axis.x - torque_of_axis.y - torque_of_axis.z)));
	rotors4_thrust->LBC = (uint16_t)(average_thrust - (int16_t)((-torque_of_axis.x + torque_of_axis.y - torque_of_axis.z)));
	rotors4_thrust->RBW = (uint16_t)(average_thrust - (int16_t)((-torque_of_axis.x - torque_of_axis.y + torque_of_axis.z)));
}
/*
	function angle_stab_algorithm calculate torque for each axis orient platform by given quaternion
*/
void angle_stab_algorithm(Quaternion * quaternion, Vector3 * gyro, uint8_t integral_switcher, Vector3 * torque){

	//Vector3 axis_errors;
	/*
	convertQuaternionToEuclidAngles(quaternion, &axis_errors);
	axis_errors.y = -axis_errors.y;
	axis_errors.z = -axis_errors.z;
	*/
	//quaternionDecomposition(quaternion, &axis_errors);
	if (quaternion->q0<0.0f) {
		quaternion->q0 = -quaternion->q0;
		quaternion->q1 = -quaternion->q1;
		quaternion->q2 = -quaternion->q2;
		quaternion->q3 = -quaternion->q3;
	}
	torque->x = PID(quaternion->q1, gyro->x, &PIDs_array[pitch_PID],  integral_switcher);
	torque->y = PID(quaternion->q2, gyro->y, &PIDs_array[roll_PID],  integral_switcher);
	torque->z = PID(quaternion->q3, gyro->z, &PIDs_array[yaw_PID],  integral_switcher);
}
void angle_stab_algorithm2(Quaternion * quaternion, Vector3 * gyro, uint8_t integral_switcher, Vector3 * torque){


	if (quaternion->q0 < 0.0f) {
		quaternion->q0 = -quaternion->q0;
		quaternion->q1 = -quaternion->q1;
		quaternion->q2 = -quaternion->q2;
		quaternion->q3 = -quaternion->q3;
	}
	//gyro derivative computing
	static Vector3 gyro_prev = {0.0f, 0.0f, 0.0f};
	Vector3 gyro_derivate;
	gyro_derivate.x = gyro->x - gyro_prev.x;
	gyro_derivate.y = gyro->y - gyro_prev.y;
	gyro_derivate.z = gyro->z - gyro_prev.z;
	gyro_prev = *gyro;

	#define SEQ_LEN  16
	static float filter_seq[3][SEQ_LEN];
	gyro_derivate.x = moving_avarage_filter(gyro_derivate.x, &filter_seq[0][0], SEQ_LEN);
	gyro_derivate.y = moving_avarage_filter(gyro_derivate.y, &filter_seq[1][0], SEQ_LEN);
	gyro_derivate.z = moving_avarage_filter(gyro_derivate.z, &filter_seq[2][0], SEQ_LEN);



	float d2_gain = (float)(-RadioChannel_getRxChannal(4));
	Vector3 d2_summand;
	torque->x = PIDD2(quaternion->q1, gyro->x, gyro_derivate.x, d2_gain, &PIDs_array[pitch_PID],  integral_switcher, &d2_summand.x);
	torque->y = PIDD2(quaternion->q2, gyro->y, gyro_derivate.y, d2_gain, &PIDs_array[roll_PID],  integral_switcher, &d2_summand.y);
	torque->z = PIDD2(quaternion->q3, gyro->z, gyro_derivate.z, d2_gain, &PIDs_array[yaw_PID],  integral_switcher, &d2_summand.z);
	int16_t tmp[3];
	tmp[0] = (int16_t)d2_summand.x;
	tmp[1] = (int16_t)d2_summand.y;
	tmp[2] = (int16_t)d2_summand.z;
	RadioChannel_setTxChannals(tmp, RESERVED_CHANNAL3, 3);
}

void manual_stab(Quaternion * real_quaternion, Vector3 * gyro, Quaternion * RC_quaternion, Vector3 * RC_gyro, uint16_t thrust, Rotor4 * rotor4_thrust){

	Vector3 torque;
	if(thrust<low_trottle){
		rotor4_thrust->LBC = 0;
		rotor4_thrust->LFW = 0;
		rotor4_thrust->RBW = 0;
		rotor4_thrust->RFC = 0;
		return ;
	}

	Quaternion need_quaternion;
	quaternionMultiplication(RC_quaternion, real_quaternion, &need_quaternion);
	uint8_t integral_switcher = 0;
	if (thrust>integration_trottle && acosf(fabsf(need_quaternion.q0))< M_PI*(25.0f/180.0f)) {
		integral_switcher = 1;
	}

	/*
	 * posible need to rotate RC_gyro from global case to local
	 * */
	need_quaternion.q0 = - need_quaternion.q0;
	rotateVector3ByQuatern(&need_quaternion, RC_gyro);
	need_quaternion.q0 = - need_quaternion.q0;

	Vector3 result_gyro;
	Vector3_sub(gyro, RC_gyro, &result_gyro);
/*
	angle_stab_algorithm(&need_quaternion, &result_gyro, integral_switcher, &torque);
*/
	angle_stab_algorithm2(&need_quaternion, &result_gyro, integral_switcher, &torque);
	calc_rotor4_thrust(torque, thrust, rotor4_thrust);
}



void update_PID_config(uint16_t raw){
	uint8_t pid_num, pid_comp;
	uint16_t pid_val;
	pid_num = (raw >> 13) & 0b111;
	pid_comp = (raw >> 10) & 0b111;
	pid_val = (raw) & 0x3FF;
	configure_PID(&PIDs_array[pid_num], pid_comp, (float)pid_val);
}
void defaultPIDinit(){
	uint8_t i = 0;
	for (i = 0; i < 3; i++) {
		PIDs_array[i].config_array[0] = PID_ANGLE_P_GAIN_DEFAULT;
		PIDs_array[i].config_array[1] = PID_ANGLE_I_GAIN_DEFAULT;
		PIDs_array[i].config_array[2] = PID_ANGLE_D_GAIN_DEFAULT;
		PIDs_array[i].config_array[3] = PID_ANGLE_P_LIMIT_DEFAULT;
		PIDs_array[i].config_array[4] = PID_ANGLE_I_LIMIT_DEFAULT;
		PIDs_array[i].config_array[5] = PID_ANGLE_D_LIMIT_DEFAULT;
		PIDs_array[i].integral_sum = 0.0f;

	}
};
void getAnglesPidSummands(int16_t * array){
	get_PID_summands(&PIDs_array[0], array);
	get_PID_summands(&PIDs_array[1], array+3);
	get_PID_summands(&PIDs_array[2], array+6);
}
void loadPidsTelemetry(){
	int16_t tmp[9];
	getAnglesPidSummands(tmp);
	RadioChannel_setTxChannals(tmp, ANGLES_PIDS, 9);
}
void getRawPidTelemetry(uint8_t *p){
	getAnglesPidSummands(p);
}

