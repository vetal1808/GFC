#include "motor_algorithm.h"
#include "ESC_control.h"

uint8_t motor_mask = 0;
void MOTORS_InitESC(){
	ESC_init();
}
void MOTORS_SetMask(uint8_t mask){
	motor_mask = mask;
}
void MOTORS_UpdateThrust(Vector3 * torque_of_axis_float, int16_t average_thrust){
	Vector3_int16 torque_of_axis_int;
	torque_of_axis_int.x = torque_of_axis_float->x;
	torque_of_axis_int.y = torque_of_axis_float->y;
	torque_of_axis_int.z = torque_of_axis_float->z;
	Rotor4 rotors4_thrust = {0, 0, 0, 0};
	if(!(motor_mask & (1<<0)))
		rotors4_thrust.LFW = (uint16_t)(average_thrust - ((torque_of_axis_int.x + torque_of_axis_int.y + torque_of_axis_int.z)));
	if(!(motor_mask & (1<<1)))
		rotors4_thrust.RFC = (uint16_t)(average_thrust - ((torque_of_axis_int.x - torque_of_axis_int.y - torque_of_axis_int.z)));
	if(!(motor_mask & (1<<2)))
		rotors4_thrust.LBC = (uint16_t)(average_thrust - ((-torque_of_axis_int.x + torque_of_axis_int.y - torque_of_axis_int.z)));
	if(!(motor_mask & (1<<3)))
		rotors4_thrust.RBW = (uint16_t)(average_thrust - ((-torque_of_axis_int.x - torque_of_axis_int.y + torque_of_axis_int.z)));
	uint16_t tmp[4] = {rotors4_thrust.LFW, rotors4_thrust.RFC, rotors4_thrust.RBW, rotors4_thrust.LBC};
	ESC_setPower(tmp);
}
