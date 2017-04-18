#include "q_config.h"
#include "helpers.h"
#include "PID.h"
#define loop_time UPDATE_PERIOD_IN_SEC


float dynamic_d2_gain(float angle, float angle_velo);
float PID(float error, float d_error, PID_struct * PID_state, uint8_t integral_switcher){

	if(integral_switcher)
		PID_state->integral_sum += error*loop_time;

	PID_state->integral_sum = limit_float_(PID_state->integral_sum, PID_state->config_array[I_limit]/PID_state->config_array[I_gain]);
	PID_state->PID_summand[0] = limit_float_(error*PID_state->config_array[P_gain], PID_state->config_array[P_limit]);
	PID_state->PID_summand[1] = PID_state->integral_sum*PID_state->config_array[I_gain];
	PID_state->PID_summand[2] = limit_float_(d_error*PID_state->config_array[D_gain], PID_state->config_array[D_limit]);

	return PID_state->PID_summand[0] + PID_state->PID_summand[1] + PID_state->PID_summand[2];
}
float PIDD2(float error, float d_error, float d2_error, float d2_gain, PID_struct * PID_state, uint8_t integral_switcher, float * return_d2){
	float tmp = PID(error, d_error, PID_state, integral_switcher);
	d2_gain*=dynamic_d2_gain(error, d_error);
	*return_d2 = d2_error*d2_gain;
	return tmp + *return_d2;
}
void configure_PID(PID_struct * PID, uint8_t pos, float val){
	if (pos < 6) {
		PID->config_array[pos] = val;
	}
}
void get_PID_summands(PID_struct * PID_state, int16_t * array){
	array[0] = (int16_t) (PID_state->PID_summand[0]);
	array[1] = (int16_t) (PID_state->PID_summand[1]);
	array[2] = (int16_t) (PID_state->PID_summand[2]);
}
float dynamic_d2_gain(float angle, float angle_velo){
	const float angle_coef = 1.0/((3.14/180.0)*5.0);
	const float angle_velo_coeff = 1.0/((3.14/180.0)*60.0);
	angle*=angle_coef;
	angle_velo*=angle_velo_coeff;
	float tmp = angle*angle+angle_velo*angle_velo;
	if(tmp>2.0){
		return 1.0/tmp;
	}
	else {
		return 0.5;
	}

}
