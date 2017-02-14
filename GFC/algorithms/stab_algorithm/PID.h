#ifndef __PID_H__
#define __PID_H__

#include <stdlib.h>
#include <math.h>

#define P_gain 0
#define I_gain 1
#define D_gain 2

#define P_limit 3
#define I_limit 4
#define D_limit 5


typedef struct{
	float config_array[6];
	float PID_summand [3];
	float integral_sum;;
} PID_struct;

float PID(float error, float d_error, PID_struct * PID_config, 
			uint8_t integral_switcher);
float PIDD2(float error, float d_error, float d2_error, float d2_gain, PID_struct * PID_state, uint8_t integral_switcher, float * return_d2);
void configure_PID(PID_struct * PID, uint8_t pos, float val);
void get_PID_summands(PID_struct * PID_state, int16_t * array);
#endif
