#include "helpers.h"

void limit_int16(int16_t * val, int16_t lim){
	if(*val > lim)
		*val =  lim;
	else
		if(*val < -lim)
			*val =  -lim;
}
void limit_float(float * val, float lim){
	if(*val > lim)
		*val =  lim;
	else
		if(*val < -lim)
			*val =  -lim;
}
float limit_float_(float val, float lim){
	if(val > lim)
		return lim;
	if(val < -lim)
		return -lim;
	return val;
}
float moving_avarage_filter(float new_value, float * seq, const uint8_t n)
{
	uint8_t i;
	for (i = n-1; i > 1; i--) {
		seq[i] = seq[i-1];
	}
	seq[0] = new_value;
	float tmp = 0.0f;
	for (i = 0; i < n; i++) {
		tmp+=seq[i];
	}
	tmp/=(float) n;
	return tmp;
}
float FIR_filter_float(float new_value, float * seq, const float * coef, uint8_t n){
	uint8_t i;
	for (i = n-1; i > 1; i--) {
		seq[i] = seq[i-1];
	}
	seq[0] = new_value;
	float tmp = 0;
	for (i = 0; i < n; i++) {
		tmp+=seq[i]*coef[i];
	}
	return tmp;
}
int16_t FIR_filter_int16(int16_t new_value, int16_t * seq, const int16_t * coef, const int16_t final_divider , uint8_t n){
	uint8_t i;
	for (i = n-1; i > 1; i--) {
		seq[i] = seq[i-1];
	}
	seq[0] = new_value;
	int64_t tmp = 0;
	for (i = 0; i < n; i++) {
		tmp+=(int32_t)(seq[i])*(int32_t)(coef[i]);
	}
	tmp/=final_divider;
	return tmp;
}
