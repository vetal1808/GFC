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
