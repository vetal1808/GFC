#include "FIR_filter.h"
#include "FIR_filter_config.h"
int32_t FIR_filter_int32(int32_t new_value, FIR_filter_int32_struct * filter) {
	uint8_t i;
	for (i = filter->len - 1; i > 0; i--) {
		filter->seq[i] = filter->seq[i - 1];
	}
	filter->seq[0] = new_value;
	int64_t tmp = 0;
	for (i = 0; i < filter->len; i++) {
		tmp += (int64_t)(filter->seq[i])*(int64_t)(filter->coef[i]);
	}
	tmp /= filter->final_divider;
	return tmp;
}
void FIR_filter_int32_configue2(FIR_filter_int32_struct * filter_struct,
		const int32_t * _coef, int32_t * _seq, uint8_t len, int32_t _final_divider){
	filter_struct->coef = _coef;
	filter_struct->seq = _seq;
	filter_struct->len = len;
	filter_struct->final_divider = _final_divider;
}
void FIR_filter_int32_configue(FIR_filter_int32_struct * filter_struct, int32_t * _seq, uint8_t asset){
	//this is temporary solution for one asset of configures
	filter_struct->coef = FIR_coef;
	filter_struct->seq = _seq;
	filter_struct->len = len;
	filter_struct->final_divider = final_divider;
}
float FIR_filter_float(float new_value, FIR_filter_float_struct * filter) {
	uint8_t i;
	for (i = filter->len - 1; i > 0; i--) {
		filter->seq[i] = filter->seq[i - 1];
	}
	filter->seq[0] = new_value;
	float tmp = 0;
	for (i = 0; i < filter->len; i++) {
		tmp += (filter->seq[i])*(filter->coef[i]);
	}
	return tmp;
}
void FIR_filter_float_configue2(FIR_filter_float_struct * filter_struct,
		const float * _coef, float * _seq, uint8_t len){
	filter_struct->coef = _coef;
	filter_struct->seq = _seq;
	filter_struct->len = len;
}
