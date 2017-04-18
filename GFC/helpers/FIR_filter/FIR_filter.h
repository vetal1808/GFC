#ifndef __FIR_FILTER_H__
#define __FIR_FILTER_H__

#include "stdint.h"





typedef struct {
	uint8_t len;
	const int32_t * coef;
	int32_t * seq;
	int32_t final_divider;
} FIR_filter_int32_struct;
int32_t FIR_filter_int32(int32_t new_value, FIR_filter_int32_struct * filter);
void FIR_filter_int32_configue(FIR_filter_int32_struct * , int32_t * _seq, uint8_t asset);
void FIR_filter_int32_configue2(FIR_filter_int32_struct * filter_struct, const int32_t * _coef, int32_t * _seq, uint8_t len, int32_t _final_divider);
typedef struct {
	uint8_t len;
	const float * coef;
	float * seq;
} FIR_filter_float_struct;
float FIR_filter_float(float new_value, FIR_filter_float_struct * filter);
void FIR_filter_float_configue(FIR_filter_float_struct * filter_struct, float * _seq, uint8_t asset);
void FIR_filter_float_configue2(FIR_filter_float_struct * filter_struct, const float * _coef, float * _seq, uint8_t len);
#endif
