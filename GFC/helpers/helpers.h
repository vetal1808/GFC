#ifndef HELPERS_H_
#define HELPERS_H_

#include "types.h"

void limit_int16(int16_t * val, int16_t lim);
void limit_float(float * val, float lim);
float limit_float_(float val, float lim);
float moving_avarage_filter(float new_value, float * seq, const uint8_t n);
float FIR_filter_float(float new_value, float * seq, const float * coef, uint8_t n);
int16_t FIR_filter_int16(int16_t new_value, int16_t * seq, const int16_t * coef, const int16_t final_divider , uint8_t n);
#endif
