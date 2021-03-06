#include "AB_filter.h"

void AlfaBetaFilterFloat_SetupStruct(AlfaBetaFilterFloat_Struct * filter, float _k, float _period){
	filter->prev = 0.0;
	filter->K = _k;
	filter->K1 = 1.0-_k;
	filter->period = _period;
}
float AlfaBetaFilerFloat_Update(AlfaBetaFilterFloat_Struct * filter, float derivative, float value){
	filter->prev = (derivative*filter->period + filter->prev)*filter->K + (value * filter->K1);
	return filter->prev;
}
void AlfaBetaFilterFloat_SetPrevious(AlfaBetaFilterFloat_Struct * filter, float previous){
	filter->prev = previous;
}
void AlfaBetaFilterInt_SetupStruct(AlfaBetaFilterInt_Struct * filter, int32_t _k, int32_t _freq){
	filter->prev = 0;
	filter->K = _k;
	filter->K1 = 1024-_k;
	filter->freq = _freq;
}
int32_t AlfaBetaFilerInt_Update(AlfaBetaFilterInt_Struct * filter, int32_t derivative, int32_t value){
	filter->prev = (derivative/filter->freq + filter->prev)*filter->K/1024 + (value * filter->K1 / 1024);
	return filter->prev;
}
void AlfaBetaFilterInt_SetPrevious(AlfaBetaFilterInt_Struct * filter, int32_t previous){
	filter->prev = previous;
}
