#ifndef __AB_FILTER_H__
#define __AB_FILTER_H__
#include "stdint.h"

typedef struct {
	float K, K1;
	float prev;
	float period;
} AlfaBetaFilterFloat_Struct;

void AlfaBetaFilterFloat_SetupStruct(AlfaBetaFilterFloat_Struct * filter, float _k, float period);
float AlfaBetaFilerFloat_Update(AlfaBetaFilterFloat_Struct * filter, float derivative, float value);
void AlfaBetaFilterFloat_SetPrevious(AlfaBetaFilterFloat_Struct * filter, float previous);


typedef struct {
	int32_t K, K1;
	int32_t prev;
	int32_t freq;
} AlfaBetaFilterInt_Struct;
void AlfaBetaFilterInt_SetupStruct(AlfaBetaFilterInt_Struct * filter, int32_t _k, int32_t _freq);
int32_t AlfaBetaFilerInt_Update(AlfaBetaFilterInt_Struct * filter, int32_t derivative, int32_t value);
void AlfaBetaFilterInt_SetPrevious(AlfaBetaFilterInt_Struct * filter, int32_t previous);
#endif
