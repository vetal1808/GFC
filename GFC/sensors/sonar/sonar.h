#ifndef __SONAR_H__
#define __SONAR_H__
#include "stdint.h"

void Sonar_Init();
void Sonar_Update();
uint16_t Sonar_GetRawDistanse();
void Sonar_GetProcessedData(int32_t * dist, int32_t * dist_velo);
uint8_t Sonar_IsValid();
uint8_t Sonar_IsLastValid();
#endif
