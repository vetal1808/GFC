#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

void TIMER_initialization();
uint32_t TIMER_micros();
void TIMER_delayUs(uint32_t us);
void TIMER_startSynchronizationLoop();
void TIMER_waitEndOfLoop(uint32_t loop_time);
uint32_t TIMER_timeInLoop();
#endif
