#include "timer.h"

volatile uint16_t timer_overflow = 0;
static uint32_t sync_time = 0;

void TIM4_IRQHandler(void)
{
    TIM4->SR &= ~TIM_SR_UIF;
    timer_overflow++;
}
void TIMER_initialization()
{
	  NVIC_SetPriority(TIM4_IRQn, 1);
	  NVIC_EnableIRQ(TIM4_IRQn);
	  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	  TIM4->PSC = 71;
	  TIM4->ARR = 0xFFFF;
	  TIM4->DIER |= TIM_DIER_UIE;
	  TIM4->CR1 |= TIM_CR1_CEN;
}
uint32_t TIMER_micros()
{
	NVIC_DisableIRQ(TIM4_IRQn);
	uint16_t count = TIM4->CNT;
	uint16_t ovf = timer_overflow;

	if(TIM4->SR & TIM_SR_UIF)//overflow happened just now
	{
		ovf++;
		count = 0;
	}
	NVIC_EnableIRQ(TIM4_IRQn);
	return (ovf<<16) | (count);
}
void TIMER_delayUs(uint32_t us)
{
	uint32_t start = TIMER_micros();
	while(TIMER_micros()-start<us);
}
void TIMER_startSynchronizationLoop(){
	sync_time = TIMER_micros();
}
void TIMER_waitEndOfLoop(uint32_t loop_time){
	while (loop_time > (TIMER_micros()-sync_time));
	sync_time += loop_time;
}
uint32_t TIMER_timeInLoop(){
	return (TIMER_micros()-sync_time);
}
