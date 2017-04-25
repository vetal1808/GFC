#include "timer.h"
#include "core_cm3.h"

volatile uint32_t sync_time = 0;
volatile uint32_t SysTick_OVF_counter = 0;

void SyncTimer_Init(){
	SysTick->LOAD=0xFFFFFF;
	SysTick->VAL=0xFFFFFF;
	SysTick->CTRL=	SysTick_CTRL_TICKINT_Msk   |
	                SysTick_CTRL_ENABLE_Msk;
}
void SysTick_Handler(){
	SysTick_OVF_counter++;
}

void TIMER_initialization()
{
	SyncTimer_Init();
}
uint32_t TIMER_micros()
{
	// Glitch free clock
	uint32_t v0 = SysTick->VAL;
	uint32_t c0 = SysTick_OVF_counter;
	uint32_t v1 = SysTick->VAL;
	uint32_t c1 = SysTick_OVF_counter;

	if (v1 < v0)
		// Downcounting, no systick rollover
		return (c0<<24 | (0xFFFFFF - v0))/9;
	else
		// systick rollover, use last count value
		return (c1<<24 | (0xFFFFFF - v1))/9;

}
void TIMER_delayUs(uint32_t us)
{
	uint32_t start = TIMER_micros();
	while((TIMER_micros()-start)<us);
}
void TIMER_startSynchronizationLoop(){
	sync_time = TIMER_micros();
}
void TIMER_waitEndOfLoop(uint32_t loop_time){
	if((loop_time*2) > (TIMER_micros()-sync_time)){
		//if computing is over two or more times resync
		sync_time = TIMER_micros();
	}
	else{
		while (loop_time > (TIMER_micros()-sync_time));
		sync_time += loop_time;
	}
}
uint32_t TIMER_timeInLoop(){
	return (TIMER_micros()-sync_time);
}
