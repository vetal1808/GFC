#include "timer.h"
#include "core_cm3.h"

//#define BUILT_IN_LED_BLINK

#ifdef BUILT_IN_LED_BLINK
	#include "stm32f10x_gpio.h"
#endif
static volatile uint32_t sync_time = 0;
static volatile uint16_t SysTick_OVF_counter = 0;
static volatile uint8_t ovf_event;


void SyncTimer_Init(){
	SysTick->LOAD=0xFFFFFF;
	SysTick->VAL=0xFFFFFF;
	SysTick->CTRL=	SysTick_CTRL_TICKINT_Msk   |
	                SysTick_CTRL_ENABLE_Msk;
}
void SysTick_Handler(){
	SysTick_OVF_counter++;
	ovf_event = 1;
	#ifdef BUILT_IN_LED_BLINK
		//invert built in led. frequency can be changed by shifting mask
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, SysTick_OVF_counter & 0b1);
	#endif
}

void TIMER_initialization()
{
	SyncTimer_Init();
	#ifdef BUILT_IN_LED_BLINK

	#endif
}
uint32_t TIMER_micros()
{
	// fix situation than timer catch overflow while coping volatile data
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
	if((loop_time*2) < (TIMER_micros()-sync_time)){
		//if computing is over two or more times resync
		sync_time = TIMER_micros();
	}
	else{
		while (loop_time > (TIMER_micros()-sync_time));
		sync_time += loop_time;
	}
}
uint32_t TIMER_timeInLoop(){
	uint32_t tmp = TIMER_micros();
	return (tmp-sync_time);
}
uint8_t TIMER_DidOvfHappend(uint8_t reset){
	uint8_t ret = ovf_event;
	if (reset)
		ovf_event = 0;
	return ret;
}
