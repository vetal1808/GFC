#include "sonar.h"
#include "misc.h"

/*
 * A9 - echo
 * A10 - trig
 * */

#include "timer.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"

#define ECHO_PIN GPIO_Pin_11
#define TRIG_PIN GPIO_Pin_12

#define RESCAN_PERIOD_US 50000
#define RESCAN_FRQ 1000000/RESCAN_PERIOD_US
#define POSTPOCESSING

#ifdef POSTPOCESSING
	#include "FIR_filter.h"
	FIR_filter_int32_struct FIR_disatanse_velocity;

	void Sonar_Postprocessing();
	void Sonar_Postprocessing_init();
#endif

volatile uint32_t echo_delay = 0;
volatile uint8_t new_ready = 0;
uint32_t sync_time_ = 0;
int32_t sonar_distanse;
int32_t sonar_velocity;


void init_gpio();
void init_external_interupt();
void Sonar_StartMeasuring();

void Sonar_Init(){
	init_gpio();
	init_external_interupt();
#ifdef POSTPOCESSING
	Sonar_Postprocessing_init();
#endif
}

void init_gpio(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	GPIO_InitTypeDef PORT;

	PORT.GPIO_Pin = ECHO_PIN;
	PORT.GPIO_Mode = GPIO_Mode_IPD;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &PORT);

	PORT.GPIO_Pin = TRIG_PIN;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &PORT);
}


void init_external_interupt(){

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);


	AFIO->EXTICR[0]|=AFIO_EXTICR1_EXTI1_PA;
	EXTI_InitTypeDef  EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line11;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & ECHO_PIN)
	{
		new_ready = 1;
		echo_delay = TIMER_micros() - sync_time_;
		EXTI->PR = ECHO_PIN; // reset interrupt
	}
}

void Sonar_StartMeasuring(){
	GPIO_SetBits(GPIOA, TRIG_PIN);
	TIMER_delayUs(11);
	GPIO_ResetBits(GPIOA, TRIG_PIN);
}

void Sonar_Update(){
	if ((TIMER_micros() - sync_time_)>=RESCAN_PERIOD_US) {
		Sonar_StartMeasuring();
		sync_time_ = TIMER_micros();
	}
#ifdef POSTPOCESSING
	Sonar_Postprocessing();
#endif
}
uint16_t Sonar_GetRawDistanse(){
	//return distanse in millimeters
	return (echo_delay*174)/1024;
}
void Sonar_GetProcessedData(int32_t * dist, int32_t * dist_velo){
	*dist =  sonar_distanse;
	*dist_velo =  sonar_velocity;
}
#ifdef POSTPOCESSING
	void Sonar_Postprocessing(){
		if (new_ready == 1) {
			new_ready = 0;
			int16_t sonar_dist_next = Sonar_GetRawDistanse();
			int32_t diff = (sonar_dist_next - sonar_distanse)*RESCAN_FRQ;
			sonar_distanse = sonar_dist_next;
			sonar_velocity = diff;
			//sonar_velocity = FIR_filter_int32(diff, &FIR_disatanse_velocity);
		}
	}
	void Sonar_Postprocessing_init(){
		static int32_t disatanse_velo_seq[32];
		FIR_filter_int32_configue(&FIR_disatanse_velocity, disatanse_velo_seq, 1);
	}
#endif
