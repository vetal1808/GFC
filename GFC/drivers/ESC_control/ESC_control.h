#ifndef __ESC_CONTROL_H
#define __ESC_CONTROL_H

#include "STM32F10x_TIM.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stdint.h"
void ESC_init();
void ESC_setPower(uint16_t * val);

#endif
