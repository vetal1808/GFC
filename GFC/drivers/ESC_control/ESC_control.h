#ifndef __ESC_CONTROL_H
#define __ESC_CONTROL_H

#include "STM32F10x_TIM.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "types.h"
void ESC_init();
void ESC_set_power(uint16_t * val);
void update_rotors(Rotor4 * thrust4, uint8_t rotor_mask);

#endif