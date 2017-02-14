#ifndef RADIO_CHANNAL_H_
#define RADIO_CHANNAL_H_

#include "easy_uart.h"
#include "stdlib.h"

#define rx_channal_num 16
#define tx_channal_num 20
#define package_size 6

void set_USARTn(USART_TypeDef* USARTx);
void receive_all_available();
void transmit_masked_channal();
void set_tx_mask(uint32_t val);
uint8_t is_rx_channal_fresh(uint8_t pos);
void set_tx_channals(int16_t * p, uint8_t st_pos, uint8_t num);
void set_tx_channal(int16_t val, uint8_t pos);
void get_rx_channals(int16_t * p, uint8_t st_pos, uint8_t num);
int16_t get_rx_channal(uint8_t pos);

#endif
