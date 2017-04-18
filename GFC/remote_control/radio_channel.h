#ifndef __RADIO_CHANNEL_H__
#define __RADIO_CHANNEL_H__

#include "easy_uart.h"
#include "stdlib.h"

#define RX_CHANNAL_NUM 32
#define TX_CHANNAL_NUM 32
#define PACKAGE_SIZE 6

#define RC_USART USART3
#define RC_USART_BAUDRATE 921600

void RadioChannel_initialization();
void RadioChannel_receiveAllAvailable();
void RadioChannel_transmitMaskedChannal();
void RadioChannel_setTxMask(uint64_t val);
uint8_t RadioChannel_isRxChannalFresh(uint8_t pos);
void RadioChannel_setTxChannal(int16_t val, uint8_t pos);
void RadioChannel_setTxChannals(int16_t * p, uint8_t st_pos, uint8_t num);
int16_t RadioChannel_getRxChannal(uint8_t pos);
void RadioChannel_getRxChannals(int16_t * p, uint8_t st_pos, uint8_t num);

#endif
