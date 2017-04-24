#include "radio_channel.h"

static volatile int16_t rx_channal[RX_CHANNAL_NUM];
static volatile int16_t tx_channal[TX_CHANNAL_NUM];
static uint64_t rx_fresh = 0;
static uint64_t tx_mask = 0;

void RadioChannel_initialization(){
	USART_init(RC_USART, RC_USART_BAUDRATE);
}

void RadioChannel_receiveAllAvailable(){
	if(USART_line_available(RC_USART)){
		uint8_t tmp_buff[PACKAGE_SIZE+2];
		USART_readLine(RC_USART, tmp_buff, PACKAGE_SIZE+2);
		tmp_buff[0] -= 'A';
		tmp_buff[PACKAGE_SIZE+1] = 0;
		if(tmp_buff[0]<(RX_CHANNAL_NUM)){
			rx_fresh |= 1<<tmp_buff[0];
			rx_channal[tmp_buff[0]] = atoi(tmp_buff+1);
		}
	}
}
void RadioChannel_transmitMaskedChannal(){
	uint8_t i = 0;
	for(i=0;i<TX_CHANNAL_NUM;i++){
		if(tx_mask & (1<<i)){
			uint8_t tmp_buff[PACKAGE_SIZE+2];
			itoa((int32_t)(tx_channal[i]),(char *)tmp_buff+1,10);
			uint8_t j;
			for(j=2 ; j < PACKAGE_SIZE+2; j++)
				if(tmp_buff[j] == 0){
					tmp_buff[j] = '\n';
					break;
				}
			j++;
			tmp_buff[0] = 'A' + i;
			USART_send(RC_USART,tmp_buff,j);
		}
	}
	USART_send(RC_USART,"z0\n",3);
}
void RadioChannel_setTxMask(uint64_t val){
	tx_mask = val;
}
void RadioChannel_setTxChannals(int16_t * p, uint8_t st_pos, uint8_t num){
	uint8_t i = 0;
	for(i = 0; (i < num) && (st_pos < RX_CHANNAL_NUM);i++)
	{
		tx_channal[st_pos] = p[i];
		st_pos++;
	}
}
void RadioChannel_setTxChannal(int16_t val, uint8_t pos){
	if (pos < TX_CHANNAL_NUM)
		tx_channal[pos] = val;
}
void RadioChannel_getRxChannals(int16_t * p, uint8_t st_pos, uint8_t num){
	uint8_t i = 0;
	for(i = 0; (i < num) && (st_pos < RX_CHANNAL_NUM);i++)
	{
		p[i] = rx_channal[st_pos];
		st_pos++;
	}
}
int16_t RadioChannel_getRxChannal(uint8_t pos){
	return rx_channal[pos];
}
uint8_t RadioChannel_isRxChannalFresh(uint8_t pos){
	uint8_t return_ = 0;
	if(rx_fresh & (1 << pos))
		return_ = 1;
	rx_fresh = ((rx_fresh) & ~(1 << pos));
	return return_;
}

