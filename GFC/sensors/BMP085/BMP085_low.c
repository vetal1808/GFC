/*
 * BMP085.c
 *
 *  Created on: 09.02.2016
 *      Author: vetal
 */
#include "BMP085_low.h"
#include "timer.h"
#include <math.h>

int32_t BMP085_computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

uint8_t BMP085_read8(uint8_t a) {
	uint8_t ret;
	Status _status = I2C1_ReadBytes(BMP085_I2CADDR, a, 1, &ret);

	return ret;
}

uint16_t BMP085_read16(uint8_t a) {

	uint8_t rx_buf[2];
	Status _status = I2C1_ReadBytes(BMP085_I2CADDR, a, 2, rx_buf);
	return (((int16_t)rx_buf[0]) << 8) | rx_buf[1];
}

void BMP085_write8(uint8_t a, uint8_t d) {
	Status _status = I2C1_WriteBytes(BMP085_I2CADDR, a, 1, &d);
}
