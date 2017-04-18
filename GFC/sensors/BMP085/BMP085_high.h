#ifndef __BMP085_HIGH_H__
#define __BMP085_HIGH_H__

#include <stdint.h>

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3

uint8_t BMP085_begin(uint8_t mode);  // by default go highres

void BMP085_readRawTemperature_reqest();
uint16_t BMP085_readRawTemperature_ask();
void BMP085_readRawPressure_reqest();
uint32_t BMP085_readRawPressure_ask();

int32_t BMP085_calculatePressure(uint16_t UT, uint32_t UP);
float BMP085_calculateAltitude(float pressure) ;

uint32_t BMP085_measurePressure();
void BMP085_setZeroPressure(uint32_t pressure);
void BMP085_setZeroPressure2(uint32_t current_pressure, uint32_t altitude);
void BMP085_update();

void BMP085_getProcessesData(int32_t * altitude, int32_t * velocity);
int32_t BMP085_getRawAltitude();
#endif
