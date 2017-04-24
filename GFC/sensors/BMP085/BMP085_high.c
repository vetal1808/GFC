#include "BMP085_high.h"
#include "BMP085_low.h"
#include <math.h>
#include "timer.h"

#define update_frq 40
#define POSTPOCESSING

float BMP_zero_press = 100000;

uint32_t BMP_raw_pressure;
uint16_t BMP_raw_temperature;
int32_t BMP_raw_altitude;
int32_t BMP_altitude, BMP_velocity;

#ifdef POSTPOCESSING

#include "FIR_filter.h"
FIR_filter_int32_struct FIR_altitude;
FIR_filter_int32_struct FIR_alt_velocity;

void BMP085_postprocessing();
void BMP085_postprocessingInit();

#endif

uint8_t BMP085_begin(uint8_t mode) {

	if (mode > BMP085_ULTRAHIGHRES)
	mode = BMP085_ULTRAHIGHRES;
	oversampling = mode;

	if (BMP085_read8(0xD0) != 0x55)
	  return 0;

	/* read calibration data */
	ac1 = BMP085_read16(BMP085_CAL_AC1);
	ac2 = BMP085_read16(BMP085_CAL_AC2);
	ac3 = BMP085_read16(BMP085_CAL_AC3);
	ac4 = BMP085_read16(BMP085_CAL_AC4);
	ac5 = BMP085_read16(BMP085_CAL_AC5);
	ac6 = BMP085_read16(BMP085_CAL_AC6);

	b1 = BMP085_read16(BMP085_CAL_B1);
	b2 = BMP085_read16(BMP085_CAL_B2);

	mb = BMP085_read16(BMP085_CAL_MB);
	mc = BMP085_read16(BMP085_CAL_MC);
	md = BMP085_read16(BMP085_CAL_MD);

#ifdef POSTPOCESSING
	BMP085_postprocessingInit();
#endif

	return 255;
}

void BMP085_readRawTemperature_reqest(){
	BMP085_write8(BMP085_CONTROL, BMP085_READTEMPCMD);
}

uint16_t BMP085_readRawTemperature_ask(){
	return BMP085_read16(BMP085_TEMPDATA);
}

void BMP085_readRawPressure_reqest(){
	BMP085_write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
}

uint32_t BMP085_readRawPressure_ask(){
	uint16_t tmp[2];

	tmp[0] = BMP085_read16(BMP085_PRESSUREDATA);
	tmp[1] = BMP085_read8(BMP085_PRESSUREDATA+2);

	return ((tmp[0]<<8 | tmp[1]) >> (8-oversampling));
}

int32_t BMP085_calculatePressure(uint16_t UT, uint32_t UP){
	int32_t B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;

	  B5 = BMP085_computeB5(UT);

	  // do pressure calcs
	  B6 = B5 - 4000;
	  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
	  X2 = ((int32_t)ac2 * B6) >> 11;
	  X3 = X1 + X2;
	  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

	  X1 = ((int32_t)ac3 * B6) >> 13;
	  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
	  X3 = ((X1 + X2) + 2) >> 2;
	  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
	  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

	  if (B7 < 0x80000000) {
	    p = (B7 * 2) / B4;
	  } else {
	    p = (B7 / B4) * 2;
	  }
	  X1 = (p >> 8) * (p >> 8);
	  X1 = (X1 * 3038) >> 16;
	  X2 = (-7357 * p) >> 16;

	  p = p + ((X1 + X2 + (int32_t)3791)>>4);
	  //return pressure in Pa
	  return p;
}
int16_t BMP085_getTemperature(uint16_t UT) {
	int32_t B5 = BMP085_computeB5(UT);
	return (B5 + 8) >> 4;
}
float BMP085_calculateAltitude(float pressure){
	//return altitude in millimeters
	return 44330000.0 * (1.0 - pow(pressure /BMP_zero_press,0.1903));
}


void BMP085_update(){
    const uint8_t temp_meas_skip = 4;
	static uint8_t BMP_state = 0, temp_meas_skip_counter = 0;
	static uint32_t sync_time;
	static uint16_t UT; // uncompensated temperature value
	static uint32_t UP;	// uncompensated pressure value

	switch (BMP_state){
		case 0 :
        {
			BMP085_readRawTemperature_reqest();
			sync_time = TIMER_micros();
			BMP_state++;
			break;
		}
		case 1 :
        {
			if(TIMER_micros() - sync_time > 4499){
				UT = BMP085_readRawTemperature_ask();
				BMP085_readRawPressure_reqest();
				BMP_raw_temperature = BMP085_getTemperature(UT);
				BMP_state++;
				sync_time = TIMER_micros();
			}
			break;
		}
		case 2 :
        {
			if(TIMER_micros() - sync_time > 25499){
				UP = BMP085_readRawPressure_ask();
				BMP_raw_pressure = BMP085_calculatePressure(UT,UP);
                BMP_raw_altitude = (int32_t) BMP085_calculateAltitude((float)BMP_raw_pressure);
                #ifdef POSTPOCESSING
                BMP085_postprocessing();
                #endif 
				if(temp_meas_skip_counter < temp_meas_skip){ // skip temperature reading 5 times
					BMP085_readRawPressure_reqest();
					sync_time = TIMER_micros();
					temp_meas_skip_counter++;
				}
				else {
                    BMP085_readRawTemperature_reqest();
                    sync_time = TIMER_micros();
					BMP_state = 1;
					temp_meas_skip_counter = 0;
				}
			}
			break;

		}
		default :{
			BMP_state = 0;
			break;
		}
	}
}

uint32_t BMP085_measurePressure(){
	uint16_t BMP_tmp1;
	uint32_t BMP_tmp2;
	BMP085_readRawTemperature_reqest();
	TIMER_delayUs(4500);
	BMP_tmp1 = BMP085_readRawTemperature_ask();
	BMP085_readRawPressure_reqest();
	TIMER_delayUs(24500);
	BMP_tmp2 = BMP085_readRawPressure_ask();
	return BMP085_calculatePressure(BMP_tmp1,BMP_tmp2);
}
void BMP085_setZeroPressure(uint32_t pressure){
	BMP_zero_press = pressure;
}
/*
 * set zero pressure using current pressure and current altitude from other source
 */
void BMP085_setZeroPressure2(uint32_t current_altitude){
	//this algorithm work work approximately
	const float press_to_dist = 0.01187f; // pascal per millimeter
	int16_t pressure_offset = (int16_t)(press_to_dist*(float)current_altitude);
	BMP_zero_press = BMP_raw_pressure + pressure_offset;
}
void BMP085_setZeroPressure3(uint32_t current_pressuer, uint32_t current_altitude){
	//this algorithm work work approximately
	const float press_to_dist = 0.01187f; // pascal per millimeter
	int16_t pressure_offset = (int16_t)(press_to_dist*(float)current_altitude);
	BMP_zero_press = current_pressuer + pressure_offset;
}

void BMP085_getProcessesData(int32_t *altitude, int32_t *velocity){
	*altitude = BMP_altitude;
	*velocity = BMP_velocity;
}
int32_t BMP085_getRawAltitude(){
	return BMP_raw_altitude;
}
#ifdef POSTPOCESSING
void BMP085_postprocessing(){
	int32_t diff = (BMP_raw_altitude - (FIR_altitude.seq[0]))*update_frq;
	BMP_altitude = FIR_filter_int32(BMP_raw_altitude, &FIR_altitude);
	BMP_velocity = FIR_filter_int32(diff, &FIR_alt_velocity);
}
void BMP085_postprocessingInit(){
	static int32_t altitude_seq[32];
	static int32_t velocity_seq[32];
	FIR_filter_int32_configue(&FIR_altitude, altitude_seq, 0);
	FIR_filter_int32_configue(&FIR_alt_velocity, velocity_seq, 0);
}
#endif
