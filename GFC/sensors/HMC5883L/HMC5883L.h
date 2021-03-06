

#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include <stdint.h>
#include "I2CRoutines.h"
#include "types.h"

#define HMC_SERIAL_ADDR 0x1E

#define HMC_CFG_A_REG  0x00
#define HMC_CFG_B_REG  0x01
#define HMC_MODE_REG   0x02
#define HMC_X_MSB_REG  0x03
#define HMC_X_LSB_REG  0x04
#define HMC_Z_MSB_REG  0x05
#define HMC_Z_LSB_REG  0x06
#define HMC_Y_MSB_REG  0x07
#define HMC_Y_LSB_REG  0x08
#define HMC_STATUS_REG 0x09
#define HMC_IDNT_A_REG 0x0A
#define HMC_IDNT_B_REG 0x0B
#define HMC_IDNT_C_REG 0x0C


#define MEASURE_CONTIN 0x00
#define MEASURE_SINGLE 0x01
#define MEASURE_IDLE   0x03

//calibrion defines

/*
	bias = {-108.979064 ; 27.997480 ; 43.288363}

	A = {{  31.575157 ; -0.069780 ; 0.171385};
	{ -0.069780 ; 30.976586; -0.014672};
	{ 0.171385; -0.014672; 32.284786}}
 */

#define bias_x -109
#define bias_y 28
#define bias_z 43

#define a11_u 505
#define a12_u -9
#define a13_u 11
#define a21_u -9
#define a22_u 1332
#define a23_u -15
#define a31_u 11
#define a32_u -15
#define a33_u 226

#define a11_d 16
#define a12_d 128
#define a13_d 64
#define a21_d 128
#define a22_d 43
#define a23_d 1024
#define a31_d 64
#define a32_d 1024
#define a33_d 7


#define skip_val 20


void HMC_init();

void HMC_getRawData(Vector3_int16 *magnet_vector);
void HMC_getCalibratedData(Vector3_int16 *magnert_vector);


#endif /* HMC_H_ */
