#include "HMC5883L.h"


uint8_t skip_counter = 0;
void HMC_init()
{
    uint8_t tx_buf = 0x70;

    I2C1_WriteBytes(HMC_SERIAL_ADDR, HMC_CFG_A_REG, 1, &tx_buf);

    tx_buf = 0xA0;
    I2C1_WriteBytes(HMC_SERIAL_ADDR, HMC_CFG_B_REG, 1, &tx_buf);

    tx_buf = 0x00;
    I2C1_WriteBytes(HMC_SERIAL_ADDR, HMC_MODE_REG, 1, &tx_buf);

}

void HMC_getRawData(Vector3_int16 *magnet_vector)
{
    uint8_t tmp[6];
    I2C1_ReadBytes(HMC_SERIAL_ADDR, HMC_X_MSB_REG, 6, &tmp);
    magnet_vector->x = (tmp[0] << 8) | tmp[1];
    magnet_vector->y = (tmp[4] << 8) | tmp[5];
    magnet_vector->z = (tmp[2] << 8) | tmp[3];
}

void HMC_getCalibratedData(Vector3_int16 *magnert_vector)
{
	Vector3_int16 magnert_vector_raw;
	HMC_getRawData(&magnert_vector_raw);

	magnert_vector_raw.x = magnert_vector_raw.x - bias_x;
	magnert_vector_raw.y = magnert_vector_raw.y - bias_y;
	magnert_vector_raw.z = magnert_vector_raw.z - bias_z;

	magnert_vector->x = (int16_t)(((int32_t)magnert_vector_raw.x* a11_u) / a11_d) + (((int32_t)magnert_vector_raw.y* a12_u) / a12_d) + (((int32_t)magnert_vector_raw.z* a13_u) / a13_d);
	magnert_vector->y = (int16_t)(((int32_t)magnert_vector_raw.x* a21_u) / a21_d) + (((int32_t)magnert_vector_raw.y* a22_u) / a22_d) + (((int32_t)magnert_vector_raw.z* a23_u) / a23_d);
	magnert_vector->z = (int16_t)(((int32_t)magnert_vector_raw.x* a31_u) / a31_d) + (((int32_t)magnert_vector_raw.y* a32_u) / a12_d) + (((int32_t)magnert_vector_raw.z* a33_u) / a33_d);

}
