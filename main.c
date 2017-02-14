#include "stm32f10x.h"
#include "math.h"
#include "timer.h"
#include "telemetry.h"
#include "radio_channal.h"
#include "mpu6050.h"
#include "MadgwickAHRS.h"
#include "q_config.h"
#include "radio_control.h"
#include "stab_algorithm.h"
#include "drivers/ESC_control/ESC_control.h"

void setup();


Vector3 MPU6050_accel, MPU6050_gyro;
Quaternion Global_quaternion; //quaternion rotation relative to the horizon

Quaternion RC_plane_quaternion;
Quaternion RC_quaternion;
Vector3 RC_spin;
int16_t thrust;
Rotor4 rotor4_thrust;
int8_t last_connect;
uint32_t s_t = 0;

int main(void)
{
	setup();

    while(1)
    {

    	RC_update();

    	MPU6050_getFloatMotion6(&MPU6050_accel, &MPU6050_gyro);

    	MadgwickAHRSupdateIMU(MPU6050_gyro.x, MPU6050_gyro.y, MPU6050_gyro.z,
    			MPU6050_accel.x, MPU6050_accel.y, MPU6050_accel.z);

    	Quaternion Global_quaternion = GetOffsetMadgwickAHRSQuaternion();

    	get_RC_state(&RC_quaternion, &RC_spin, &thrust, &last_connect);

    	quaternionMultiplication(&RC_quaternion, &Global_quaternion, &RC_plane_quaternion);

    	manual_stab(&Global_quaternion, &MPU6050_gyro, &RC_quaternion, &RC_spin, thrust, &rotor4_thrust);

    	load_axis_errors(&RC_plane_quaternion);
    	load_euclid_angles_derivative(&Global_quaternion, MPU6050_gyro);
    	load_euclid_angles(&RC_plane_quaternion);
    	loadPidsTelemetry();
    	uint32_t lol = micros()-s_t;
    	set_tx_channal((int16_t)lol, LOOP_TIME);
    	telemetry_update();

    	while((micros()-s_t) < update_period_in_us);
    	s_t+=update_period_in_us;
    	//synchronous_delay(update_period_in_us);

    	update_rotors(&rotor4_thrust, (uint8_t)get_rx_channal(MOTOR_MASK));

    }

}
void setup(){
	init_timer();
	set_USARTn(USART3);

	I2C_LowLevel_Init(I2C1);
	MPU6050_initialize();
	MPU6050_setDLPFMode(0x06);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	delay_us(1000000);
	MPU6050_calibration(3000); //accumulation gyroscope offset
	MPU6050_setSampleRateDiv(3);

	set_tx_mask(0b111111);
	ESC_init();
	defaultPIDinit();
	s_t = micros();
	//start_synchronization();

}
