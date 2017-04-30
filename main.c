#include "stm32f10x.h"
#include "math.h"
#include "timer.h"
#include "telemetry.h"
#include "I2CRoutines.h"
#include "algorithms/orientation/orientation.h"
#include "algorithms/altitude_algorithm/altitude_algorithm.h"
#include "algorithms/motor_algorithm/motor_algorithm.h"
#include "q_config.h"
#include "radio_control.h"
#include "stab_algorithm.h"
#include "types.h"
#include "blinker.h"
#include "ADC.h"
void setup();

int main(void)
{
	setup();
    while(1)
    {

    	//update radio data

    	RC_update();
    	//update flight state

    	//apply new algorithms settings

    	//computing orientation
    	Orientation_Update();

    	Vector3 local_gyro;
    	Orientation_getLocalGyro(&local_gyro);
    	Vector3 global_accel;
    	Orientation_getGlobalAccel(&global_accel);
    	Quaternion Global_quaternion;
    	Orientation_getQuaternion(&Global_quaternion);
    	//computing geo-position
    	Altitude_AlgorithmUpdate(global_accel.z);

    	float altitude, altitude_velocity, altitude_acceleration;
    	Altitude_GetVerticalState(&altitude, &altitude_velocity, &altitude_acceleration);
    	RadioChannel_setTxChannal((int16_t)altitude, 3);
    	RadioChannel_setTxChannal((int16_t)altitude_velocity, 4);
    	RadioChannel_setTxChannal((int16_t)altitude_acceleration, 5);
    	Altitude_GetVerticalState1(&altitude, &altitude_velocity, &altitude_acceleration);
    	RadioChannel_setTxChannal((int16_t)altitude, 6);
    	RadioChannel_setTxChannal((int16_t)altitude_velocity, 7);
    	RadioChannel_setTxChannal((int16_t)altitude_acceleration, 8);
    	RadioChannel_setTxChannal((int16_t)TIMER_timeInLoop(), LOOP_TIME);
    	RadioChannel_setTxChannal((int16_t)Voltmeter_GetData(), BATTERY_VOLTAGE);
    	Telemetry_sendToPilot();
    	//computing main algorithm

    	//setup motors trust

    	//send telemetry

    	//waiting end of cycle
    	Blinker_Update();
    	Voltmeter_Update();
    	TIMER_waitEndOfLoop(UPDATE_PERIOD_IN_US);

    }

}
void setup(){
	TIMER_initialization();
	RadioChannel_initialization();

	I2C_LowLevel_Init(I2C1);
	Orientation_InitSensors(COMPASS_DO_NOT_USE);
	Altitude_Init();
	MOTORS_InitESC();
	TIMER_startSynchronizationLoop();
	RadioChannel_setTxMask(0b1000111111000);
	Voltmeter_Init();
	Blinker_Init(UPDATE_FRQ/4);
}
