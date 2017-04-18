#include "stm32f10x.h"
#include "math.h"
#include "timer.h"
#include "telemetry.h"

#include "algorithms/orientation/orientation.h"
#include "algorithms/altitude_algorithm/altitude_algorithm.h"
#include "algorithms/motor_algorithm/motor_algorithm.h"
#include "q_config.h"
#include "radio_control.h"
#include "stab_algorithm.h"
#include "types.h"

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
    	Altitude_AlgorithmUpdate(&global_accel);

    	int32_t altitude, altitude_velocity;
    	Altitude_GetVerticalState(&altitude, &altitude_velocity);

    	//computing main algorithm

    	//setup motors trust

    	//send telemetry

    	//waiting end of cycle
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
}
