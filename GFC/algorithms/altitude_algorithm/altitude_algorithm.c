#include "altitude_algorithm.h"
#include "BMP085_high.h"
#include "sonar.h"
#include "AB_filter.h"
#include "q_config.h"

#define SWITCH_SONAR_TO_BARO 1700 //altitude of switching between sonar and baro
#define SWITCH_BARO_TO_SONAR 1500

float altitude;
float altitude_velocity;
float altitude_acceleration;


float altitude1;
float altitude_velocity1;
float altitude_acceleration1;

uint8_t state;
AlfaBetaFilterFloat_Struct baro_filter, baro_velo_filter;
AlfaBetaFilterFloat_Struct sonar_filter, sonar_velo_filter;
/*
0 - sonar main, initial state
1 - sonar return value greates then SWITCH_SONAR_TO_BARO, check if it not ones
2 - barometer main, sonar return more than SWITCH_BARO_TO_SONAR
3 - sonar return less than SWITCH_BARO_TO_SONAR, check if it not ones and then
1 - sonar can't work, baro used
2 - altitude is less than 2 meters but sonar can't find a ground
*/
void Altitude_Init(){
    BMP085_begin(BMP085_ULTRAHIGHRES);
    uint32_t current_pressure = BMP085_measurePressure();
    BMP085_setZeroPressure3(current_pressure, 50);
    Sonar_Init();
    altitude = 0.0f;
    altitude_velocity = 0.0f;
    state = 0;
    AlfaBetaFilterFloat_SetupStruct(&baro_filter, 0.01, UPDATE_PERIOD_IN_SEC);
    AlfaBetaFilterFloat_SetupStruct(&sonar_filter, 0.01, UPDATE_PERIOD_IN_SEC);
    AlfaBetaFilterFloat_SetupStruct(&baro_velo_filter, 0.01, UPDATE_PERIOD_IN_SEC);
    AlfaBetaFilterFloat_SetupStruct(&sonar_velo_filter, 0.01, UPDATE_PERIOD_IN_SEC);


}
void Altitude_AlgorithmUpdate(float accel_z){
	static float accel_z_bias = 9490.0f;
	accel_z -=accel_z_bias;
	accel_z_bias += accel_z*0.00001f;
    BMP085_update();
    Sonar_Update();
    int32_t sonar_alt_raw, sonar_alt_velo_raw;
    int32_t baro_alt_raw, baro_alt_velo_raw;
    Sonar_GetProcessedData(&sonar_alt_raw,&sonar_alt_velo_raw);
    BMP085_getProcessesData(&baro_alt_raw,&baro_alt_velo_raw);

    altitude_acceleration = (int32_t)(accel_z);
    altitude_velocity = AlfaBetaFilerFloat_Update(&sonar_velo_filter, accel_z, (float)sonar_alt_velo_raw);
	altitude = AlfaBetaFilerFloat_Update(&sonar_filter, altitude_velocity, (float)sonar_alt_raw);
	altitude_velocity1 = AlfaBetaFilerFloat_Update(&baro_velo_filter, accel_z, (float)baro_alt_velo_raw);
	altitude1 = AlfaBetaFilerFloat_Update(&baro_filter, altitude_velocity1, (float)baro_alt_raw);

}
void Altitude_GetVerticalState(float * _altitude, float * _vertical_velocity, float * _altitude_acceleration){
	*_altitude = altitude;
	*_vertical_velocity = altitude_velocity;
	*_altitude_acceleration = altitude_acceleration;
}
void Altitude_GetVerticalState1(float * _altitude, float * _vertical_velocity, float * _altitude_acceleration){
	*_altitude = altitude1;
	*_vertical_velocity = altitude_velocity1;
	*_altitude_acceleration = altitude_acceleration;
}
uint8_t Altitude_GetCurrentSensor(){
	return state;
}
