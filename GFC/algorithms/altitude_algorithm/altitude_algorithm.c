#include "altitude_algorithm.h"
#include "BMP085_high.h"
#include "sonar.h"
#include "AB_filter.h"
#include "q_config.h"

#define SWITCH_SONAR_TO_BARO 2000 //altitude of switching between sonar and baro
#define SWITCH_BARO_TO_SONAR 1500


//algorithm modes
#define BARO_ONLY	1
#define SONAR_ONLY	2
#define BAROnSONAR	3


int32_t altitude;
int32_t altitude_velocity;
uint8_t state;
AlfaBetaFilterInt_Struct baro_filter, baro_velo_filter;
AlfaBetaFilterInt_Struct sonar_filter, sonar_velo_filter;
/*
0 - sonar used
1 - sonar can't work, baro used
2 - altitude is less than 2 meters but sonar can't find a ground
*/
void Altitude_Init(){
    BMP085_begin(BMP085_ULTRAHIGHRES);
    Sonar_Init();
    altitude = 0.0f;
    altitude_velocity = 0.0f;
    state = 0;
    AlfaBetaFilterInt_SetupStruct(&baro_filter, 950, UPDATE_FRQ);
    AlfaBetaFilterInt_SetupStruct(&sonar_filter, 900, UPDATE_FRQ);
    AlfaBetaFilterInt_SetupStruct(&baro_velo_filter, 950, UPDATE_FRQ);
    AlfaBetaFilterInt_SetupStruct(&sonar_velo_filter, 950, UPDATE_FRQ);

}
void Altitude_AlgorithmUpdate(Vector3 * global_accel){
    BMP085_update();
    Sonar_Update();
    int32_t alt, alt_velo;
    switch (state) {
		case 0:
			Sonar_GetProcessedData(&alt,&alt_velo);
			altitude_velocity = AlfaBetaFilerInt_Update(&sonar_velo_filter, (int32_t)(global_accel->z*1000.0f), alt_velo);
			altitude = AlfaBetaFilerInt_Update(&sonar_filter, altitude_velocity, alt);
			if (altitude > SWITCH_SONAR_TO_BARO)
				state = 1;
			break;
		case 1:
			BMP085_getProcessesData(&alt,&alt_velo);
			altitude_velocity = AlfaBetaFilerInt_Update(&baro_velo_filter, (int32_t)(global_accel->z*1000.0f), alt_velo);
			altitude = AlfaBetaFilerInt_Update(&baro_filter, altitude_velocity, alt);
			if (altitude < SWITCH_BARO_TO_SONAR)
				state = 0;
			break;
		default:
			break;
	}
}
void Altitude_GetVerticalState(int32_t * _altitude, int32_t * _vertical_velocity){
	altitude = *_altitude;
	altitude_velocity = *_vertical_velocity;
}
uint8_t Altitude_GetCurrentSensor(){
	return state;
}
