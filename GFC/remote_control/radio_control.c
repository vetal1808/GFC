#include "radio_control.h"
#include "radio_channel.h"

#include "q_config.h"
#include "stab_algorithm.h"

Quaternion RC_quaternion;
Vector3 RC_spin;
int16_t RC_thrust;
Vector3 RC_position;
Vector3 RC_position_velocity;
#define LOSE_CONNECTION_MAX 100
int8_t last_connect_counter = LOSE_CONNECTION_MAX;

void RC_update(){
    RadioChannel_receiveAllAvailable();
    /*if TX_MASK4 channel updated, refresh transmit mask */
    if (RadioChannel_isRxChannalFresh(TX_MASK4)) {
    	int16_t tmp[4];
    	RadioChannel_getRxChannals(tmp, TX_MASK, 4);
    	RadioChannel_setTxMask(*((uint64_t *)tmp));
	}
	//update lose connect state
	if (RadioChannel_isRxChannalFresh(LOST_CONNECTION)) {
		last_connect_counter = 100;
	}
	if (last_connect_counter > 0) {
		last_connect_counter--;
	}
	int16_t tmp[4];
	RadioChannel_getRxChannals(tmp, RC_ANGLES, 4);
	switch (RC_getFlightMode()) {
		case 0://standby
			RC_thrust = 0;
			break;
		case 1://manual stab
			manualStateUpdate(tmp);
			break;
		case 2:
			altitudeHoldUpdate();
			break;
		case 3:
			positionHoldUpdate();
			break;
		default:
			break;
	}
}

int16_t RC_getFlightMode(){
	return RadioChannel_getRxChannal(FLIGHT_MODE);
}
int16_t RC_getLoseConnectionState(){
	return last_connect_counter;
}
void RC_getManualState(Quaternion *RC_quaternion_, Vector3 *RC_spin_, uint16_t thrust_){
	*RC_quaternion_ = RC_quaternion;
	*RC_spin_ = RC_spin;
	thrust_ = RC_thrust;
}
void RC_getAltitudeHoldState(Quaternion *RC_quaternion_, Vector3 *RC_spin_, float altitude, float alttitude_velocity){
	*RC_quaternion_ = RC_quaternion;
	*RC_spin_ = RC_spin;
	altitude = RC_position.z;
	alttitude_velocity = RC_position_velocity.z;
}
void RC_getPositionHoldState(Vector3 *RC_position_, Vector3 *RC_position_velocity_, float yaw, float yaw_derivative){
	*RC_position_ = RC_position;
	*RC_position_velocity_ = RC_position_velocity;
	yaw = RC_quaternion.q3;
	yaw_derivative = RC_spin.z;
}

void manualStateUpdate(int16_t * data){
	static float RC_yaw = 0.0;
	static Vector3_int16 RC_raw_data_prev = {0, 0, 0};
	static uint32_t avarage_thrust = 0; //for fail-safe
	//if lose connection set zero pitch, roll. Yaw still previous. Thrust average minus 5%
	if (last_connect_counter == 0) {
		RC_thrust = (avarage_thrust/256) - 50;
		EuclidAngles RC_heading;
		RC_heading.pitch = 0.0f;
		RC_heading.roll = 0.0f;
		RC_heading.yaw = RC_yaw;
		quaternionComposition(&RC_quaternion, &RC_heading);
		RC_raw_data_prev.x = 0;
		RC_raw_data_prev.y = 0;
		RC_raw_data_prev.z = 0;
		RC_spin.x = 0.0f;
		RC_spin.y = 0.0f;
		RC_spin.z = 0.0f;
	} else {
		avarage_thrust = (avarage_thrust*4095 + ((uint32_t)data[4])*256)/4096;
		//step 1: heading rotation
		float delta_yaw = (float)data[3]*ARC_MINUTE_TO_RAD; //heading derivation in radian per second
		RC_yaw += delta_yaw *UPDATE_PERIOD_IN_SEC;
		EuclidAngles RC_heading;
		RC_heading.pitch = 0.0f;
		RC_heading.roll = 0.0f;
		RC_heading.yaw = RC_yaw;
		Quaternion heading_quaternion;
		quaternionComposition(&RC_heading, &heading_quaternion);
		Vector3 heading_spin = {0.0, 0.0, delta_yaw};

		//step 2: pitch and roll (inclination) rotation

		//calculate differense between previous RC_state and current
		Vector3_int16 inclination_spin_raw;
		inclination_spin_raw.x = (data[0] - RC_raw_data_prev.x);
		inclination_spin_raw.y = (data[1] - RC_raw_data_prev.y);
		//limit different for smooth RC sensitivity
		#define RC_SMOOTH_LIMIT 10  // arc minute per period
		limit_int16(&inclination_spin_raw.x, RC_SMOOTH_LIMIT);
		limit_int16(&inclination_spin_raw.y, RC_SMOOTH_LIMIT);
		RC_raw_data_prev.x += inclination_spin_raw.x/UPDATE_FRQ;
		RC_raw_data_prev.y += inclination_spin_raw.y/UPDATE_FRQ;
		EuclidAngles RC_inclination;
		RC_inclination.pitch = RC_raw_data_prev.x * ARC_MINUTE_TO_RAD;
		RC_inclination.roll = RC_raw_data_prev.y * ARC_MINUTE_TO_RAD;
		RC_inclination.yaw = 0.0f;
		Quaternion inclination_quaternion;
		quaternionComposition(&RC_inclination, &inclination_quaternion);
		Vector3 inclination_spin;
		inclination_spin.x = inclination_spin_raw.x * ARC_MINUTE_TO_RAD;
		inclination_spin.y = inclination_spin_raw.y * ARC_MINUTE_TO_RAD;
		inclination_spin.z = inclination_spin_raw.z * ARC_MINUTE_TO_RAD;

		//step3: compose previous result
		quaternionMultiplication(&inclination_quaternion, &heading_quaternion, &RC_quaternion);
		rotateVector3ByQuatern(&inclination_quaternion, &heading_spin);
		RC_spin.x = heading_spin.x + inclination_spin.x;
		RC_spin.y = heading_spin.y + inclination_spin.y;
		RC_spin.z = heading_spin.z + inclination_spin.z;

	}
}
void altitudeHoldUpdate(int16_t *data){

}
void positionHoldUpdate(int16_t *data){

}
