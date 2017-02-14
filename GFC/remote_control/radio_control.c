#include "radio_control.h"
#include "radio_channal.h"

#include "q_config.h"
#include "stab_algorithm.h"

Quaternion RC_quaternion;
Vector3 RC_spin;
int16_t RC_thrust;
int8_t last_connect_counter = 100;

void RC_update(){
    receive_all_available();
    if (is_rx_channal_fresh(TX_MASK2)) {
    	int16_t tmp[2];
    	get_rx_channals(tmp, TX_MASK, 2);
    	set_tx_mask(tmp[0] << 16 | tmp[1]);
	}
    if (is_rx_channal_fresh(PID_CONFIG)) {
    	int16_t tmp = get_rx_channal(PID_CONFIG);
    	update_PID_config(tmp);
	}
//    RC_state_update();

}
void RC_state_update(){
	static float RC_yaw = 0.0;
	static Vector3_int16 RC_raw_data_prev = {0, 0, 0};
	//update lose connect state
	if (is_rx_channal_fresh(LOST_CONNECTION)) {
		last_connect_counter = 100;
	}
	if (last_connect_counter > 0) {
		last_connect_counter--;
	}

	//load new data from RC
	Vector3_int16 RC_raw_data;
	get_rx_channals((int16_t *)&RC_raw_data, RC_ANGLES, 3);
	RC_thrust = get_rx_channal(THRUST);
	//set zero x and y angles for saving stability
	if (last_connect_counter == 0)
	{
		RC_thrust /= 2;
		RC_raw_data.x = 0;
		RC_raw_data.y = 0;
	}
	// calculate different between previous algorithm's angles state and current RC state
	EuclidAngles delta;
	delta.pitch = RC_raw_data.x - RC_raw_data_prev.x;
	delta.roll = RC_raw_data.y - RC_raw_data_prev.y;
	//limit different for smooth RC sensitivity
	static int16_t limit = 10;
	limit_float( &delta.pitch, limit);
	limit_float( &delta.roll, limit);
	//set smooth RC state as current for this iteration and previous for next one
	RC_raw_data_prev.x += delta.pitch;
	RC_raw_data_prev.y += delta.roll;
	//add changes for z angle
	float delta_yaw = RC_raw_data.z*minuteArc_to_rad;
	RC_yaw += delta_yaw *update_period_in_sec;

	EuclidAngles RC_angles;
	RC_angles.pitch = RC_raw_data_prev.x*minuteArc_to_rad;
	RC_angles.roll = RC_raw_data_prev.y*minuteArc_to_rad;
	RC_angles.yaw = 0.0f;
	Quaternion tmp1, tmp2;
	quaternionComposition(RC_angles, &tmp1);

	RC_angles.pitch = 0.0f;
	RC_angles.roll = 0.0f;
	RC_angles.yaw = RC_yaw;
	quaternionComposition(RC_angles, &tmp2);
	quaternionMultiplication(&tmp1, &tmp2, &RC_quaternion);

	RC_spin.x = delta.pitch*minuteArc_to_rad*frq;
	RC_spin.y = delta.roll*minuteArc_to_rad*frq;
	RC_spin.z = delta_yaw;
}
void get_RC_state(Quaternion * RC_quaternion_, Vector3 * RC_spin_, int16_t * thrust_, int8_t * last_connect_counter_)
{
	*RC_quaternion_ = RC_quaternion;
	*RC_spin_ = RC_spin;
	*thrust_ = RC_thrust;
	*last_connect_counter_ = last_connect_counter;
}
