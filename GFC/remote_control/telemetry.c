#include "telemetry.h"

void Telemetry_sendToPilot(){
	static uint8_t skip_counter = 0;
	const uint8_t skip_num = 1;
	if(skip_counter<skip_num){
		skip_counter++;
		return;
	}
	skip_counter = 0;
	RadioChannel_transmitMaskedChannal();
}
