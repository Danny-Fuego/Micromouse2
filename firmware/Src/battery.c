#include "battery.h"
#include "sensors.h"


#define BATTERY_ADC_CHANNEL 9

// TODO: replace with real R1/R2 once resistors are chosen.
#define BATTERY_R1 10000.0f
#define BATTERY_R2 5100.0f
#define BATTERY_DIVIDER_RATIO (BATTERY_R2 / (BATTERY_R1 + BATTERY_R2))

#define ADC_FSR 4095.0f
#define ADC_REF_VOLTS 3.3f
#define BATTERY_MULTIPLIER (ADC_REF_VOLTS / ADC_FSR / BATTERY_DIVIDER_RATIO)

static float lastVoltage = 7.4f; // sane startup default until first real read completes
static uint8_t requestOutstanding = 0;

// Call this once per control tick (e.g. from the main loop or Systick,
// alongside IR_BackgroundServiceStep). It does NOT block - it either
// starts a new request or collects a finished one, same non-blocking
// pattern as the IR background scanner.
void Battery_Update(void){
	if (requestOutstanding){
		if (IR_ExternalReadReady()){
			uint16_t raw = IR_GetExternalReadResult();
			lastVoltage = (float)raw * BATTERY_MULTIPLIER;
			requestOutstanding = 0;
		}
		return;
	}

	if (IR_RequestExternalRead(BATTERY_ADC_CHANNEL)){
		requestOutstanding = 1;
	}
}

float Battery_ReadVoltage(void){
	return lastVoltage;
}
