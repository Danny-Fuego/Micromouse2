#ifndef MOTION_ACTIONS_H
#define MOTION_ACTIONS_H

#include <stdint.h>

extern uint8_t g_verboseMotionLogging;

void Run_PositionControlTest(float distance_mm, float maxSpeed);
void Run_SpinTurnTest(float angle_deg);
float Run_ResquareTest(void);

#endif
