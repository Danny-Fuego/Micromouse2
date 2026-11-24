#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdint.h>
#include "ramp_profile.h"

void MotionController_Init(void);
void MotionController_Reset(void);

// forwardTargetSpeed/rotationTargetSpeed come from the two RampProfiles.
// currentHeading_deg comes from BNO055_ReadHeading.
void MotionController_Update(float forwardTargetSpeed_mm_s,
                              float forwardTargetPos_mm,
                              float rotationTargetSpeed_deg_s,
                              float rotationTargetPos_deg,
                              float currentHeading_deg,
                              float steeringAdjustment_deg,
                              uint8_t robotIsMoving);

void MotionController_SetOutputEnabled(uint8_t enabled);
void MotionController_SetFeedforwardEnabled(uint8_t enabled);

// Debugging
float MotionController_GetLastLeftVolts(void);
float MotionController_GetLastRightVolts(void);
float MotionController_GetLastPosOutput(void);
float MotionController_GetLastRotOutput(void);
float MotionController_GetLastRotError(void);

// Called ONCE per fresh IR scan cycle (~every 9ms), from inside
// IR_BackgroundServiceStep() — recomputes crossTrackError, the P-term,
// and the speed-normalized angle estimate against real elapsed time
// since the last genuinely fresh reading. Do NOT call this every SysTick
// tick — the IR readings are only actually fresh once per full scan cycle.
void MotionController_OnFreshIRReading(float leftDist_in, float rightDist_in,
                                         uint8_t seeLeft, uint8_t seeRight,
                                         float forwardSpeed_mm_s);

// Cheap, safe to call every SysTick tick (1ms) — just combines whatever
// MotionController_OnFreshIRReading() last computed. Does no new sensing
// or math beyond clamping and storing the result.
float MotionController_CalculateSteeringAdjustment(void);

float MotionController_GetLastSteeringAdjustment(void);
void MotionController_ResetSteering(void);
float MotionController_GetLastAngleEstimateDeg(void);

#endif
