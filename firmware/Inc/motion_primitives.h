#ifndef MOTION_PRIMITIVES_H
#define MOTION_PRIMITIVES_H

#include <stdint.h>

// One cell's worth of forward travel, in mm. Matches your validated
// 10-inch (254mm) move distance.
#define CELL_DISTANCE_MM  254.0f
#define MOVE_SPEED_MM_S    800.0f

// Empirically-corrected turn angles (from spin-turn validation).
#define TURN_RIGHT_DEG   86.0f
#define TURN_LEFT_DEG   -86.0f
#define TURN_180_DEG    176.0f

void MoveForward(void);
void MoveForwardDistance(float distance_mm);
void TurnRight(void);
void TurnLeft(void);
void Turn180(void);

// Drift-corrected turn variants — pass the signed heading drift returned by
// Resquare() (0.0f if resquare was skipped) and it gets folded straight into
// the commanded angle.
void TurnRightCorrected(float driftCorrection_deg);
void TurnLeftCorrected(float driftCorrection_deg);
void Turn180Corrected(float driftCorrection_deg);

float Resquare(void);

#endif
