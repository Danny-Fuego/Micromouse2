#include "motion_primitives.h"
#include "motion_actions.h"

void MoveForward(void)
{
    Run_PositionControlTest(CELL_DISTANCE_MM, MOVE_SPEED_MM_S);
}

void MoveForwardDistance(float distance_mm)
{
    Run_PositionControlTest(distance_mm, MOVE_SPEED_MM_S);
}

void TurnRight(void)
{
    Run_SpinTurnTest(TURN_RIGHT_DEG);
}

void TurnLeft(void)
{
    Run_SpinTurnTest(TURN_LEFT_DEG);
}

void Turn180(void)
{
    Run_SpinTurnTest(TURN_180_DEG);
}

void TurnRightCorrected(float driftCorrection_deg)
{
    Run_SpinTurnTest(TURN_RIGHT_DEG - driftCorrection_deg);
}

void TurnLeftCorrected(float driftCorrection_deg)
{
    Run_SpinTurnTest(TURN_LEFT_DEG - driftCorrection_deg);
}

void Turn180Corrected(float driftCorrection_deg)
{
    Run_SpinTurnTest(TURN_180_DEG - driftCorrection_deg);
}

float Resquare(void)
{
    // Returns 0.0f automatically if there was no front wall to resquare
    // against — Run_ResquareTest() handles that skip case internally.
    return Run_ResquareTest();
}
