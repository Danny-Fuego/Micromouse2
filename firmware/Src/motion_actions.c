#include "motion_actions.h"
#include "bt.h"
#include "encoder.h"
#include "motor.h"
#include "bno055.h"
#include "battery.h"
#include "sensors.h"
#include "ramp_profile.h"
#include "motion_controller.h"
#include "motion_state.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;

#define RESQUARE_OVERSHOOT_IN     0.8f
#define RESQUARE_BACKTRACK_IN     3.0f
#define RESQUARE_DRIVE_SPEED      800.0f
#define RESQUARE_REVERSE_SPEED    800.0f

// CHANGED: wait settings pulled out as tunables (were hard-coded 3 / 10 / 300).
// Each "still tick" below is one 100 ms check of the loop gate.
#define STRAIGHT_STILL_TICKS_REQUIRED  2     // was 3
#define SPIN_STILL_TICKS_REQUIRED      2     // was 10
#define RESQUARE_SETTLE_MS             50    // was HAL_Delay(300)

// CHANGED: 1 = keep the one-line "finalHeading / residualError" summary after
// every move (your quality metric while tuning). Set to 0 once you're happy.
// Everything else chatty is behind g_verboseMotionLogging.
#define PRINT_MOVE_SUMMARY             1

uint8_t g_verboseMotionLogging = 0;

static float s_startHeadingDeg = 0.0f;
static float s_finalHeadingDeg = 0.0f;
static float s_residualHeadingErrorDeg = 0.0f;

static float NormalizeAngleDifference(float angleDeg)
{
    while (angleDeg > 180.0f)  angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

void Run_PositionControlTest(float distance_mm, float maxSpeed)
{
    g_rotationBraked = 0;
    g_robotIsMoving = 1;

    uint32_t startTime, lastPrintTime, now;
    int32_t startCntL, startCntR;
    int32_t prevCntL, prevCntR;
    uint32_t stillTicks = 0;
    const uint32_t STILL_TICKS_REQUIRED = STRAIGHT_STILL_TICKS_REQUIRED;   // CHANGED (was 3)

    Encoder_ResetAll();
    Encoder_Update();
    startCntL = Encoder_GetLeftCount();
    startCntR = Encoder_GetRightCount();
    prevCntL = startCntL;
    prevCntR = startCntR;

    MotionController_Reset();

    s_startHeadingDeg = g_currentHeading;

    if (g_verboseMotionLogging)
    {
        BT_Sendf("startHeading=%.2f\r\n", s_startHeadingDeg);
    }

    RampProfile_SetMove(&forwardProfile, distance_mm, maxSpeed);
    g_motionControlEnabled = 1;

    startTime = HAL_GetTick();
    lastPrintTime = startTime - 200;

    if (g_verboseMotionLogging)
    {
        BT_Sendf("\r\n===== POSITION CONTROL TEST dist=%.1f maxSpeed=%.1f =====\r\n",
                 distance_mm, maxSpeed);
        BT_SendLine("time_ms,targetPos,targetSpeed,isDecel,measPos,posError,heading,rotError,posOutput,rotOutput,leftVolts,rightVolts,cntL,cntR,distL,distR,seeL,seeR,steerAdj,kdAngleDeg");
    }

    while (1)
    {
        now = HAL_GetTick();

        if ((now - lastPrintTime) >= 100)
        {
            Encoder_Update();
            int32_t cntL = Encoder_GetLeftCount();
            int32_t cntR = Encoder_GetRightCount();

            float distL = (float)(cntL - startCntL) * MM_PER_TICK_LEFT;
            float distR = (float)(cntR - startCntR) * MM_PER_TICK_RIGHT;
            float measPos = 0.5f * (distL + distR);

            float targetPos = RampProfile_GetPosition(&forwardProfile);
            float posError = targetPos - measPos;

            float irDistL = IR_GetDistanceL_BG();
            float irDistR = IR_GetDistanceR_BG();
            uint8_t irSeeL = IR_WallLeft_BG();
            uint8_t irSeeR = IR_WallRight_BG();
            float steerAdj = MotionController_GetLastSteeringAdjustment();
            float kdAngleDeg = MotionController_GetLastAngleEstimateDeg();

            if (g_verboseMotionLogging)
            {
                BT_Sendf("%lu,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%.2f,%.2f,%u,%u,%.3f,%.3f\r\n",
                         now - startTime, targetPos, RampProfile_GetSpeed(&forwardProfile),
                         RampProfile_IsDecelerating(&forwardProfile), measPos, posError,
                         g_currentHeading, MotionController_GetLastRotError(),
                         MotionController_GetLastPosOutput(), MotionController_GetLastRotOutput(),
                         MotionController_GetLastLeftVolts(), MotionController_GetLastRightVolts(),
                         cntL, cntR,
                         irDistL, irDistR, irSeeL, irSeeR, steerAdj, kdAngleDeg);
            }

            int32_t deltaL = cntL - prevCntL;
            int32_t deltaR = cntR - prevCntR;

            if (RampProfile_IsFinished(&forwardProfile) &&
                abs(deltaL) < 2 && abs(deltaR) < 2)
            {
                g_robotIsMoving = 0;
                stillTicks++;
                if (stillTicks >= STILL_TICKS_REQUIRED)
                {
                    break;
                }
            }
            else
            {
                g_robotIsMoving = 1;
                stillTicks = 0;
            }

            prevCntL = cntL;
            prevCntR = cntR;
            lastPrintTime = now;
        }

        if (g_runExpired) break;   // run clock: time is up, motors are already cut

        if ((now - startTime) > 10000)
        {
            BT_SendLine("===== POSITION CONTROL TEST TIMEOUT =====");
            break;
        }
    }

    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    MotionController_Reset();
    Motor_Stop(MOTOR_BRAKE);

    s_finalHeadingDeg = g_currentHeading;
    s_residualHeadingErrorDeg = NormalizeAngleDifference(s_finalHeadingDeg - s_startHeadingDeg);

#if PRINT_MOVE_SUMMARY   // CHANGED: was unconditional
    BT_Sendf("finalHeading=%.2f residualError=%.2f\r\n",
             s_finalHeadingDeg, s_residualHeadingErrorDeg);
#endif
    if (g_verboseMotionLogging)   // CHANGED: was unconditional
    {
        BT_SendLine("===== POSITION CONTROL TEST DONE =====");
    }
}

void Run_SpinTurnTest(float angle_deg)
{
    g_rotationBraked = 0;
    uint32_t startTime, lastPrintTime, now;
    int32_t prevCntL, prevCntR;
    uint32_t stillTicks = 0;

    // CHANGED: the pre-turn "wait until still" loop that used to be here
    // (HAL_Delay(50) x 5 checks = ~250 ms per turn) is removed. Every turn
    // follows a straight / resquare / previous turn that already confirmed
    // the wheels were still before returning.

    g_robotIsMoving = 1;

    RampProfile_Reset(&rotationProfile);
    RampProfile_SetMove(&rotationProfile, angle_deg, 50.0f);

    float startHeading = g_currentHeading;
    g_rotationTargetHeading = startHeading + angle_deg;
    g_spinTurnActive = 1;

    g_motionControlEnabled = 1;

    if (g_verboseMotionLogging)
    {
        BT_Sendf("\r\n===== SPIN TURN TEST angle=%.1f =====\r\n", angle_deg);
        BT_Sendf("startHeading=%.2f target=%.2f\r\n", startHeading, g_rotationTargetHeading);
        BT_SendLine("time_ms,targetAngle,targetOmega,isDecel,heading,rotError,rotOutput,leftVolts,rightVolts,cntL,cntR,remReal,ovr,tgt");
    }

    startTime = HAL_GetTick();
    lastPrintTime = startTime - 200;
    stillTicks = 0;
    Encoder_Update();
    prevCntL = Encoder_GetLeftCount();
    prevCntR = Encoder_GetRightCount();

    while (1)
    {
        now = HAL_GetTick();

        if ((now - lastPrintTime) >= 100)
        {
            Encoder_Update();
            int32_t cntL = Encoder_GetLeftCount();
            int32_t cntR = Encoder_GetRightCount();

            float remainingReal = BNO055_AngleError(g_rotationTargetHeading, g_currentHeading);
            uint8_t overrideActive = (g_spinTurnActive &&
                                       RampProfile_GetSpeed(&rotationProfile) == 0.0f &&
                                       RampProfile_IsFinished(&rotationProfile) &&
                                       fabsf(remainingReal) > ROT_FINAL_ANGLE_TOLERANCE_DEG) ? 1 : 0;

            if (g_verboseMotionLogging)
            {
                BT_Sendf("%lu,%.2f,%.2f,%u,%.2f,%.2f,%.3f,%.3f,%.3f,%ld,%ld,%.2f,%u,%.2f\r\n",
                         now - startTime,
                         RampProfile_GetPosition(&rotationProfile),
                         RampProfile_GetSpeed(&rotationProfile),
                         RampProfile_IsDecelerating(&rotationProfile),
                         g_currentHeading,
                         MotionController_GetLastRotError(),
                         MotionController_GetLastRotOutput(),
                         MotionController_GetLastLeftVolts(),
                         MotionController_GetLastRightVolts(),
                         cntL, cntR,
                         remainingReal, overrideActive, g_rotationTargetHeading);
            }

            int32_t deltaL = cntL - prevCntL;
            int32_t deltaR = cntR - prevCntR;

            if (fabsf(remainingReal) <= ROT_FINAL_ANGLE_TOLERANCE_DEG &&
                abs(deltaL) < 2 && abs(deltaR) < 2)
            {
                g_robotIsMoving = 0;
                stillTicks++;
                if (stillTicks >= SPIN_STILL_TICKS_REQUIRED) break;   // CHANGED (was 10)
            }
            else
            {
                g_robotIsMoving = 1;
                stillTicks = 0;
            }

            prevCntL = cntL;
            prevCntR = cntR;
            lastPrintTime = now;
        }

        if (g_runExpired) break;   // run clock: time is up, motors are already cut

        if ((now - startTime) > 5000)
        {
            BT_SendLine("===== SPIN TURN TIMEOUT =====");
            break;
        }
    }

    g_spinTurnActive = 0;
    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    Motor_Stop(MOTOR_BRAKE);

    float actualTurned = BNO055_AngleError(g_currentHeading, startHeading);
    float finalError = BNO055_AngleError(angle_deg, actualTurned);
#if PRINT_MOVE_SUMMARY   // CHANGED: was unconditional
    BT_Sendf("finalHeading=%.2f actualTurned=%.2f targetAngle=%.2f error=%.2f\r\n",
             g_currentHeading, actualTurned, angle_deg, finalError);
#endif

    MotionController_Reset();
    MotionController_ResetSteering();

    if (!g_runExpired && (fabsf(finalError) > SPIN_TURN_ANOMALY_THRESHOLD_DEG))   // never re-correct once the clock has cut the motors
    {
        BT_Sendf("ANOMALY DETECTED — correcting residual error of %.2f deg\r\n", finalError);
        Run_SpinTurnTest(finalError);
    }

    if (g_verboseMotionLogging)   // CHANGED: was unconditional
    {
        BT_SendLine("===== SPIN TURN DONE =====");
    }
}

float Run_ResquareTest(void)
{
    uint8_t seeFront = IR_WallFront_BG();
    float frontDist_in = IR_GetDistanceF_BG();

    // CHANGED: all chatter in this function is now behind g_verboseMotionLogging
    // (it used to print 3-4 lines on every call, even when it skipped).
    if (g_verboseMotionLogging)
    {
        BT_SendLine("===== RESQUARE TEST START =====");
        BT_Sendf("seeFront=%u frontDist=%.2f\r\n", seeFront, frontDist_in);
    }

    if (!seeFront)
    {
        if (g_verboseMotionLogging)
        {
            BT_SendLine("No front wall detected — skipping resquare.");
            BT_SendLine("===== RESQUARE TEST DONE (skipped) =====");
        }
        return 0.0f;
    }

    float driveDistance_in = frontDist_in + RESQUARE_OVERSHOOT_IN;
    float driveDistance_mm = driveDistance_in * 25.4f;

    if (g_verboseMotionLogging)
    {
        BT_Sendf("Driving %.2f in (%.1f mm) to make wall contact...\r\n",
                 driveDistance_in, driveDistance_mm);
    }

    Run_PositionControlTest(driveDistance_mm, RESQUARE_DRIVE_SPEED);

    // True, wall-verified parallel heading, captured right after contact —
    // everything from here is judged against this reference point.
    float trueParallelHeading = g_currentHeading;

    HAL_Delay(RESQUARE_SETTLE_MS);   // CHANGED (was 300)

    float backtrack_mm = -RESQUARE_BACKTRACK_IN * 25.4f;

    if (g_verboseMotionLogging)
    {
        BT_Sendf("Reversing fixed %.2f in (%.1f mm) to turning distance...\r\n",
                 RESQUARE_BACKTRACK_IN, backtrack_mm);
    }

    Run_PositionControlTest(backtrack_mm, RESQUARE_REVERSE_SPEED);

    // Drift introduced by reverse alone — signed, so the caller can fold it
    // directly into the next turn's commanded angle rather than attempting
    // a separate, unreliably-small correction turn.
    float reverseDrift_deg = BNO055_AngleError(g_currentHeading, trueParallelHeading);

    if (g_verboseMotionLogging)
    {
        BT_Sendf("reverseDrift=%.2f deg\r\n", reverseDrift_deg);

        float finalFrontDist_in = IR_GetDistanceF_BG();
        BT_Sendf("finalFrontDist=%.2f (target ~%.1f)\r\n",
                 finalFrontDist_in, RESQUARE_BACKTRACK_IN);

        BT_SendLine("===== RESQUARE TEST DONE =====");
    }

    return reverseDrift_deg;
}
