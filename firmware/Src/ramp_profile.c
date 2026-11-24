#include "ramp_profile.h"
#include <math.h>

void RampProfile_Init(RampProfile *p,
                       float approachSpeed,
                       float accel,
                       float decel,
                       float brakeMargin,
                       float finalBrakeDist,
                       float minMoveSpeed,
                       float kineticMinSpeed,
                       float stopDistTol)
{
    p->approachSpeed   = approachSpeed;
    p->accel           = accel;
    p->decel           = decel;
    p->brakeMargin     = brakeMargin;
    p->finalBrakeDist  = finalBrakeDist;
    p->minMoveSpeed    = minMoveSpeed;
    p->kineticMinSpeed = kineticMinSpeed;
    p->stopDistTol     = stopDistTol;

    p->sign               = 1;
    p->targetDist         = 0.0f;
    p->requestedMaxSpeed  = 0.0f;
    p->maxSpeed           = 0.0f;
    p->position           = 0.0f;
    p->speed              = 0.0f;
    p->isDecelerating     = 0;
    p->isFinished         = 0;
}

void RampProfile_Reset(RampProfile *p)
{
    p->position       = 0.0f;
    p->speed          = 0.0f;
    p->isDecelerating = 0;
    p->isFinished     = 0;
}

void RampProfile_SetMove(RampProfile *p, float distance, float maxSpeed)
{
    float reachableSpeed;
    float denom;

    p->sign = (distance < 0.0f) ? -1 : 1;
    if (distance < 0.0f) distance = -distance;
    if (maxSpeed < 0.0f) maxSpeed = -maxSpeed;

    p->targetDist        = distance;
    p->requestedMaxSpeed = maxSpeed;

    if (p->requestedMaxSpeed < p->approachSpeed)
    {
        p->requestedMaxSpeed = p->approachSpeed;
    }

    denom = (1.0f / (2.0f * p->accel)) + (1.0f / (2.0f * p->decel));

    if (p->targetDist > 0.0f && denom > 0.0f)
    {
        reachableSpeed = sqrtf(p->targetDist / denom);
    }
    else
    {
        reachableSpeed = p->approachSpeed;
    }

    p->maxSpeed = p->requestedMaxSpeed;

    if (reachableSpeed < p->maxSpeed) p->maxSpeed = reachableSpeed;
    if (p->maxSpeed < p->approachSpeed) p->maxSpeed = p->approachSpeed;

    RampProfile_Reset(p);
}

float RampProfile_Update(RampProfile *p)
{
    float remaining;
    float decelDistToApproach;
    float absSpeed = fabsf(p->speed);
    float absPosition = fabsf(p->position);

    remaining = p->targetDist - absPosition;
    if (remaining < 0.0f) remaining = 0.0f;

    if (remaining <= p->finalBrakeDist || remaining <= p->stopDistTol)
    {
        p->speed = 0.0f;
        p->isDecelerating = 0;
        p->isFinished = 1;
        return p->speed;
    }

    if (absSpeed > p->approachSpeed)
    {
        decelDistToApproach = ((absSpeed * absSpeed) - (p->approachSpeed * p->approachSpeed)) / (2.0f * p->decel);
    }
    else
    {
        decelDistToApproach = 0.0f;
    }

    if ((decelDistToApproach + p->brakeMargin + p->finalBrakeDist) >= remaining)
    {
        p->isDecelerating = 1;
        absSpeed -= p->decel * LOOP_INTERVAL_SEC;
        if (absSpeed < p->approachSpeed) absSpeed = p->approachSpeed;
    }
    else
    {
        p->isDecelerating = 0;
        absSpeed += p->accel * LOOP_INTERVAL_SEC;
        if (absSpeed > p->maxSpeed) absSpeed = p->maxSpeed;
    }

    float currentFloor = (absSpeed == 0.0f) ? p->minMoveSpeed : p->kineticMinSpeed;
    if (absSpeed < currentFloor) absSpeed = currentFloor;

    p->speed = p->sign * absSpeed;
    p->position += p->speed * LOOP_INTERVAL_SEC;

    return p->speed;
}

float RampProfile_GetPosition(RampProfile *p) { return p->position; }
float RampProfile_GetSpeed(RampProfile *p)    { return p->speed; }
uint8_t RampProfile_IsDecelerating(RampProfile *p) { return p->isDecelerating; }
uint8_t RampProfile_IsFinished(RampProfile *p) { return p->isFinished; }


// ============================================================================
// ADDED for fast-motion handoff tests.
// Everything above this line is your original file, unmodified.
// These are new functions only; none of the existing functions call them.
// ============================================================================
#include "stm32f4xx.h"   // __get_PRIMASK / __disable_irq / __set_PRIMASK

float RampProfile_GetApproachSpeed(RampProfile *p)
{
    return p->approachSpeed;
}

// Change the speed a move brakes down to (and holds) before its final stop.
// Call it BEFORE RampProfile_SetMove: SetMove plans using the value that is set.
void RampProfile_SetApproachSpeed(RampProfile *p, float approachSpeed)
{
    p->approachSpeed = approachSpeed;   // single 32-bit write, atomic on Cortex-M4
}

// Distance the profile still has to cover before it reaches its target
// (goes slightly negative if it has run past it).
float RampProfile_GetRemaining(RampProfile *p)
{
    return p->targetDist - fabsf(p->position);
}

// Continue the move that is running into a longer one WITHOUT stopping:
//   - keeps the current speed and direction (no reset to 0)
//   - the distance not yet covered is kept, and 'distance' more is added after it
//   - the new stretch is planned with 'approachSpeed' (the speed it brakes to
//     before its own final stop), so pass your normal approach speed here
// 'distance' is always taken in the current direction of travel.
// Done with interrupts masked so SysTick never sees a half-written profile.
// Returns 1 if the profile was still running (seamless), 0 if it had already
// started its final stop (the speed dipped to 0 first - handoff was too late).
uint8_t RampProfile_AppendMove(RampProfile *p, float distance, float maxSpeed, float approachSpeed)
{
    if (distance < 0.0f) distance = -distance;
    if (maxSpeed < 0.0f) maxSpeed = -maxSpeed;

    float requested = maxSpeed;
    if (requested < approachSpeed) requested = approachSpeed;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t seamless = p->isFinished ? 0 : 1;

    float total = (p->targetDist - fabsf(p->position)) + distance;
    if (total < 0.0f) total = 0.0f;

    float denom = (1.0f / (2.0f * p->accel)) + (1.0f / (2.0f * p->decel));
    float reachable = (total > 0.0f && denom > 0.0f) ? sqrtf(total / denom) : approachSpeed;

    float planned = requested;
    if (reachable < planned) planned = reachable;
    if (planned < approachSpeed) planned = approachSpeed;

    p->approachSpeed     = approachSpeed;
    p->targetDist        = total;
    p->requestedMaxSpeed = requested;
    p->maxSpeed          = planned;
    p->position          = 0.0f;
    p->isDecelerating    = 0;
    p->isFinished        = 0;
    // p->speed and p->sign are deliberately left alone.

    __set_PRIMASK(primask);

    return seamless;
}
