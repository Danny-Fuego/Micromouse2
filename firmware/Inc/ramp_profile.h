#ifndef RAMP_PROFILE_H
#define RAMP_PROFILE_H

#include <stdint.h>

#define LOOP_FREQUENCY_HZ  1000.0f
#define LOOP_INTERVAL_SEC  (1.0f / LOOP_FREQUENCY_HZ)

typedef struct
{
    float targetDist;
    float requestedMaxSpeed;
    float maxSpeed;
    float approachSpeed;
    float accel;
    float decel;
    float brakeMargin;
    float finalBrakeDist;
    float minMoveSpeed;
    float kineticMinSpeed;
    float stopDistTol;

    int8_t sign;

    float position;
    float speed;
    uint8_t isDecelerating;
    uint8_t isFinished;
} RampProfile;

void RampProfile_Init(RampProfile *p,
                       float approachSpeed,
                       float accel,
                       float decel,
                       float brakeMargin,
                       float finalBrakeDist,
                       float minMoveSpeed,
                       float kineticMinSpeed,
                       float stopDistTol);

void RampProfile_Reset(RampProfile *p);
void RampProfile_SetMove(RampProfile *p, float distance, float maxSpeed);
float RampProfile_Update(RampProfile *p);

float RampProfile_GetPosition(RampProfile *p);
float RampProfile_GetSpeed(RampProfile *p);
uint8_t RampProfile_IsDecelerating(RampProfile *p);
uint8_t RampProfile_IsFinished(RampProfile *p);

// ADDED for fast-motion handoff tests (implemented at the end of ramp_profile.c)
float   RampProfile_GetApproachSpeed(RampProfile *p);
void    RampProfile_SetApproachSpeed(RampProfile *p, float approachSpeed);
float   RampProfile_GetRemaining(RampProfile *p);
uint8_t RampProfile_AppendMove(RampProfile *p, float distance, float maxSpeed, float approachSpeed);


#endif
