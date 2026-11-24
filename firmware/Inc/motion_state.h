#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include "ramp_profile.h"

extern RampProfile forwardProfile;
extern RampProfile rotationProfile;

void MotionState_Init(void);

#endif
