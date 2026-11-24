#include "motion_state.h"

RampProfile forwardProfile;
RampProfile rotationProfile;

void MotionState_Init(void)
{
	RampProfile_Init(&forwardProfile,
	                  300.0f,   // approachSpeed — unchanged unless you want to revisit
	                  800.0f,   // accel — unchanged
	                  600.0f,   // decel — unchanged
	                  30.0f,    // brakeMargin — unchanged
	                  5.0f,     // finalBrakeDist — unchanged
	                  415.0f,   // minMoveSpeed (SF) — was 593.0f
	                  107.0f,   // kineticMinSpeed (KF) — was 163.0f
	                  2.0f);    // stopDistTol — unchanged

	RampProfile_Init(&rotationProfile,
	                  100.0f,    // approachSpeed — unchanged unless you want to revisit
	                  2000.0f,
	                  2000.0f,
	                  10.0f,     // brakeMargin — restored from 0
	                  3.0f,      // finalBrakeDist — restored from 0
	                  328.0f,    // minMoveSpeed (SF) — was 521.0f
	                  45.0f,    // kineticMinSpeed (KF) — was 102.0f
	                  0.0f);    // stopDistTol (deg)
}
