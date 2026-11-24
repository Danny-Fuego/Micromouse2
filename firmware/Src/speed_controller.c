#include <speed_controller.h>

#define CONTROL_DT_SEC 0.010f
#define MM_PER_TICK 0.595f
#define PWM_MAX 1000

#define SPEED_KP 0.5f
#define SPEED_KI 0.1f
#define SPEED_KD 0.0f
#define ENABLE_DECEL_BIAS 0


static int32_t prevLeftCount  = 0;
static int32_t prevRightCount = 0;

static float leftIntegral  = 0.0f;
static float rightIntegral = 0.0f;

static float leftPrevError  = 0.0f;
static float rightPrevError = 0.0f;

static float leftMeasuredSpeed  = 0.0f;
static float rightMeasuredSpeed = 0.0f;

static int16_t leftPWM  = 0;
static int16_t rightPWM = 0;

static uint8_t hasStarted = 0;

static float minUsableSpeed_mm_s = 0.0f; // computed at init not hardcoded


#define LOOKUP_SIZE_L 9
#define LOOKUP_SIZE_R 10

static float leftPWMTrim[LOOKUP_SIZE_L] = {0};
static float rightPWMTrim[LOOKUP_SIZE_R] = {0};

// Left motor — recalibrated, averaged across 3 runs (charged/rested battery)
// Deadzone below 600 PWM — breakaway is inconsistent/unreliable in the
// 400-550 range across repeated runs, so the floor is set safely above it
// rather than at the literal (unreliable) edge.
static const float leftSpeedBP[LOOKUP_SIZE_L] = {
    0.0f,   600.0f,  670.0f,  735.0f,  845.0f,
    945.0f, 1055.0f, 1140.0f, 1370.0f
};
static const float leftPWMTable[LOOKUP_SIZE_L] = {
    0.0f, 600.0f, 650.0f, 700.0f, 750.0f,
    800.0f, 850.0f, 900.0f, 1000.0f
};

// Right motor — recalibrated, averaged across 3 runs (charged/rested battery)
// Deadzone below 600 PWM — same reasoning as left; right showed the most
// dramatic run-to-run swing at 550 PWM (0, ~400, 0 across the 3 runs).
static const float rightSpeedBP[LOOKUP_SIZE_R] = {
    0.0f,   495.0f,  575.0f,  670.0f,  790.0f,
    925.0f, 1060.0f, 1155.0f, 1275.0f, 1415.0f
};
static const float rightPWMTable[LOOKUP_SIZE_R] = {
    0.0f, 600.0f, 650.0f, 700.0f, 750.0f,
    800.0f, 850.0f, 900.0f, 950.0f, 1000.0f
};

static void Controller_ComputeMinUsableSpeed(void){
    // The first non-zero breakpoint in each table is that motor's real
    // deadzone edge. Use the higher of the two, so the startup gate
    // doesn't release until BOTH motors can actually respond.
	float leftEdge = leftSpeedBP[1];
	float rightEdge = rightSpeedBP[1];
	minUsableSpeed_mm_s = (leftEdge > rightEdge) ? leftEdge : rightEdge;
}

static int16_t Controller_ClampPWM(float pwm)
{
    if (pwm >  PWM_MAX) return  PWM_MAX;
    if (pwm < -PWM_MAX) return -PWM_MAX;
    return (int16_t)pwm;
}

static float Controller_Lookup1D(float x,
                                  const float *xBP,
                                  const float *yTable,
                                  int size)
{
    if (x <= xBP[0]) return yTable[0];
    if (x >= xBP[size - 1]) return yTable[size - 1];
    for (int i = 0; i < size - 1; i++)
    {
        if (x >= xBP[i] && x <= xBP[i + 1])
        {
            float ratio = (x - xBP[i]) / (xBP[i + 1] - xBP[i]);
            return yTable[i] + ratio * (yTable[i + 1] - yTable[i]);
        }
    }
    return yTable[size - 1];
}

static float Controller_LookupLeftPWM(float speed)
{
    float absSpeed = fabsf(speed);
    float pwm = Controller_Lookup1D(absSpeed, leftSpeedBP, leftPWMTable, LOOKUP_SIZE_L);
    if (speed < 0.0f) pwm = -pwm;
    return pwm;
}

static float Controller_LookupRightPWM(float speed)
{
    float absSpeed = fabsf(speed);
    float pwm = Controller_Lookup1D(absSpeed, rightSpeedBP, rightPWMTable, LOOKUP_SIZE_R);
    if (speed < 0.0f) pwm = -pwm;
    return pwm;
}

static float Controller_PID(float target,
                             float measured,
                             float dt_sec,
                             float *integral,
                             float *prevError)
{
    float error = target - measured;
    if (fabsf(target) < 1.0f)
    {
        *integral  = 0.0f;
        *prevError = 0.0f;
        return 0.0f;
    }
    *integral += error * dt_sec;
    float derivative = (error - *prevError) / dt_sec;
    *prevError = error;
    return (SPEED_KP * error) + (SPEED_KI * (*integral)) + (SPEED_KD * derivative);
}

void Controller_Init(void) {
	Controller_ComputeMinUsableSpeed();
	Controller_Reset(); }

void Controller_Reset(void)
{
    Encoder_Update();
    prevLeftCount  = Encoder_GetLeftCount();
    prevRightCount = Encoder_GetRightCount();
    leftIntegral   = 0.0f;
    rightIntegral  = 0.0f;
    leftPrevError  = 0.0f;
    rightPrevError = 0.0f;
    leftMeasuredSpeed  = 0.0f;
    rightMeasuredSpeed = 0.0f;
    leftPWM    = 0;
    rightPWM   = 0;
    hasStarted = 0;
    Motor_SetRawPWM(0, 0);
}

void Controller_UpdateSpeed(float targetLeft_mm_s,
                             float targetRight_mm_s,
                             float dt_sec,
                             uint8_t isDecelerating,
                             float targetSpeed_mm_s,
                             float maxCruiseSpeed_mm_s,
                             float totalDist_mm)
{
    // Startup gate
    if (!hasStarted)
    {
        if (fabsf(targetLeft_mm_s)  < minUsableSpeed_mm_s &&
            fabsf(targetRight_mm_s) < minUsableSpeed_mm_s)
        {
            leftPWM  = 0;
            rightPWM = 0;
            Motor_SetRawPWM(0, 0);
            return;
        }
        else
        {
            hasStarted = 1;
        }
    }

    Encoder_Update();

    int32_t currentLeftCount  = Encoder_GetLeftCount();
    int32_t currentRightCount = Encoder_GetRightCount();
    int32_t dLeft  = currentLeftCount  - prevLeftCount;
    int32_t dRight = currentRightCount - prevRightCount;
    prevLeftCount  = currentLeftCount;
    prevRightCount = currentRightCount;

    leftMeasuredSpeed  = ((float)dLeft  / dt_sec) * MM_PER_TICK;
    rightMeasuredSpeed = ((float)dRight / dt_sec) * MM_PER_TICK;

    float leftFeedforward  = Controller_LookupLeftPWM(targetLeft_mm_s);
    float rightFeedforward = Controller_LookupRightPWM(targetRight_mm_s);

    // Average feedforward below 700mm/s during startup ramp
//    if (fabsf(targetLeft_mm_s)  < 700.0f &&
//        fabsf(targetRight_mm_s) < 700.0f)
//    {
//        float avgFF = 0.5f * (leftFeedforward + rightFeedforward);
//        leftFeedforward  = avgFF;
//        rightFeedforward = avgFF;
//    }

    float leftCorrection =
        Controller_PID(targetLeft_mm_s,  leftMeasuredSpeed,
                       dt_sec, &leftIntegral,  &leftPrevError);
    float rightCorrection =
        Controller_PID(targetRight_mm_s, rightMeasuredSpeed,
                       dt_sec, &rightIntegral, &rightPrevError);

    // ── Proportional decel bias ───────────────────────────────────────────
    // Right motor is sluggish during deceleration — runs faster than left
    // for the same PWM. The gap grows as speed drops so we scale the bias
    // stronger at lower speeds. Works for all speeds and distances.
    float decelBias = 0.0f;
	#if ENABLE_DECEL_BIAS
		if (isDecelerating)
		{
			float speedDiff = rightMeasuredSpeed - leftMeasuredSpeed;
			if (speedDiff > 0.0f)
			{
				float sf = 1.0f - (targetSpeed_mm_s / maxCruiseSpeed_mm_s);
				if (sf < 0.0f) sf = 0.0f;
				if (sf > 1.0f) sf = 1.0f;
				float speedFactor = sqrtf(sf);

				// Logarithmic distance scaling — fitted to experimental data:
				// 254mm→15, 914mm→22, 3000mm→29
				float distCoeff = 5.670f * logf(totalDist_mm) - 16.40f;
				if (distCoeff <  3.0f) distCoeff =  3.0f;
				if (distCoeff > 35.0f) distCoeff = 35.0f;

				// Speed scaling — reduce coefficient as speed increases above 800mm/s
				// Every 100mm/s above 800 reduces coefficient by 1
				float speedPenalty = (maxCruiseSpeed_mm_s - 800.0f) / 100.0f;
				if (speedPenalty < 0.0f) speedPenalty = 0.0f;
				distCoeff -= speedPenalty;
				if (distCoeff < 3.0f) distCoeff = 3.0f;

				decelBias = speedDiff * (0.3f + speedFactor * distCoeff);
				if (decelBias > 500.0f) decelBias = 500.0f;
			}
		}
	#endif
    // ─────────────────────────────────────────────────────────────────────

    leftPWM  = Controller_ClampPWM(leftFeedforward  + leftCorrection);
    rightPWM = Controller_ClampPWM(rightFeedforward + rightCorrection - decelBias);

    Motor_SetRawPWM(leftPWM, rightPWM);
}

float Controller_GetLeftMeasuredSpeed(void)  { return leftMeasuredSpeed; }
float Controller_GetRightMeasuredSpeed(void) { return rightMeasuredSpeed; }
int16_t Controller_GetLeftPWM(void)          { return leftPWM; }
int16_t Controller_GetRightPWM(void)         { return rightPWM; }
