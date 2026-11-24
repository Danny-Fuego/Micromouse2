#include "motion_controller.h"
#include "encoder.h"
#include "motor.h"
#include "bno055.h"
#include "battery.h"
#include <math.h>

#define SPEED_FF   0.00477f
#define BIAS_FF    3.55f
#define ACC_FF     0.00095f

#define MOUSE_RADIUS_MM   39.35f
#define DEG_TO_RAD        0.0174533f

#define MAX_MOTOR_VOLTS  9.0f

#define FWD_KP  2.0f
#define FWD_KD  0.01f

#define ROT_KP  0.13f
#define ROT_KD  4.5f

#define MOTOR_MAX_PWM  1000

#define STEERING_KP           8.0f
#define STEERING_KD           1000.0f
#define STEERING_ADJUST_LIMIT 30.0f
#define SIDE_NOMINAL_IN       3.0f

#define MIN_SPEED_FOR_ANGLE_MM_S  50.0f

// Real IR background-scan refresh rate — one full left+front+right cycle
// takes ~9ms (9 state-machine steps at ~1ms each), NOT 1ms. Using this
// instead of LOOP_FREQUENCY_HZ fixes the original derivative-timing bug:
// the rate is now computed against the true elapsed time between fresh
// readings, since this function is only ever called once per real cycle.
#define IR_REFRESH_RATE_HZ  111.0f

static float lastAngleEstimateDeg = 0.0f;
static float lastCrossTrackError = 0.0f;
static float lastPTerm = 0.0f;
static float lastDTerm = 0.0f;
static float lastSteeringAdjustment = 0.0f;

static uint8_t outputEnabled = 1;
static uint8_t feedforwardEnabled = 1;

static float fwdError = 0.0f;
static float prevFwdError = 0.0f;
static float rotError = 0.0f;
static float prevRotError = 0.0f;

static float oldLeftSpeed = 0.0f;
static float oldRightSpeed = 0.0f;

static int32_t prevLeftCount = 0;
static int32_t prevRightCount = 0;

static uint8_t headingInitialized = 0;
static float prevHeading = 0.0f;

static float lastLeftVolts = 0.0f;
static float lastRightVolts = 0.0f;
static float lastPosOutput = 0.0f;
static float lastRotOutput = 0.0f;

float MotionController_GetLastLeftVolts(void)  { return lastLeftVolts; }
float MotionController_GetLastRightVolts(void) { return lastRightVolts; }
float MotionController_GetLastPosOutput(void)  { return lastPosOutput; }
float MotionController_GetLastRotOutput(void)  { return lastRotOutput; }
float MotionController_GetLastRotError(void)   { return rotError; }

void MotionController_Init(void)
{
    MotionController_Reset();
}

void MotionController_Reset(void)
{
    fwdError = 0.0f;
    prevFwdError = 0.0f;
    rotError = 0.0f;
    prevRotError = 0.0f;
    oldLeftSpeed = 0.0f;
    oldRightSpeed = 0.0f;
    prevLeftCount = Encoder_GetLeftCount();
    prevRightCount = Encoder_GetRightCount();
    headingInitialized = 0;
}

static float ClampVolts(float volts)
{
    if (volts > MAX_MOTOR_VOLTS) return MAX_MOTOR_VOLTS;
    if (volts < -MAX_MOTOR_VOLTS) return -MAX_MOTOR_VOLTS;
    return volts;
}

static int16_t ClampPWM(int pwm)
{
    if (pwm > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
    if (pwm < -MOTOR_MAX_PWM) return -MOTOR_MAX_PWM;
    return (int16_t)pwm;
}

static int PwmCompensated(float desired_voltage, float battery_voltage)
{
    if (battery_voltage < 0.5f)
    {
        battery_voltage = 0.5f;
    }
    float pwm = (float)MOTOR_MAX_PWM * desired_voltage / battery_voltage;
    return (int)pwm;
}

static float LeftFeedForward(float speed)
{
    float ff = speed * SPEED_FF;
    if (speed > 0.0f) ff += BIAS_FF;
    else if (speed < 0.0f) ff -= BIAS_FF;

    float acc = (speed - oldLeftSpeed) * LOOP_FREQUENCY_HZ;
    oldLeftSpeed = speed;
    ff += ACC_FF * acc;

    // Prevent the acceleration term from pushing feedforward output in the
    // direction opposite to the commanded speed. During a reverse-to-stop
    // transition, speed rises from negative toward 0, so acc is naturally
    // positive — without this guard, that alone can push ff positive,
    // commanding real forward motor voltage while the robot was moving in
    // reverse the whole time. No-op whenever speed and ff already agree
    // in sign, so normal forward behavior is unaffected.
    if (speed < 0.0f && ff > 0.0f) ff = 0.0f;
    if (speed > 0.0f && ff < 0.0f) ff = 0.0f;

    return ff;
}

static float RightFeedForward(float speed)
{
    float ff = speed * SPEED_FF;
    if (speed > 0.0f) ff += BIAS_FF;
    else if (speed < 0.0f) ff -= BIAS_FF;

    float acc = (speed - oldRightSpeed) * LOOP_FREQUENCY_HZ;
    oldRightSpeed = speed;
    ff += ACC_FF * acc;

    if (speed < 0.0f && ff > 0.0f) ff = 0.0f;
    if (speed > 0.0f && ff < 0.0f) ff = 0.0f;

    return ff;
}

void MotionController_SetOutputEnabled(uint8_t enabled)
{
    outputEnabled = enabled;
}

void MotionController_SetFeedforwardEnabled(uint8_t enabled)
{
    feedforwardEnabled = enabled;
}

void MotionController_Update(float forwardTargetSpeed_mm_s,
                              float forwardTargetPos_mm,
                              float rotationTargetSpeed_deg_s,
                              float rotationTargetPos_deg,
                              float currentHeading_deg,
                              float steeringAdjustment_deg,
                              uint8_t robotIsMoving)
{
    (void)forwardTargetPos_mm;
    (void)rotationTargetPos_deg;

    int32_t leftCount = Encoder_GetLeftCount();
    int32_t rightCount = Encoder_GetRightCount();

    int32_t deltaLeft = leftCount - prevLeftCount;
    int32_t deltaRight = rightCount - prevRightCount;
    prevLeftCount = leftCount;
    prevRightCount = rightCount;

    float leftChange_mm  = (float)deltaLeft  * MM_PER_TICK_LEFT;
    float rightChange_mm = (float)deltaRight * MM_PER_TICK_RIGHT;
    float fwdChange_mm = 0.5f * (leftChange_mm + rightChange_mm);

    float fwdIncrement = forwardTargetSpeed_mm_s * LOOP_INTERVAL_SEC;
    fwdError += fwdIncrement - fwdChange_mm;
    float fwdDiff = fwdError - prevFwdError;
    prevFwdError = fwdError;
    float posOutput = FWD_KP * fwdError + FWD_KD * fwdDiff;

    if (!headingInitialized)
    {
        prevHeading = currentHeading_deg;
        headingInitialized = 1;
    }

    float headingChange_deg = BNO055_AngleError(currentHeading_deg, prevHeading);
    prevHeading = currentHeading_deg;

    float rotIncrement = rotationTargetSpeed_deg_s * LOOP_INTERVAL_SEC;
    rotError += rotIncrement - headingChange_deg + (steeringAdjustment_deg * LOOP_INTERVAL_SEC);

    while (rotError > 180.0f) rotError -= 360.0f;
    while (rotError < -180.0f) rotError += 360.0f;

    float rotDiff = rotError - prevRotError;
    prevRotError = rotError;

    // rotOutput only applies while a motion primitive has explicitly declared
    // the robot moving (g_robotIsMoving, set live by Run_PositionControlTest
    // and Run_SpinTurnTest). rotError itself keeps accumulating regardless —
    // this only suppresses the ACT of correcting once the robot is genuinely,
    // physically still, so real drift is still picked up by the next move.
    float rotOutput = robotIsMoving ? (ROT_KP * rotError + ROT_KD * rotDiff) : 0.0f;

    float leftVolts  = posOutput + rotOutput;
    float rightVolts = posOutput - rotOutput;

    float tangentSpeed = rotationTargetSpeed_deg_s * MOUSE_RADIUS_MM * DEG_TO_RAD;
    float leftSpeedTarget  = forwardTargetSpeed_mm_s + tangentSpeed;
    float rightSpeedTarget = forwardTargetSpeed_mm_s - tangentSpeed;

    if (feedforwardEnabled)
    {
        leftVolts  += LeftFeedForward(leftSpeedTarget);
        rightVolts += RightFeedForward(rightSpeedTarget);
    }

    leftVolts  = ClampVolts(leftVolts);
    rightVolts = ClampVolts(rightVolts);

    lastPosOutput = posOutput;
    lastRotOutput = rotOutput;

    if (outputEnabled)
    {
        float batteryVolts = Battery_ReadVoltage();

        int16_t leftPWM  = ClampPWM(PwmCompensated(leftVolts, batteryVolts));
        int16_t rightPWM = ClampPWM(PwmCompensated(rightVolts, batteryVolts));

        Motor_SetRawPWM(leftPWM, rightPWM);
    }

    lastLeftVolts = leftVolts;
    lastRightVolts = rightVolts;
}

// Called once per fresh IR scan cycle (~9ms), from sensors.c. Recomputes
// crossTrackError and the derivative against the REAL refresh interval —
// this is the only place these values change.
void MotionController_OnFreshIRReading(float leftDist_in, float rightDist_in,
                                         uint8_t seeLeft, uint8_t seeRight,
                                         float forwardSpeed_mm_s)
{
    float crossTrackError = 0.0f;

    if (seeLeft && seeRight)
    {
        crossTrackError = rightDist_in - leftDist_in;
    }
    else if (seeLeft)
    {
        crossTrackError = 2.0f * (SIDE_NOMINAL_IN - leftDist_in);
    }
    else if (seeRight)
    {
        crossTrackError = -2.0f * (SIDE_NOMINAL_IN - rightDist_in);
    }

    lastPTerm = STEERING_KP * crossTrackError;

    float rawRate = (crossTrackError - lastCrossTrackError) * IR_REFRESH_RATE_HZ;

    float absSpeed = fabsf(forwardSpeed_mm_s);

    if (absSpeed > MIN_SPEED_FOR_ANGLE_MM_S)
    {
        float ratio = rawRate / absSpeed;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;
        float angleEstimate_rad = asinf(ratio);
        lastDTerm = STEERING_KD * angleEstimate_rad;

        lastAngleEstimateDeg = angleEstimate_rad * (180.0f / 3.14159265f);
    }
    else
    {
        lastDTerm = 0.0f;
        lastAngleEstimateDeg = 0.0f;
    }

    lastCrossTrackError = crossTrackError;
}

// Cheap, called every SysTick tick — just combines whatever the last fresh
// IR reading produced. No new sensing or math here.
float MotionController_CalculateSteeringAdjustment(void)
{
    float adjustment = lastPTerm + lastDTerm;

    if (adjustment > STEERING_ADJUST_LIMIT) adjustment = STEERING_ADJUST_LIMIT;
    if (adjustment < -STEERING_ADJUST_LIMIT) adjustment = -STEERING_ADJUST_LIMIT;

    lastSteeringAdjustment = adjustment;

    return adjustment;
}

void MotionController_ResetSteering(void)
{
    lastCrossTrackError = 0.0f;
    lastPTerm = 0.0f;
    lastDTerm = 0.0f;
    lastSteeringAdjustment = 0.0f;
}

float MotionController_GetLastSteeringAdjustment(void) { return lastSteeringAdjustment; }
float MotionController_GetLastAngleEstimateDeg(void) { return lastAngleEstimateDeg; }
