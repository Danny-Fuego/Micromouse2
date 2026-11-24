#include "calibration.h"
#include "turn.h"
#include "sensors.h"
#include "ramp_profile.h"
#include "motion_controller.h"
#include "motion_state.h"
#include "battery.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

#define TEST_ENCODER_ONLY 0
#define TEST_MOTOR_ENCODER_FOREVER 0
#define TEST_BNO055_ONLY 0
#define TEST_BNO055_WITH_MOTOR 0
#define TEST_BLUETOOTH_ONLY 0
#define TEST_ENCODER_CPR_CALIB 0
#define TEST_MOTOR_SPEED_CALIB 0
#define TEST_DEADZONE 0
#define TEST_TURN_SIGN 0
#define TEST_TURN 0
#define TEST_TURN_PROFILE_CALIB 0
#define TEST_IR_LIVE 0
#define TEST_IR_BLUETOOTH 0
#define TEST_IR_CALIB 0
#define TEST_IR_STATUS 0
#define TEST_IR_LIVE_MONITOR 0
#define TEST_POSITION_CONTROL 0
#define TEST_BATTERY_VOLTAGE 0
#define TEST_VOLTAGE_SWEEP 0
#define TEST_ACC_FF 0
#define TEST_BNO_ASYNC 0
#define TEST_STEP2 0
#define TEST_GYRO_STEP3 0
#define TEST_STEP4 0
#define TEST_BNO_SYSTICK_WATCH 0
#define TEST_HEADING_DIRECTION 0
#define TEST_ROTATION_SWEEP 0
#define TEST_GYRO_STEP2 0
#define TEST_BNO_CYCLE_WATCH 0
#define TEST_BNO_ALTERNATE 0
#define TEST_SPIN_TURN 0
#define TEST_ROTATION_DECEL 0
#define TEST_STRAIGHT_TURN_STRESS 0
#define TEST_STEERING_ADJUSTMENT 0
#define TEST_SYSTICK_TIMING 0
#define TEST_FLASHLIGHT_RECEIVER 0
#define TEST_TEN_INCH_LOOP 0
#define TEST_TEN_TURN_LOOP 0
#define TEST_MOVE_THEN_TURN 0
#define TEST_EMITTER_HOLD_ON 0
#define TEST_MOVE_THEN_RESQUARE 0
#define TEST_FAST_HANDOFF 0
#define TEST_SMOOTH_TURN 0
#define TEST_PATH 1
#define TEST_SMOOTH_TURN_DETAIL 0
#define TEST_PATH_LOOP 0
#define TEST_SQUARE 0

// 1 = IR steering is switched off while an arc is running (rotation profile not finished). 0 = old behaviour.
// Needs g_irSteerOffInArc defined in the project (SysTick edit in stm32f4xx_it.c).
#define IR_OFF_IN_ARC 1
extern volatile uint8_t g_irSteerOffInArc;



#define BNO_TEST_MODE_NDOF 0
#define BNO_TEST_MODE_IMUPLUS 0

#define CONTROL_DT_MS 10
#define CONTROL_DT_SEC 0.010f
#define TEST_DURATION_MS 3000
#define VOLTAGE_SWEEP_STEP 0.25f
#define VOLTAGE_SWEEP_MAX 9.0f
#define VOLTAGE_SWEEP_HOLD_MS 1500
#define VOLTAGE_SWEEP_SAMPLE_MS 500
#define ACC_FF_STEP_VOLTAGE 6.0f
#define ACC_FF_LOG_DURATION_MS 500
#define ACC_FF_LOG_INTERVAL_MS 15
#define ACC_FF_MAX_SAMPLES 500
#define ROT_SWEEP_VOLTAGE_STEP    0.25f
#define ROT_SWEEP_VOLTAGE_MAX     9.00f
#define ROT_SWEEP_HOLD_MS         1000
#define ROT_SWEEP_SAMPLE_MS       300
#define ROT_DECEL_TEST_VOLTAGE     6.0f
#define ROT_DECEL_TEST_HOLD_MS     800
#define ROT_DECEL_LOG_INTERVAL_MS  5
#define ROT_DECEL_LOG_DURATION_MS  1000
#define ROT_DECEL_MAX_SAMPLES      250
#define RESQUARE_OVERSHOOT_IN     0.2f
#define RESQUARE_BACKTRACK_IN     3.0f
#define RESQUARE_DRIVE_SPEED      800.0f
#define RESQUARE_REVERSE_SPEED    800.0f

typedef struct
{
    uint32_t time_ms;
    float omega_dps;
} RotDecelSample;

static RotDecelSample rotDecelSamples[ROT_DECEL_MAX_SAMPLES];
int16_t rawL;
int16_t rawR;
int32_t cntL;
int32_t cntR;

extern BNO055_Status bnoStatus;
BNO055_Euler bnoEuler;
BNO055_Calib bnoCalib;
BNO055_Gyro bnoGyro;

extern uint8_t bnoChipID;
uint8_t bnoSysStat = 0;
uint8_t bnoSysErr = 0;

float bnoHeading = 0.0f;
float bnoRoll = 0.0f;
float bnoPitch = 0.0f;
float bnoGz = 0.0f;

// IR live debug variables
volatile uint16_t dbg_irL = 0;
volatile uint16_t dbg_irF = 0;
volatile uint16_t dbg_irR = 0;
volatile float dbg_distL = 0.0f;
volatile float dbg_distF = 0.0f;
volatile float dbg_distR = 0.0f;
volatile uint8_t dbg_wallL = 0;
volatile uint8_t dbg_wallF = 0;
volatile uint8_t dbg_wallR = 0;
volatile uint8_t dbg_appL = 0;
volatile uint8_t dbg_appF = 0;
volatile uint8_t dbg_appR = 0;

typedef struct{
	uint32_t time_ms;
	float speed_mm_s;
} AccFFSample;

static void BT_SendLine(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 1000);
}

static void BT_Sendf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 1000);
}

static float AngleDiff(float now, float start)
{
    return BNO055_AngleError(now, start);
}

static void Run_BatteryVoltageTest(void)
{
    uint32_t lastPrintTime;
    uint32_t now;

    BT_SendLine("===== BATTERY VOLTAGE TEST =====");
    BT_SendLine("time_ms,voltage");

    lastPrintTime = HAL_GetTick();

    while (1)
    {
        now = HAL_GetTick();

        if ((now - lastPrintTime) >= 500)
        {
            BT_Sendf("%lu,%.3f\r\n", now, Battery_ReadVoltage());
            lastPrintTime = now;
        }
    }
}

static void Run_VoltageSweepOneMotor(const char *label, uint8_t isLeft){
	Encoder_ResetAll();

	BT_Sendf("\r\n===== VOLTAGE SWEEP %s ======\r\n", label);
	BT_SendLine("pass,voltage,measuredSpeed,batteryVoltage");

	for (float v = VOLTAGE_SWEEP_STEP; v <= VOLTAGE_SWEEP_MAX; v += VOLTAGE_SWEEP_STEP){
		int pwm = (int)(1000.0f * v / Battery_ReadVoltage());

		if (isLeft) Motor_SetRawPWM(pwm, 0);
		else        Motor_SetRawPWM(0, pwm);

		HAL_Delay(VOLTAGE_SWEEP_HOLD_MS - VOLTAGE_SWEEP_SAMPLE_MS);

		int32_t startCount = isLeft ? Encoder_GetLeftCount() : Encoder_GetRightCount();
		HAL_Delay(VOLTAGE_SWEEP_SAMPLE_MS);
		Encoder_Update();
		int32_t endCount = isLeft ? Encoder_GetLeftCount() : Encoder_GetRightCount();

		float mmPerTick = isLeft ? MM_PER_TICK_LEFT : MM_PER_TICK_RIGHT;
		float speed = ((float)(endCount - startCount) * mmPerTick) / (VOLTAGE_SWEEP_SAMPLE_MS / 1000.0f);

		BT_Sendf("UP,%.2f,%.2f,%.3f\r\n", v, speed, Battery_ReadVoltage());
	}

	for (float v = VOLTAGE_SWEEP_MAX; v >= 0.0f; v -=VOLTAGE_SWEEP_STEP){
		int pwm = (int)(1000.0f * v / Battery_ReadVoltage());

		if (isLeft) Motor_SetRawPWM(pwm, 0);
		else        Motor_SetRawPWM(0, pwm);

		HAL_Delay(VOLTAGE_SWEEP_HOLD_MS - VOLTAGE_SWEEP_SAMPLE_MS);

		int32_t startCount = isLeft ? Encoder_GetLeftCount() : Encoder_GetRightCount();
		HAL_Delay(VOLTAGE_SWEEP_SAMPLE_MS);
		Encoder_Update();
		int32_t endCount = isLeft ? Encoder_GetLeftCount() : Encoder_GetRightCount();

		float mmPerTick = isLeft ? MM_PER_TICK_LEFT : MM_PER_TICK_RIGHT;
		float speed = ((float)(endCount - startCount) * mmPerTick) / (VOLTAGE_SWEEP_SAMPLE_MS / 1000.0f);

		BT_Sendf("DOWN,%.2f,%.2f,%.3f\r\n", v, speed, Battery_ReadVoltage());
	}

	Motor_Stop(MOTOR_BRAKE);
	HAL_Delay(1500);
}

static void Run_VoltageSweepBothMotors(void)
{
    Encoder_ResetAll();

    BT_SendLine("\r\n===== VOLTAGE SWEEP BOTH MOTORS =====");
    BT_SendLine("pass,voltage,speedLeft,speedRight,batteryVoltage");

    for (float v = VOLTAGE_SWEEP_STEP; v <= VOLTAGE_SWEEP_MAX; v += VOLTAGE_SWEEP_STEP)
    {
        int pwm = (int)(1000.0f * v / Battery_ReadVoltage());
        Motor_SetRawPWM(pwm, pwm);

        HAL_Delay(VOLTAGE_SWEEP_HOLD_MS - VOLTAGE_SWEEP_SAMPLE_MS);

        int32_t startL = Encoder_GetLeftCount();
        int32_t startR = Encoder_GetRightCount();
        HAL_Delay(VOLTAGE_SWEEP_SAMPLE_MS);
        Encoder_Update();
        int32_t endL = Encoder_GetLeftCount();
        int32_t endR = Encoder_GetRightCount();

        float speedL = ((float)(endL - startL) * MM_PER_TICK_LEFT) / (VOLTAGE_SWEEP_SAMPLE_MS / 1000.0f);
        float speedR = ((float)(endR - startR) * MM_PER_TICK_RIGHT) / (VOLTAGE_SWEEP_SAMPLE_MS / 1000.0f);

        BT_Sendf("UP,%.2f,%.2f,%.2f,%.3f\r\n", v, speedL, speedR, Battery_ReadVoltage());
    }

    for (float v = VOLTAGE_SWEEP_MAX; v >= 0.0f; v -= VOLTAGE_SWEEP_STEP)
    {
        int pwm = (int)(1000.0f * v / Battery_ReadVoltage());
        Motor_SetRawPWM(pwm, pwm);

        HAL_Delay(VOLTAGE_SWEEP_HOLD_MS - VOLTAGE_SWEEP_SAMPLE_MS);

        int32_t startL = Encoder_GetLeftCount();
        int32_t startR = Encoder_GetRightCount();
        HAL_Delay(VOLTAGE_SWEEP_SAMPLE_MS);
        Encoder_Update();
        int32_t endL = Encoder_GetLeftCount();
        int32_t endR = Encoder_GetRightCount();

        float speedL = ((float)(endL - startL) * MM_PER_TICK_LEFT) / (VOLTAGE_SWEEP_SAMPLE_MS / 1000.0f);
        float speedR = ((float)(endR - startR) * MM_PER_TICK_RIGHT) / (VOLTAGE_SWEEP_SAMPLE_MS / 1000.0f);

        BT_Sendf("DOWN,%.2f,%.2f,%.2f,%.3f\r\n", v, speedL, speedR, Battery_ReadVoltage());
    }

    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(1500);
}

static void Run_VoltageSweepTest(void){
	BT_SendLine("===== VOLTAGE SWEEP TEST START ======");
	BT_Sendf("step=%.2fV max=%.2fV\r\n", VOLTAGE_SWEEP_STEP, VOLTAGE_SWEEP_MAX);

	Run_VoltageSweepOneMotor("LEFT", 1);
	Run_VoltageSweepOneMotor("RIGHT", 0);

	BT_SendLine("===== VOLTAGE SWEEP TEST DONE =====");
	while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

static AccFFSample accFFSamples[ACC_FF_MAX_SAMPLES];

static void Run_AccFFStepTest(const char *label, uint8_t isLeft){
	Encoder_ResetAll();
	Motor_Stop(MOTOR_BRAKE);
	HAL_Delay(500);

	BT_Sendf("\r\n==== ACC_FF STEP TEST %S =====\r\n", label);
	BT_Sendf("stepVoltage=%.2fV\r\n", ACC_FF_STEP_VOLTAGE);
	BT_SendLine("time_ms,speed_mm_s,batteryVoltage");

	int32_t prevCount = isLeft ? Encoder_GetLeftCount() : Encoder_GetRightCount();
	uint32_t startTime = HAL_GetTick();
	uint32_t lastSampleTime = startTime;
	uint32_t sampleCount = 0;

	// Command the step, right at t = 0
	int pwm = (int)(1000.0f * ACC_FF_STEP_VOLTAGE / Battery_ReadVoltage());
	if (isLeft) Motor_SetRawPWM(pwm, 0);
	else		Motor_SetRawPWM(0, pwm);

	while (((HAL_GetTick() - startTime) < ACC_FF_LOG_DURATION_MS) && (sampleCount < ACC_FF_MAX_SAMPLES)){
		uint32_t now = HAL_GetTick();
		uint32_t dt_ms = now - lastSampleTime;

		if (dt_ms >= ACC_FF_LOG_INTERVAL_MS){
			Encoder_Update();
			int32_t currCount = isLeft ? Encoder_GetLeftCount() : Encoder_GetRightCount();
			int32_t delta = currCount - prevCount;

			float mmPerTick = isLeft ? MM_PER_TICK_LEFT : MM_PER_TICK_RIGHT;
			float speed = ((float)delta * mmPerTick) / (dt_ms / 1000.0f);

			accFFSamples[sampleCount].time_ms = now - startTime;
			accFFSamples[sampleCount].speed_mm_s = speed;
			sampleCount++;

			prevCount = currCount;
			lastSampleTime = now;
		}
	}

	Motor_Stop(MOTOR_BRAKE);
	HAL_Delay(300);

	BT_Sendf("\r\n===== ACC_FF STEP TEST %S =====\r\n", label);
	BT_Sendf("stepVoltage=%.2fV samples=%lu\r\n", ACC_FF_STEP_VOLTAGE, sampleCount);
	BT_SendLine("time_ms,speed_mm_s");

	for (uint32_t i = 0; i < sampleCount; i++){
		BT_Sendf("%lu,%.2f\r\n", accFFSamples[i].time_ms, accFFSamples[i].speed_mm_s);
	}

	HAL_Delay(1500);
}

static void Run_AccFFTest(void){
	BT_SendLine("===== ACC_FF STEP RESPONSE TEST START =====");
	Run_AccFFStepTest("LEFT", 1);
	Run_AccFFStepTest("RIGHT", 0);
	BT_SendLine("===== ACC_FF STEP RESPONSE TEST DONE =====");
	while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

// ============================================================
// STILL-WORKING TESTS — unchanged from before, no dependency
// on the deleted motion_profile.c / speed_controller.c /
// straight_controller.c
// ============================================================

static void Run_TurnProfileCalibration(void)
{
    const int16_t pwmList[] = {300, 350, 400, 450, 500, 550, 600, 650, 700};
    const int pwmCount = sizeof(pwmList) / sizeof(pwmList[0]);
    float startHeading = 0.0f;
    float heading = 0.0f;
    float delta = 0.0f;
    BNO055_Gyro gyro;
    BNO055_Status st;
    uint32_t startTime;
    uint32_t now;
    uint32_t lastPrint;

    BT_SendLine("===== TURN PROFILE CALIBRATION READY =====");
    BT_SendLine("Place robot on ground. Press button.");
    Wait_For_Button();

    BT_SendLine("===== TEST 1: TURN BREAKAWAY PWM =====");
    BT_SendLine("pwm,dir,startHeading,endHeading,deltaHeading,moved");
    for (int i = 0; i < pwmCount; i++)
    {
        int16_t pwm = pwmList[i];
        BNO055_ReadHeading(&hi2c1, &startHeading);
        Motor_SetRawPWM(pwm, -pwm);
        HAL_Delay(350);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(250);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,RIGHT,%.2f,%.2f,%.2f,%d\r\n", pwm, startHeading, heading, delta, fabsf(delta) > 2.0f ? 1 : 0);
        HAL_Delay(800);

        BNO055_ReadHeading(&hi2c1, &startHeading);
        Motor_SetRawPWM(-pwm, pwm);
        HAL_Delay(350);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(250);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,LEFT,%.2f,%.2f,%.2f,%d\r\n", pwm, startHeading, heading, delta, fabsf(delta) > 2.0f ? 1 : 0);
        HAL_Delay(1000);
    }

    BT_SendLine("===== TEST 2: ANGULAR SPEED VS PWM =====");
    BT_SendLine("time_ms,pwm,dir,heading,gz");
    for (int i = 0; i < pwmCount; i++)
    {
        int16_t pwm = pwmList[i];
        startTime = HAL_GetTick();
        lastPrint = startTime;
        Motor_SetRawPWM(pwm, -pwm);
        while ((HAL_GetTick() - startTime) < 700)
        {
            now = HAL_GetTick();
            if ((now - lastPrint) >= 20)
            {
                BNO055_ReadHeading(&hi2c1, &heading);
                st = BNO055_ReadGyro(&hi2c1, &gyro);
                if (st == BNO055_OK)
                    BT_Sendf("%lu,%d,RIGHT,%.2f,%.2f\r\n", now - startTime, pwm, heading, gyro.z_dps);
                lastPrint = now;
            }
        }
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);

        startTime = HAL_GetTick();
        lastPrint = startTime;
        Motor_SetRawPWM(-pwm, pwm);
        while ((HAL_GetTick() - startTime) < 700)
        {
            now = HAL_GetTick();
            if ((now - lastPrint) >= 20)
            {
                BNO055_ReadHeading(&hi2c1, &heading);
                st = BNO055_ReadGyro(&hi2c1, &gyro);
                if (st == BNO055_OK)
                    BT_Sendf("%lu,%d,LEFT,%.2f,%.2f\r\n", now - startTime, pwm, heading, gyro.z_dps);
                lastPrint = now;
            }
        }
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);
    }

    BT_SendLine("===== TEST 3: TURN BRAKE DISTANCE =====");
    BT_SendLine("pwm,dir,gzAtBrake,headingAtBrake,finalHeading,extraDeg");
    for (int i = 0; i < pwmCount; i++)
    {
        int16_t pwm = pwmList[i];
        Motor_SetRawPWM(pwm, -pwm);
        HAL_Delay(350);
        BNO055_ReadHeading(&hi2c1, &startHeading);
        BNO055_ReadGyro(&hi2c1, &gyro);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(400);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,RIGHT,%.2f,%.2f,%.2f,%.2f\r\n", pwm, gyro.z_dps, startHeading, heading, delta);
        HAL_Delay(1200);

        Motor_SetRawPWM(-pwm, pwm);
        HAL_Delay(350);
        BNO055_ReadHeading(&hi2c1, &startHeading);
        BNO055_ReadGyro(&hi2c1, &gyro);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(400);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,LEFT,%.2f,%.2f,%.2f,%.2f\r\n", pwm, gyro.z_dps, startHeading, heading, delta);
        HAL_Delay(1200);
    }

    Motor_Stop(MOTOR_BRAKE);
    BT_SendLine("===== TURN PROFILE CALIBRATION DONE =====");
    while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

static void Run_MotorSpeedTest(int16_t pwmL, int16_t pwmR)
{
    int32_t prevL = 0, prevR = 0, currL = 0, currR = 0, dL = 0, dR = 0;
    float speedL = 0.0f, speedR = 0.0f;
    uint32_t startTime, lastTime, now, dt_ms;
    float dt_sec;

    Encoder_ResetAll();
    Encoder_Update();
    prevL = Encoder_GetLeftCount();
    prevR = Encoder_GetRightCount();
    BT_Sendf("\r\n===== PWM TEST L:%d R:%d =====\r\n", pwmL, pwmR);
    BT_SendLine("time_ms,dt_ms,pwmL,pwmR,cntL,cntR,dL,dR,speedL_mm_s,speedR_mm_s");
    Motor_SetRawPWM(pwmL, pwmR);
    startTime = HAL_GetTick();
    lastTime = startTime;

    while ((HAL_GetTick() - startTime) < TEST_DURATION_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastTime;
        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;
            Encoder_Update();
            currL = Encoder_GetLeftCount();
            currR = Encoder_GetRightCount();
            dL = currL - prevL;
            dR = currR - prevR;
            speedL = (dL / dt_sec) * MM_PER_TICK_LEFT;
            speedR = (dR / dt_sec) * MM_PER_TICK_RIGHT;
            BT_Sendf("%lu,%lu,%d,%d,%ld,%ld,%ld,%ld,%.2f,%.2f\r\n",
                     now - startTime, dt_ms, pwmL, pwmR, currL, currR, dL, dR, speedL, speedR);
            prevL = currL;
            prevR = currR;
            lastTime = now;
        }
    }
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(100);
    HAL_Delay(1000);
}

static void Run_DeadzoneTest(int16_t pwm)
{
    Encoder_ResetAll();
    Encoder_Update();
    BT_Sendf("\r\n===== DEADZONE TEST PWM:%d =====\r\n", pwm);
    int32_t startL = Encoder_GetLeftCount();
    int32_t startR = Encoder_GetRightCount();
    Motor_SetRawPWM(pwm, pwm);
    HAL_Delay(2000);
    Motor_SetRawPWM(0, 0);
    Encoder_Update();
    int32_t endL = Encoder_GetLeftCount();
    int32_t endR = Encoder_GetRightCount();
    BT_Sendf("LeftTicks:%ld RightTicks:%ld\r\n", endL - startL, endR - startR);
}

static void Run_TurnSignTest(void)
{
    float startHeading = 0.0f, endHeading = 0.0f, delta = 0.0f;
    BT_SendLine("===== TURN SIGN TEST READY =====");
    BT_SendLine("Place robot on ground. Press button.");
    Wait_For_Button();

    BNO055_ReadHeading(&hi2c1, &startHeading);
    BT_SendLine("===== TEST 1: Motor_SetRawPWM(600, -600) =====");
    BT_Sendf("startHeading: %.2f\r\n", startHeading);
    Motor_SetRawPWM(600, -600);
    HAL_Delay(1000);
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(300);
    BNO055_ReadHeading(&hi2c1, &endHeading);
    delta = BNO055_AngleError(endHeading, startHeading);
    BT_Sendf("endHeading: %.2f  delta: %.2f\r\n", endHeading, delta);
    HAL_Delay(1500);

    BNO055_ReadHeading(&hi2c1, &startHeading);
    BT_SendLine("===== TEST 2: Motor_SetRawPWM(-600, 600) =====");
    BT_Sendf("startHeading: %.2f\r\n", startHeading);
    Motor_SetRawPWM(-600, 600);
    HAL_Delay(1000);
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(300);
    BNO055_ReadHeading(&hi2c1, &endHeading);
    delta = BNO055_AngleError(endHeading, startHeading);
    BT_Sendf("endHeading: %.2f  delta: %.2f\r\n", endHeading, delta);

    BT_SendLine("===== TURN SIGN TEST DONE =====");
    while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

// ============================================================
// NEW TEST — exercises the RampProfile + MotionController
// architecture directly. Straight-line only (rotationProfile
// is not triggered).
// ============================================================
static float s_startHeadingDeg = 0.0f;
static float s_finalHeadingDeg = 0.0f;
static float s_residualHeadingErrorDeg = 0.0f;

static float NormalizeAngleDifference(float angleDeg)
{
    while (angleDeg > 180.0f)  angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

static void Run_PositionControlTest(float distance_mm, float maxSpeed)
{
    g_rotationBraked = 0;
    g_robotIsMoving = 1;

    uint32_t startTime, lastPrintTime, now;
    int32_t startCntL, startCntR;
    int32_t prevCntL, prevCntR;
    uint32_t stillTicks = 0;
    const uint32_t STILL_TICKS_REQUIRED = 3;

    Encoder_ResetAll();
    Encoder_Update();
    startCntL = Encoder_GetLeftCount();
    startCntR = Encoder_GetRightCount();
    prevCntL = startCntL;
    prevCntR = startCntR;

    MotionController_Reset();

    s_startHeadingDeg = g_currentHeading;

    BT_Sendf("startHeading=%.2f\r\n", s_startHeadingDeg);

    RampProfile_SetMove(&forwardProfile, distance_mm, maxSpeed);
    g_motionControlEnabled = 1;

    startTime = HAL_GetTick();
    lastPrintTime = startTime - 200;

    BT_Sendf("\r\n===== POSITION CONTROL TEST dist=%.1f maxSpeed=%.1f =====\r\n",
             distance_mm, maxSpeed);
    BT_SendLine("time_ms,targetPos,targetSpeed,isDecel,measPos,posError,heading,rotError,posOutput,rotOutput,leftVolts,rightVolts,cntL,cntR,distL,distR,seeL,seeR,steerAdj,kdAngleDeg");

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

            BT_Sendf("%lu,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%.2f,%.2f,%u,%u,%.3f,%.3f\r\n",
                     now - startTime, targetPos, RampProfile_GetSpeed(&forwardProfile),
                     RampProfile_IsDecelerating(&forwardProfile), measPos, posError,
                     g_currentHeading, MotionController_GetLastRotError(),
                     MotionController_GetLastPosOutput(), MotionController_GetLastRotOutput(),
                     MotionController_GetLastLeftVolts(), MotionController_GetLastRightVolts(),
                     cntL, cntR,
                     irDistL, irDistR, irSeeL, irSeeR, steerAdj, kdAngleDeg);

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

    BT_Sendf("finalHeading=%.2f residualError=%.2f\r\n",
             s_finalHeadingDeg, s_residualHeadingErrorDeg);
    BT_SendLine("===== POSITION CONTROL TEST DONE =====");
}

static void Run_Step4Test(void)
{
    BT_SendLine("===== STEP 4: async read inside SysTick, during real position control =====");

    Run_PositionControlTest(1000.0f, 800.0f);

    BT_Sendf("After test — last callback fired flag: %u\r\n", BNO055_Step2_DidCallbackFire());

    // try one more manual read now, to see if the mechanism still works post-test
    BNO055_Status st = BNO055_Step3_StartRead(&hi2c1);
    HAL_Delay(100);
    BT_Sendf("Post-test manual attempt — start: %d, fired: %u\r\n",
             st, BNO055_Step2_DidCallbackFire());

    while (1) { HAL_Delay(1000); }
}

static void Run_BnoSysTickWatchTest(void)
{
    BT_SendLine("===== BNO SYSTICK WATCH TEST =====");
    BT_SendLine("Move the robot by hand. Watching g_currentHeading via SysTick only.");
    BT_SendLine("Running forever - reset the board to stop.");
    BT_SendLine("time_ms,heading,attempts,failures,callbackFires,errors,recoveries");

    uint32_t startTime = HAL_GetTick();
    uint32_t lastPrint = startTime;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 200)
        {
        	BT_Sendf("%lu,%.2f,%lu,%lu,%lu,%lu,%lu,tickCount=%lu\r\n",
        	         now - startTime,
        	         g_currentHeading,
        	         BNO055_GetAsyncStartAttempts(),
        	         BNO055_GetAsyncStartFailures(),
        	         BNO055_GetAsyncCallbackFires(),
        	         BNO055_GetAsyncErrorCount(),
        	         BNO055_GetAsyncRecoveryCount(),
        	         g_systickTickCount);
            lastPrint = now;
        }
    }
}

static void Run_HeadingDirectionTest(void)
{
    BT_SendLine("===== HEADING DIRECTION TEST =====");
    BT_SendLine("Rotate the robot CLOCKWISE (viewed from above) by hand.");
    BT_SendLine("Watch whether heading goes UP or DOWN.");
    BT_SendLine("time_ms,heading");

    uint32_t startTime = HAL_GetTick();
    uint32_t lastPrint = startTime;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 200)
        {
            BT_Sendf("%lu,%.2f\r\n", now - startTime, g_currentHeading);
            lastPrint = now;
        }
    }
}

static void Run_RotationSweepOneDirection(const char *label, TurnDirection direction)
{
    BT_Sendf("\r\n===== ROTATION SWEEP %s =====\r\n", label);
    BT_SendLine("pass,voltage,measuredOmega_dps,batteryVoltage");

    for (float v = ROT_SWEEP_VOLTAGE_STEP; v <= ROT_SWEEP_VOLTAGE_MAX; v += ROT_SWEEP_VOLTAGE_STEP)
    {
        int pwm = (int)(1000.0f * v / Battery_ReadVoltage());
        int16_t leftPWM  = (direction == TURN_RIGHT) ?  pwm : -pwm;
        int16_t rightPWM = (direction == TURN_RIGHT) ? -pwm :  pwm;
        Motor_SetRawPWM(leftPWM, rightPWM);

        HAL_Delay(ROT_SWEEP_HOLD_MS - ROT_SWEEP_SAMPLE_MS);

        float sumOmega = 0.0f;
        int sampleCount = 0;
        uint32_t sampleStart = HAL_GetTick();
        while ((HAL_GetTick() - sampleStart) < ROT_SWEEP_SAMPLE_MS)
        {
            sumOmega += fabsf(g_currentGyroZ);
            sampleCount++;
            HAL_Delay(20);
        }

        float avgOmega = (sampleCount > 0) ? (sumOmega / sampleCount) : 0.0f;

        BT_Sendf("UP,%.2f,%.2f,%.3f\r\n", v, avgOmega, Battery_ReadVoltage());
    }

    for (float v = ROT_SWEEP_VOLTAGE_MAX; v >= 0.0f; v -= ROT_SWEEP_VOLTAGE_STEP)
    {
        int pwm = (int)(1000.0f * v / Battery_ReadVoltage());
        int16_t leftPWM  = (direction == TURN_RIGHT) ?  pwm : -pwm;
        int16_t rightPWM = (direction == TURN_RIGHT) ? -pwm :  pwm;
        Motor_SetRawPWM(leftPWM, rightPWM);

        HAL_Delay(ROT_SWEEP_HOLD_MS - ROT_SWEEP_SAMPLE_MS);

        float sumOmega = 0.0f;
        int sampleCount = 0;
        uint32_t sampleStart = HAL_GetTick();
        while ((HAL_GetTick() - sampleStart) < ROT_SWEEP_SAMPLE_MS)
        {
            sumOmega += fabsf(g_currentGyroZ);
            sampleCount++;
            HAL_Delay(20);
        }

        float avgOmega = (sampleCount > 0) ? (sumOmega / sampleCount) : 0.0f;

        BT_Sendf("DOWN,%.2f,%.2f,%.3f\r\n", v, avgOmega, Battery_ReadVoltage());
    }

    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(1500);
}

static void Run_RotationSweepTest(void)
{
    BT_SendLine("===== ROTATION SWEEP TEST START =====");
    Run_RotationSweepOneDirection("RIGHT", TURN_RIGHT);
    Run_RotationSweepOneDirection("LEFT", TURN_LEFT);
    BT_SendLine("===== ROTATION SWEEP TEST DONE =====");
    while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

static void Run_GyroStep2Test(void)
{
    BT_SendLine("===== GYRO STEP 2: single async read, no motors =====");

    BNO055_Status st = BNO055_StartAsyncGyroRead(&hi2c1);
    BT_Sendf("Start returned: %d\r\n", st);

    uint32_t startWait = HAL_GetTick();
    while (!BNO055_AsyncGyroReady() && (HAL_GetTick() - startWait) < 500) {}

    BT_Sendf("Ready: %u, gyroZ=%.2f, elapsed=%lums\r\n",
             BNO055_AsyncGyroReady(),
             BNO055_GetAsyncGyroZ(),
             HAL_GetTick() - startWait);

    while (1) { HAL_Delay(1000); }
}

static void Run_BnoCycleWatchTest(void)
{
    BT_SendLine("===== BNO CYCLE WATCH TEST =====");
    BT_SendLine("Watching heading + gyro cycling via SysTick. Reset to stop.");
    BT_SendLine("time_ms,heading,gyroZ,headingAttempts,headingFailures,headingCallbacks,gyroErrors,recoveries");

    uint32_t startTime = HAL_GetTick();
    uint32_t lastPrint = startTime;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 200)
        {
            BT_Sendf("%lu,%.2f,%.2f,%lu,%lu,%lu,%lu,%lu\r\n",
                     now - startTime,
                     g_currentHeading,
                     g_currentGyroZ,
                     BNO055_GetAsyncStartAttempts(),
                     BNO055_GetAsyncStartFailures(),
                     BNO055_GetAsyncCallbackFires(),
                     BNO055_GetAsyncErrorCount(),
                     BNO055_GetAsyncRecoveryCount());
            lastPrint = now;
        }
    }
}

static void Run_GyroStep3Test(void)
{
    BT_SendLine("===== GYRO STEP 3: repeated async reads =====");
    BT_SendLine("attempt,startStatus,ready,elapsedMs");

    for (int i = 1; i <= 50; i++)
    {
        BNO055_Status st = BNO055_StartAsyncGyroRead(&hi2c1);
        uint32_t start = HAL_GetTick();
        while (!BNO055_AsyncGyroReady() && (HAL_GetTick() - start) < 500) {}
        uint32_t elapsed = HAL_GetTick() - start;

        BT_Sendf("%d,%d,%u,%lu\r\n", i, st, BNO055_AsyncGyroReady(), elapsed);

        HAL_Delay(50);
    }

    BT_SendLine("===== GYRO STEP 3 DONE =====");
    while (1) { HAL_Delay(1000); }
}

static void Run_BnoAlternateTest(void)
{
    BT_SendLine("===== BNO ALTERNATE TEST (no SysTick) =====");
    BT_SendLine("cycle,headingStatus,headingReady,gyroStatus,gyroReady,heading,gyroZ");

    for (int i = 1; i <= 30; i++)
    {
        BNO055_Status hSt = BNO055_StartAsyncHeadingRead(&hi2c1);
        uint32_t start = HAL_GetTick();
        while (!BNO055_AsyncHeadingReady() && (HAL_GetTick() - start) < 500) {}
        uint8_t hReady = BNO055_AsyncHeadingReady();
        float heading = BNO055_GetAsyncHeading();

        BNO055_Status gSt = BNO055_StartAsyncGyroRead(&hi2c1);
        start = HAL_GetTick();
        while (!BNO055_AsyncGyroReady() && (HAL_GetTick() - start) < 500) {}
        uint8_t gReady = BNO055_AsyncGyroReady();
        float gyroZ = BNO055_GetAsyncGyroZ();

        BT_Sendf("%d,%d,%u,%d,%u,%.2f,%.2f\r\n", i, hSt, hReady, gSt, gReady, heading, gyroZ);

        HAL_Delay(50);
    }

    BT_SendLine("===== BNO ALTERNATE TEST DONE =====");
    while (1) { HAL_Delay(1000); }
}

static void Run_SpinTurnTest(float angle_deg)
{
    g_rotationBraked = 0;
    uint32_t startTime, lastPrintTime, now;
    int32_t prevCntL, prevCntR;
    uint32_t stillTicks = 0;

    Encoder_Update();
    prevCntL = Encoder_GetLeftCount();
    prevCntR = Encoder_GetRightCount();

    startTime = HAL_GetTick();
    while (1)
    {
        now = HAL_GetTick();
        HAL_Delay(50);

        Encoder_Update();
        int32_t cntL = Encoder_GetLeftCount();
        int32_t cntR = Encoder_GetRightCount();
        int32_t deltaL = cntL - prevCntL;
        int32_t deltaR = cntR - prevCntR;

        if (abs(deltaL) < 2 && abs(deltaR) < 2)
        {
            stillTicks++;
            if (stillTicks >= 5) break;
        }
        else
        {
            stillTicks = 0;
        }

        prevCntL = cntL;
        prevCntR = cntR;

        if ((now - startTime) > 2000)
        {
            BT_SendLine("===== SPIN TURN: pre-turn stillness timeout =====");
            break;
        }
    }

    g_robotIsMoving = 1;

    RampProfile_Reset(&rotationProfile);
    RampProfile_SetMove(&rotationProfile, angle_deg, 50.0f);

    float startHeading = g_currentHeading;
    g_rotationTargetHeading = startHeading + angle_deg;
    g_spinTurnActive = 1;

    g_motionControlEnabled = 1;

    BT_Sendf("\r\n===== SPIN TURN TEST angle=%.1f =====\r\n", angle_deg);
    BT_Sendf("startHeading=%.2f target=%.2f\r\n", startHeading, g_rotationTargetHeading);
    BT_SendLine("time_ms,targetAngle,targetOmega,isDecel,heading,rotError,rotOutput,leftVolts,rightVolts,cntL,cntR,remReal,ovr,tgt");

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

            int32_t deltaL = cntL - prevCntL;
            int32_t deltaR = cntR - prevCntR;

            if (fabsf(remainingReal) <= ROT_FINAL_ANGLE_TOLERANCE_DEG &&
                abs(deltaL) < 2 && abs(deltaR) < 2)
            {
                g_robotIsMoving = 0;
                stillTicks++;
                if (stillTicks >= 10) break;
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
    BT_Sendf("finalHeading=%.2f actualTurned=%.2f targetAngle=%.2f error=%.2f\r\n",
             g_currentHeading, actualTurned, angle_deg, finalError);

    MotionController_Reset();
    MotionController_ResetSteering();

    if (fabsf(finalError) > SPIN_TURN_ANOMALY_THRESHOLD_DEG)
    {
        BT_Sendf("ANOMALY DETECTED — correcting residual error of %.2f deg\r\n", finalError);
        Run_SpinTurnTest(finalError);
    }

    BT_SendLine("===== SPIN TURN DONE =====");
}


static void Run_RotationAccelDecelTest(TurnDirection direction)
{
    BT_SendLine("===== ROTATION ACCEL/DECEL TEST =====");

    Encoder_Update();

    // --- ACCEL PHASE: command voltage from rest, log the ramp up ---
    uint32_t startTime = HAL_GetTick();
    uint32_t lastSampleTime = startTime;
    uint32_t sampleCount = 0;

    int pwm = (int)(1000.0f * ROT_DECEL_TEST_VOLTAGE / Battery_ReadVoltage());
    int16_t leftPWM  = (direction == TURN_RIGHT) ?  pwm : -pwm;
    int16_t rightPWM = (direction == TURN_RIGHT) ? -pwm :  pwm;
    Motor_SetRawPWM(leftPWM, rightPWM);

    while (((HAL_GetTick() - startTime) < ROT_DECEL_LOG_DURATION_MS) &&
           (sampleCount < ROT_DECEL_MAX_SAMPLES))
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastSampleTime) >= ROT_DECEL_LOG_INTERVAL_MS)
        {
            rotDecelSamples[sampleCount].time_ms = now - startTime;
            rotDecelSamples[sampleCount].omega_dps = g_currentGyroZ;
            sampleCount++;
            lastSampleTime = now;
        }
    }

    BT_SendLine("--- ACCEL PHASE ---");
    BT_Sendf("samples=%lu\r\n", sampleCount);
    BT_SendLine("time_ms,omega_dps");
    for (uint32_t i = 0; i < sampleCount; i++)
    {
        BT_Sendf("%lu,%.2f\r\n", rotDecelSamples[i].time_ms, rotDecelSamples[i].omega_dps);
    }

    HAL_Delay(ROT_DECEL_TEST_HOLD_MS);

    // --- DECEL PHASE: cut power, log the coast/brake down ---
    Motor_Stop(MOTOR_BRAKE);

    startTime = HAL_GetTick();
    lastSampleTime = startTime;
    sampleCount = 0;

    while (((HAL_GetTick() - startTime) < ROT_DECEL_LOG_DURATION_MS) &&
           (sampleCount < ROT_DECEL_MAX_SAMPLES))
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastSampleTime) >= ROT_DECEL_LOG_INTERVAL_MS)
        {
            rotDecelSamples[sampleCount].time_ms = now - startTime;
            rotDecelSamples[sampleCount].omega_dps = g_currentGyroZ;
            sampleCount++;
            lastSampleTime = now;
        }
    }

    BT_SendLine("--- DECEL PHASE ---");
    BT_Sendf("samples=%lu\r\n", sampleCount);
    BT_SendLine("time_ms,omega_dps");
    for (uint32_t i = 0; i < sampleCount; i++)
    {
        BT_Sendf("%lu,%.2f\r\n", rotDecelSamples[i].time_ms, rotDecelSamples[i].omega_dps);
    }

    BT_SendLine("===== ROTATION ACCEL/DECEL TEST DONE =====");
}

static void Run_StraightAndTurnStressTest(void)
{
    BT_SendLine("===== STRAIGHT + TURN STRESS TEST START =====");
    BT_SendLine("Running forever - reset the board to stop.");

    srand(HAL_GetTick());   // seed once, using boot-time tick for variety

    uint32_t cycleCount = 0;

    while (1)
    {
        cycleCount++;
        BT_Sendf("\r\n----- CYCLE %lu -----\r\n", cycleCount);

        Run_PositionControlTest(254.0f, 800.0f);   // always move 10 inches

        int choice = rand() % 4;   // 0=90, 1=-90, 2=180, 3=no turn

        HAL_Delay(500);

        switch (choice)
        {
            case 0:
                BT_SendLine("--- Turning 90 right ---");
                Run_SpinTurnTest(87.0f);
                break;
            case 1:
                BT_SendLine("--- Turning 90 left ---");
                Run_SpinTurnTest(-87.0f);
                break;
            case 2:
                BT_SendLine("--- Turning 180 ---");
                Run_SpinTurnTest(177.0f);
                break;
            case 3:
                BT_SendLine("--- No turn this cycle ---");
                break;
        }

        HAL_Delay(500);
    }
}

static void Run_SteeringAdjustmentTest(void)
{
    BT_SendLine("===== STEERING ADJUSTMENT TEST =====");
    BT_SendLine("Move robot near walls by hand. Reset to stop.");
    BT_SendLine("time_ms,distL,distR,seeL,seeR,adjustment");

    uint32_t startTime = HAL_GetTick();
    uint32_t lastPrint = startTime;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 100)
        {
            float distL = IR_GetDistanceL_BG();
            float distR = IR_GetDistanceR_BG();
            uint8_t seeL = IR_WallLeft_BG();
            uint8_t seeR = IR_WallRight_BG();

            float steerAdj = MotionController_CalculateSteeringAdjustment();

            BT_Sendf("%lu,%.2f,%.2f,%u,%u,%.3f\r\n",
                     now - startTime, distL, distR, seeL, seeR, steerAdj);
            lastPrint = now;
        }
    }
}

static void Run_IRLiveMonitorTest(void)
{
	BT_Sendf("DEBUG: IR_LEFT_CH=%lu IR_RIGHT_CH=%lu\r\n",
	         (unsigned long)IR_GetLeftChannel(),
	         (unsigned long)IR_GetRightChannel());

    BT_SendLine("===== IR LIVE MONITOR TEST =====");
    BT_SendLine("Continuous raw readings - reset to stop.");
    BT_SendLine("Gently wiggle/press board near sensors while watching for changes.");
    BT_SendLine("time_ms,rawL,rawF,rawR");

    uint32_t startTime = HAL_GetTick();
    uint32_t lastPrint = startTime;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 100)
        {
            BT_Sendf("%lu,%u,%u,%u\r\n",
                     now - startTime,
                     IR_GetL_BG(),
                     IR_GetF_BG(),
                     IR_GetR_BG());
            lastPrint = now;
        }
    }
}

static void Run_SysTickTimingTest(void)
{
    BT_SendLine("===== SYSTICK TIMING TEST =====");
    BT_SendLine("Running for 10 seconds while all normal SysTick work happens.");

    uint32_t startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < 10000)
    {
        HAL_Delay(500);
        // Convert cycles to microseconds: cycles / (SystemCoreClock / 1,000,000)
        float lastUs = (float)g_systickLastCycles / (SystemCoreClock / 1000000.0f);
        float maxUs  = (float)g_systickMaxCycles  / (SystemCoreClock / 1000000.0f);
        BT_Sendf("last=%.2fus max=%.2fus (budget=1000us)\r\n", lastUs, maxUs);
    }

    BT_SendLine("===== SYSTICK TIMING TEST DONE =====");
}

static void Run_FlashlightReceiverTest(void)
{
    BT_SendLine("===== FLASHLIGHT RECEIVER TEST =====");
    BT_SendLine("Shine flashlight at each receiver in turn.");
    BT_SendLine("time_ms,rawL,rawF,rawR");

    uint32_t startTime = HAL_GetTick();
    uint32_t lastPrint = startTime;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 200)
        {
            uint16_t rawL = 0, rawF = 0, rawR = 0;

            if (IR_RequestExternalRead(ADC_CHANNEL_7))   // PA7, left
            {
                while (!IR_ExternalReadReady()) { }
                rawL = IR_GetExternalReadResult();
            }

            if (IR_RequestExternalRead(ADC_CHANNEL_4))   // PA4, front
            {
                while (!IR_ExternalReadReady()) { }
                rawF = IR_GetExternalReadResult();
            }

            if (IR_RequestExternalRead(ADC_CHANNEL_1))   // PA1, right
            {
                while (!IR_ExternalReadReady()) { }
                rawR = IR_GetExternalReadResult();
            }

            BT_Sendf("%lu,%u,%u,%u\r\n", now - startTime, rawL, rawF, rawR);
            lastPrint = now;
        }
    }
}

static void Run_TenInchLoopTest(int numMoves)
{
    BT_SendLine("===== TEN INCH LOOP TEST START =====");

    for (int i = 0; i < numMoves; i++)
    {
        BT_Sendf("\r\n----- MOVE %d of %d -----\r\n", i + 1, numMoves);
        Run_PositionControlTest(254.0f, 800.0f);
        HAL_Delay(500);
    }

    BT_SendLine("===== TEN INCH LOOP TEST DONE =====");
}

static void Run_TenTurnLoopTest(float angle_deg, int numTurns)
{
    BT_SendLine("===== TEN TURN LOOP TEST START =====");
    BT_Sendf("angle=%.1f numTurns=%d\r\n", angle_deg, numTurns);

    float firstStartHeading = g_currentHeading;

    for (int i = 0; i < numTurns; i++)
    {
        BT_Sendf("\r\n----- TURN %d of %d -----\r\n", i + 1, numTurns);
        Run_SpinTurnTest(angle_deg);
        HAL_Delay(500);
    }

    float finalHeading = g_currentHeading;
    float cumulativeDrift = BNO055_AngleError(finalHeading, firstStartHeading);

    BT_Sendf("\r\n===== TEN TURN LOOP DONE =====\r\n");
    BT_Sendf("firstStartHeading=%.2f finalHeading=%.2f cumulativeDrift=%.2f\r\n",
             firstStartHeading, finalHeading, cumulativeDrift);
}

static void Run_EmitterHoldOnTest(void)
{
    BT_SendLine("===== EMITTER HOLD-ON TEST =====");
    BT_SendLine("Front emitter will stay ON continuously.");
    BT_SendLine("Measure MOSFET drain-source and gate-source now.");
    BT_SendLine("Press button again to stop.");

    IR_EmitterRightOn();

    Wait_For_Button();

    IR_EmitterRightOff();

    BT_SendLine("===== EMITTER OFF =====");
}

static float Run_ResquareTest(void)
{
    BT_SendLine("===== RESQUARE TEST START =====");

    uint8_t seeFront = IR_WallFront_BG();
    float frontDist_in = IR_GetDistanceF_BG();

    BT_Sendf("seeFront=%u frontDist=%.2f\r\n", seeFront, frontDist_in);

    if (!seeFront)
    {
        BT_SendLine("No front wall detected — skipping resquare.");
        BT_SendLine("===== RESQUARE TEST DONE (skipped) =====");
        return 0.0f;
    }

    float driveDistance_in = frontDist_in + RESQUARE_OVERSHOOT_IN;
    float driveDistance_mm = driveDistance_in * 25.4f;

    BT_Sendf("Driving %.2f in (%.1f mm) to make wall contact...\r\n",
             driveDistance_in, driveDistance_mm);

    Run_PositionControlTest(driveDistance_mm, RESQUARE_DRIVE_SPEED);

    // True, wall-verified parallel heading, captured right after contact —
    // everything from here is judged against this reference point.
    float trueParallelHeading = g_currentHeading;

    HAL_Delay(300);

    float backtrack_mm = -RESQUARE_BACKTRACK_IN * 25.4f;

    BT_Sendf("Reversing fixed %.2f in (%.1f mm) to turning distance...\r\n",
             RESQUARE_BACKTRACK_IN, backtrack_mm);

    Run_PositionControlTest(backtrack_mm, RESQUARE_REVERSE_SPEED);

    // Drift introduced by reverse alone — signed, so the caller can fold it
    // directly into the next turn's commanded angle rather than attempting
    // a separate, unreliably-small correction turn.
    float reverseDrift_deg = BNO055_AngleError(g_currentHeading, trueParallelHeading);

    BT_Sendf("reverseDrift=%.2f deg\r\n", reverseDrift_deg);

    float finalFrontDist_in = IR_GetDistanceF_BG();
    BT_Sendf("finalFrontDist=%.2f (target ~%.1f)\r\n",
             finalFrontDist_in, RESQUARE_BACKTRACK_IN);

    BT_SendLine("===== RESQUARE TEST DONE =====");

    return reverseDrift_deg;
}

static void Run_MoveThenResquareTest(void)
{
    BT_SendLine("===== MOVE + RESQUARE TEST START =====");

    BT_SendLine("--- Forward move: 10 inches ---");
    Run_PositionControlTest(254.0f, 800.0f);

    HAL_Delay(50);

    BT_SendLine("--- Resquare check ---");
    Run_ResquareTest();

    BT_SendLine("===== MOVE + RESQUARE TEST DONE =====");
}

// ============================================================
// FAST HANDOFF TEST  (added - nothing above this line was changed)
//   segment 1: FH_SEG1_MM from rest, brakes only down to FH_HANDOFF_MM_S
//   handoff  : when segment 1 has FH_HANDOFF_REMAINING_MM left, segment 2 is
//              appended to the SAME running profile (no stop, no encoder reset,
//              no controller reset, no stillness wait)
//   segment 2: FH_SEG2_MM, then a normal stop with your normal approach speed
// Question it answers: does the mouse carry speed cleanly across the junction?
// Needs about (FH_SEG1_MM + FH_SEG2_MM + 100 mm) of clear straight floor.
// Nothing is printed while moving; everything is logged to RAM and printed after.
// ============================================================

// New functions added at the end of ramp_profile.c
float   RampProfile_GetApproachSpeed(RampProfile *p);
void    RampProfile_SetApproachSpeed(RampProfile *p, float approachSpeed);
float   RampProfile_GetRemaining(RampProfile *p);
uint8_t RampProfile_AppendMove(RampProfile *p, float distance, float maxSpeed, float approachSpeed);

#define FH_SEG1_MM               540.0f   // 3 cells (~21 in)
#define FH_HANDOFF_MM_S          400.0f   // speed carried through the junction
#define FH_HANDOFF_REMAINING_MM  20.0f    // append segment 2 when this much of seg 1 is left (must be > finalBrakeDist = 5 mm)
#define FH_SEG2_MM               540.0f   // 3 cells (~21 in), then a normal stop
#define FH_MAX_SPEED_MM_S        800.0f   // cap; the profile picks the real peak
#define FH_LOG_INTERVAL_MS       5
#define FH_MAX_SAMPLES           800      // 800 x 5 ms = 4 s of log

typedef struct
{
    uint16_t t_ms;
    uint8_t  stage;
    float    target_mm_s;
    float    meas_mm_s;
    float    leftVolts;
    float    rightVolts;
    float    heading;
} FHSample;

static FHSample fhSamples[FH_MAX_SAMPLES];

static void Run_FastHandoffTest(void)
{
    BT_SendLine("===== FAST HANDOFF TEST =====");
    BT_Sendf("seg1=%.1f mm -> handoff %.1f mm/s at %.1f mm remaining -> seg2=%.1f mm -> stop\r\n",
             FH_SEG1_MM, FH_HANDOFF_MM_S, FH_HANDOFF_REMAINING_MM, FH_SEG2_MM);

    g_rotationBraked = 0;
    g_robotIsMoving = 1;

    Encoder_ResetAll();
    Encoder_Update();
    MotionController_Reset();

    float startHeading = g_currentHeading;

    // Segment 1 brakes down to (and holds) the handoff speed instead of 350.
    float normalApproach = RampProfile_GetApproachSpeed(&forwardProfile);
    RampProfile_SetApproachSpeed(&forwardProfile, FH_HANDOFF_MM_S);
    RampProfile_SetMove(&forwardProfile, FH_SEG1_MM, FH_MAX_SPEED_MM_S);
    g_motionControlEnabled = 1;

    uint8_t  stage = 1;
    uint8_t  seamless = 0;
    uint32_t sampleCount = 0;
    uint32_t handoffIdx = 0;
    uint32_t handoffTime_ms = 0;
    uint32_t stillChecks = 0;

    uint32_t startTime = HAL_GetTick();
    uint32_t lastSample = startTime;
    uint32_t lastStillCheck = startTime;
    int32_t  prevL = Encoder_GetLeftCount();
    int32_t  prevR = Encoder_GetRightCount();
    int32_t  stillPrevL = prevL;
    int32_t  stillPrevR = prevR;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // --- the handoff: keep this check first and the loop short. If it is
        //     reached more than ~30 ms late the profile has already started its
        //     final stop (AppendMove then returns 0 and this is reported).
        if (stage == 1 && RampProfile_GetRemaining(&forwardProfile) <= FH_HANDOFF_REMAINING_MM)
        {
            seamless = RampProfile_AppendMove(&forwardProfile, FH_SEG2_MM, FH_MAX_SPEED_MM_S, normalApproach);
            stage = 2;
            handoffTime_ms = now - startTime;
            handoffIdx = sampleCount;
        }

        // --- log into RAM (no printing while moving)
        if (((now - lastSample) >= FH_LOG_INTERVAL_MS) && (sampleCount < FH_MAX_SAMPLES))
        {
            int32_t l = Encoder_GetLeftCount();
            int32_t r = Encoder_GetRightCount();
            float dt = (float)(now - lastSample) / 1000.0f;
            float vL = ((float)(l - prevL) * MM_PER_TICK_LEFT)  / dt;
            float vR = ((float)(r - prevR) * MM_PER_TICK_RIGHT) / dt;

            fhSamples[sampleCount].t_ms        = (uint16_t)(now - startTime);
            fhSamples[sampleCount].stage       = stage;
            fhSamples[sampleCount].target_mm_s = RampProfile_GetSpeed(&forwardProfile);
            fhSamples[sampleCount].meas_mm_s   = 0.5f * (vL + vR);
            fhSamples[sampleCount].leftVolts   = MotionController_GetLastLeftVolts();
            fhSamples[sampleCount].rightVolts  = MotionController_GetLastRightVolts();
            fhSamples[sampleCount].heading     = g_currentHeading;
            sampleCount++;

            prevL = l;
            prevR = r;
            lastSample = now;
        }

        // --- segment 2 finished: wait until the wheels are really still, then stop
        if (stage == 2 && RampProfile_IsFinished(&forwardProfile))
        {
            if ((now - lastStillCheck) >= 20)
            {
                int32_t l = Encoder_GetLeftCount();
                int32_t r = Encoder_GetRightCount();
                int32_t dl = l - stillPrevL; if (dl < 0) dl = -dl;
                int32_t dr = r - stillPrevR; if (dr < 0) dr = -dr;

                if (dl < 2 && dr < 2) stillChecks++;
                else                  stillChecks = 0;

                stillPrevL = l;
                stillPrevR = r;
                lastStillCheck = now;

                if (stillChecks >= 3) break;
            }
        }

        if ((now - startTime) > 6000)
        {
            BT_SendLine("===== FAST HANDOFF TIMEOUT =====");
            break;
        }
    }

    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    MotionController_Reset();
    Motor_Stop(MOTOR_BRAKE);

    // put your normal approach speed back for every other test
    RampProfile_SetApproachSpeed(&forwardProfile, normalApproach);

    float totalDist = 0.5f * (((float)Encoder_GetLeftCount()  * MM_PER_TICK_LEFT) +
                              ((float)Encoder_GetRightCount() * MM_PER_TICK_RIGHT));
    float residualHeading = BNO055_AngleError(g_currentHeading, startHeading);

    // ---- print the log now that the mouse is stopped ----
    BT_SendLine("time_ms,stage,targetSpeed,measSpeed,leftVolts,rightVolts,heading");
    for (uint32_t i = 0; i < sampleCount; i++)
    {
        BT_Sendf("%u,%u,%.1f,%.1f,%.2f,%.2f,%.2f\r\n",
                 fhSamples[i].t_ms, fhSamples[i].stage,
                 fhSamples[i].target_mm_s, fhSamples[i].meas_mm_s,
                 fhSamples[i].leftVolts, fhSamples[i].rightVolts,
                 fhSamples[i].heading);
    }
    if (sampleCount >= FH_MAX_SAMPLES) BT_SendLine("(log buffer filled - end of move not fully logged)");

    BT_SendLine("--- summary ---");
    if (stage == 2)
    {
        BT_Sendf("handoff at t=%lu ms (log sample %lu)\r\n",
                 (unsigned long)handoffTime_ms, (unsigned long)handoffIdx);
        if (!seamless)
            BT_SendLine("LATE HANDOFF: the profile had already started its final stop (speed went to 0). Result is not valid.");

        // measured speed within +-10 samples (+-50 ms) of the handoff
        float minNear = 1.0e9f;
        float maxNear = 0.0f;
        uint32_t lo = (handoffIdx > 10) ? (handoffIdx - 10) : 0;
        uint32_t hi = ((handoffIdx + 10) < sampleCount) ? (handoffIdx + 10) : sampleCount;
        for (uint32_t i = lo; i < hi; i++)
        {
            float m = fhSamples[i].meas_mm_s;
            if (m < minNear) minNear = m;
            if (m > maxNear) maxNear = m;
        }
        BT_Sendf("measured speed within +-50 ms of handoff: min=%.1f max=%.1f (handoff target %.1f)\r\n",
                 minNear, maxNear, FH_HANDOFF_MM_S);
    }
    else
    {
        BT_SendLine("segment 1 never reached its handoff point (see timeout above)");
    }
    BT_Sendf("measured distance=%.1f mm (planned %.1f mm)  heading drift=%.2f deg\r\n",
             totalDist, FH_SEG1_MM + FH_SEG2_MM, residualHeading);
    BT_SendLine("===== FAST HANDOFF DONE =====");
}

// ============================================================
// SMOOTH TURN TEST  (added - nothing above this line was changed)
//   One run, no stops: straight -> 90 degree ARC at speed -> straight -> stop.
//   The forward profile keeps cruising the whole time. When it reaches
//   ST_STRAIGHT_MM, the rotation profile is started on top of it, so your
//   controller (which already adds the forward and rotation targets every
//   tick) drives an arc instead of a spin.
// Question it answers: can the mouse take a corner at speed, and where and
// how straight does it come out?
// Run it on open floor with NO walls close to the sides (the IR steering is
// still active in this test; with no walls it reads zero).
// Needs about 1.5 m forward and 0.6 m to the turn side, clear.
// Nothing is printed while moving; everything is logged to RAM and printed after.
// ============================================================
#define ST_STRAIGHT_MM          295.0f   // straight before the arc starts (2 cells)
#define ST_ARC_EST_MM           300.0f   // distance the mouse travels during the arc (from simulation)
#define ST_RUNOUT_MM            360.0f   // straight after the arc, then a normal stop
#define ST_SPEED_MM_S           500.0f   // forward speed carried through the arc
#define ST_TURN_DEG             90.0f    // + = same direction as Run_SpinTurnTest(+90)
#define ST_TURN_MAX_OMEGA       250.0f   // peak turn rate, deg/s (250 -> arc exit offset ~ 182 x 186 mm)
#define ST_TURN_FINISH_COMP_DEG 3.0f     // the rotation profile ends 3 deg early (its finalBrakeDist), so ask for 3 more
#define ST_DEG2RAD              0.0174533f
#define ST_LOG_INTERVAL_MS      10
#define ST_MAX_SAMPLES          450      // 450 x 10 ms = 4.5 s of log

typedef struct
{
    uint16_t t_ms;
    uint8_t  stage;        // 0 = straight, 1 = arc, 2 = after the arc
    float    fwdTarget;    // mm/s
    float    rotTarget;    // deg/s
    float    meas_mm_s;
    float    heading;      // relative to the start, deg
    float    x_mm;         // dead-reckoned, start frame: x forward
    float    y_mm;         // dead-reckoned: + toward the turn direction
} STSample;

static STSample stSamples[ST_MAX_SAMPLES];

static void Run_SmoothTurnTest(void)
{
    BT_SendLine("===== SMOOTH TURN TEST =====");
    BT_Sendf("straight %.0f mm -> %.0f deg arc (max %.0f deg/s) -> straight %.0f mm -> stop, speed %.0f mm/s\r\n",
             ST_STRAIGHT_MM, ST_TURN_DEG, ST_TURN_MAX_OMEGA, ST_RUNOUT_MM, ST_SPEED_MM_S);

    g_rotationBraked = 0;
    g_spinTurnActive = 0;
    g_robotIsMoving = 1;

    Encoder_ResetAll();
    Encoder_Update();
    MotionController_Reset();
    MotionController_ResetSteering();

    float startHeading = g_currentHeading;

    RampProfile_SetMove(&forwardProfile, ST_STRAIGHT_MM + ST_ARC_EST_MM + ST_RUNOUT_MM, ST_SPEED_MM_S);
    g_motionControlEnabled = 1;

    uint8_t  turnStarted = 0;
    uint8_t  turnDone = 0;
    uint32_t turnStartMs = 0, turnEndMs = 0;
    float    arcStartX = 0.0f, arcStartY = 0.0f, arcStartPath = 0.0f;
    float    arcEndX = 0.0f,   arcEndY = 0.0f,   arcEndPath = 0.0f, arcEndHeading = 0.0f;

    float    x = 0.0f, y = 0.0f, path = 0.0f;
    uint32_t sampleCount = 0;
    uint32_t stillChecks = 0;

    uint32_t startTime = HAL_GetTick();
    uint32_t lastSample = startTime;
    uint32_t lastStillCheck = startTime;
    int32_t  deadL = Encoder_GetLeftCount();
    int32_t  deadR = Encoder_GetRightCount();
    int32_t  prevL = deadL;
    int32_t  prevR = deadR;
    int32_t  stillPrevL = deadL;
    int32_t  stillPrevR = deadR;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // --- dead reckoning from the encoders + gyro heading (every pass, so the
        //     arc start/end positions are accurate to a few tenths of a mm)
        {
            int32_t l = Encoder_GetLeftCount();
            int32_t r = Encoder_GetRightCount();
            float ds = 0.5f * (((float)(l - deadL) * MM_PER_TICK_LEFT) + ((float)(r - deadR) * MM_PER_TICK_RIGHT));
            deadL = l;
            deadR = r;
            float h = BNO055_AngleError(g_currentHeading, startHeading) * ST_DEG2RAD;
            x += ds * cosf(h);
            y += ds * sinf(h);
            path += ds;
        }

        // --- start the arc: rotation profile on top of the running forward profile.
        //     Done with interrupts masked so SysTick never sees it half-set.
        if (!turnStarted && RampProfile_GetPosition(&forwardProfile) >= ST_STRAIGHT_MM)
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            RampProfile_SetMove(&rotationProfile, ST_TURN_DEG + ST_TURN_FINISH_COMP_DEG, ST_TURN_MAX_OMEGA);
            __set_PRIMASK(primask);

            turnStarted = 1;
            turnStartMs = now - startTime;
            arcStartX = x;
            arcStartY = y;
            arcStartPath = path;
        }

        // --- the arc is over when the rotation profile says so
        if (turnStarted && !turnDone && RampProfile_IsFinished(&rotationProfile))
        {
            turnDone = 1;
            turnEndMs = now - startTime;
            arcEndX = x;
            arcEndY = y;
            arcEndPath = path;
            arcEndHeading = BNO055_AngleError(g_currentHeading, startHeading);
        }

        // --- log into RAM (no printing while moving)
        if (((now - lastSample) >= ST_LOG_INTERVAL_MS) && (sampleCount < ST_MAX_SAMPLES))
        {
            int32_t l = Encoder_GetLeftCount();
            int32_t r = Encoder_GetRightCount();
            float dt = (float)(now - lastSample) / 1000.0f;
            float vL = ((float)(l - prevL) * MM_PER_TICK_LEFT)  / dt;
            float vR = ((float)(r - prevR) * MM_PER_TICK_RIGHT) / dt;

            stSamples[sampleCount].t_ms      = (uint16_t)(now - startTime);
            stSamples[sampleCount].stage     = turnDone ? 2 : (turnStarted ? 1 : 0);
            stSamples[sampleCount].fwdTarget = RampProfile_GetSpeed(&forwardProfile);
            stSamples[sampleCount].rotTarget = RampProfile_GetSpeed(&rotationProfile);
            stSamples[sampleCount].meas_mm_s = 0.5f * (vL + vR);
            stSamples[sampleCount].heading   = BNO055_AngleError(g_currentHeading, startHeading);
            stSamples[sampleCount].x_mm      = x;
            stSamples[sampleCount].y_mm      = y;
            sampleCount++;

            prevL = l;
            prevR = r;
            lastSample = now;
        }

        // --- forward profile finished: wait until the wheels are really still
        if (turnDone && RampProfile_IsFinished(&forwardProfile))
        {
            if ((now - lastStillCheck) >= 20)
            {
                int32_t l = Encoder_GetLeftCount();
                int32_t r = Encoder_GetRightCount();
                int32_t dl = l - stillPrevL; if (dl < 0) dl = -dl;
                int32_t dr = r - stillPrevR; if (dr < 0) dr = -dr;

                if (dl < 2 && dr < 2) stillChecks++;
                else                  stillChecks = 0;

                stillPrevL = l;
                stillPrevR = r;
                lastStillCheck = now;

                if (stillChecks >= 3) break;
            }
        }

        if ((now - startTime) > 7000)
        {
            BT_SendLine("===== SMOOTH TURN TIMEOUT =====");
            break;
        }
    }

    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    MotionController_Reset();
    Motor_Stop(MOTOR_BRAKE);

    float finalHeading = BNO055_AngleError(g_currentHeading, startHeading);

    // ---- print the log now that the mouse is stopped ----
    BT_SendLine("time_ms,stage,fwdTarget,rotTarget,measSpeed,heading,x_mm,y_mm");
    for (uint32_t i = 0; i < sampleCount; i++)
    {
        BT_Sendf("%u,%u,%.1f,%.1f,%.1f,%.2f,%.1f,%.1f\r\n",
                 stSamples[i].t_ms, stSamples[i].stage,
                 stSamples[i].fwdTarget, stSamples[i].rotTarget, stSamples[i].meas_mm_s,
                 stSamples[i].heading, stSamples[i].x_mm, stSamples[i].y_mm);
    }
    if (sampleCount >= ST_MAX_SAMPLES) BT_SendLine("(log buffer filled - end of move not fully logged)");

    BT_SendLine("--- summary ---");
    if (turnDone)
    {
        float arcMs = (float)(turnEndMs - turnStartMs);
        float arcLen = arcEndPath - arcStartPath;
        BT_Sendf("arc: start t=%lu ms, end t=%lu ms (%.0f ms), path %.1f mm, average speed %.0f mm/s (commanded %.0f)\r\n",
                 (unsigned long)turnStartMs, (unsigned long)turnEndMs, arcMs, arcLen,
                 (arcMs > 0.0f) ? (arcLen * 1000.0f / arcMs) : 0.0f, ST_SPEED_MM_S);
        BT_Sendf("arc exit offset: forward %.1f mm, sideways %.1f mm (simulation expects about 182 / 186)\r\n",
                 arcEndX - arcStartX, arcEndY - arcStartY);
        BT_Sendf("heading when the arc ended: %.2f deg (wanted %.1f)\r\n", arcEndHeading, ST_TURN_DEG);
    }
    else
    {
        BT_SendLine("the arc never finished (see timeout above)");
    }
    BT_Sendf("final heading: %.2f deg (error %.2f)   final position: x=%.1f mm  y=%.1f mm\r\n",
             finalHeading, finalHeading - ST_TURN_DEG, x, y);
    BT_SendLine("===== SMOOTH TURN DONE =====");
}

// ============================================================
// PATH TEST  (added - nothing above this line was changed)
//   A whole route in ONE continuous move: straights joined by arcs, no stops.
//   Each step is "drive straight_mm, then arc turn_deg" (+ = same direction as
//   Run_SpinTurnTest(+90) = right, - = the other way). After the last step it
//   drives PT_RUNOUT_MM and stops.
//   The forward profile is one long move at PT_SPEED_MM_S. The rotation profile
//   is started on top of it at each corner, exactly like the smooth turn test.
//
//   straight_mm is measured from where the previous arc ended along its path
//   (the arc itself is counted as PT_ARC_PATH_MM of forward travel), so every
//   straight can be tuned on its own. Offsets are the knobs: to move a corner,
//   change that step's straight_mm.
//
//   Run it on open floor with NO walls close to the sides (IR steering is still
//   on in this test). Default route = a U: right, short straight, right again.
//   Needs about 0.9 m x 0.9 m clear, plus room to stop.
//
//   The printout has every column of the position control test, in the same
//   order, followed by extra columns for this test. It is logged to RAM while
//   moving (no printing mid-move) and printed after the stop, every
//   PT_LOG_INTERVAL_MS instead of every 100 ms.
// ============================================================
#define PT_SPEED_MM_S          400.0f   // forward speed for the whole route
#define PT_MAX_OMEGA           400.0f   // peak turn rate in the arcs, deg/s
#define PT_ARC_PATH_MM         137.3f   // forward travel inside one 90 deg arc (measured: 300.5 mm at 500 mm/s)
#define PT_FINISH_COMP_DEG     1.5f     // rotation profile ends 3 deg early (its finalBrakeDist), so ask for 3 more
#define PT_RUNOUT_MM           360.0f   // straight after the last arc, then a normal stop
#define PT_DEG2RAD             0.0174533f
#define PT_LOG_INTERVAL_MS     40
#define PT_MAX_SAMPLES         250      // 250 x 20 ms = 5 s of log (about 23 KB of RAM; lower it if the linker says RAM overflowed)
#define PT_MAX_STEPS           8

typedef struct
{
    float straight_mm;   // straight before this arc starts (after the previous arc ended)
    float turn_deg;      // + / - ; 0 = no arc here
} PathStep;

static const PathStep PT_STEPS[] =
{
    { 295.0f,  90.0f },  // straight, then arc 90 deg
    { 150.0f,  90.0f },  // short straight, then arc 90 deg the same way (U shape)
};
#define PT_NUM_STEPS ((int)(sizeof(PT_STEPS) / sizeof(PT_STEPS[0])))

typedef struct
{
    uint16_t t_ms;
    uint8_t  turnsStarted;   // 0 before the first arc, then 1, 2, ...
    uint8_t  arcActive;      // 1 while an arc is running
    uint8_t  isDecel;
    uint8_t  seeL;
    uint8_t  seeR;
    int32_t  cntL;
    int32_t  cntR;
    // same columns as the position control test
    float    targetPos;
    float    targetSpeed;
    float    measPos;
    float    posError;
    float    heading;        // absolute, as in the position control test
    float    rotError;
    float    posOutput;
    float    rotOutput;
    float    leftVolts;
    float    rightVolts;
    float    irDistL;
    float    irDistR;
    float    steerAdj;
    float    kdAngleDeg;
    // extra for this test
    float    rotTarget;      // rotation profile speed, deg/s
    float    relHeading;     // heading relative to the start, deg
    float    x_mm;           // dead-reckoned, start frame: x forward
    float    y_mm;           // + = toward a positive (+) turn
    float    battV;
} PTSample;

static PTSample ptSamples[PT_MAX_SAMPLES];

typedef struct
{
    uint32_t startMs, endMs;
    float    startHeading, endHeading;   // relative to the start of the run, deg
    float    dx, dy, path;               // global start-frame offsets and path length
    uint8_t  done;
} PTArc;

static void Run_PathTestSteps(const char *title, const PathStep *steps, int numSteps,
                              float speed, float maxOmega, float runout)
{
    BT_Sendf("===== %s =====\r\n", title);
    BT_Sendf("speed %.0f mm/s, max turn rate %.0f deg/s, %d arc(s), runout %.0f mm\r\n",
             speed, maxOmega, numSteps, runout);

    if (numSteps < 1 || numSteps > PT_MAX_STEPS)
    {
        BT_SendLine("steps must have 1..8 steps");
        return;
    }

    // forward distance of the whole route
    float totalFwd = runout;
    for (int i = 0; i < numSteps; i++)
    {
        totalFwd += steps[i].straight_mm;
        if (steps[i].turn_deg != 0.0f) totalFwd += PT_ARC_PATH_MM;
    }

    g_rotationBraked = 0;
    g_spinTurnActive = 0;
    g_robotIsMoving = 1;

    Encoder_ResetAll();
    Encoder_Update();
    int32_t startCntL = Encoder_GetLeftCount();
    int32_t startCntR = Encoder_GetRightCount();

    MotionController_Reset();
    MotionController_ResetSteering();

    float startHeading = g_currentHeading;
    BT_Sendf("startHeading=%.2f\r\n", startHeading);

    RampProfile_SetMove(&forwardProfile, totalFwd, speed);
    g_motionControlEnabled = 1;

    PTArc    arcs[PT_MAX_STEPS];
    for (int i = 0; i < PT_MAX_STEPS; i++) arcs[i].done = 0;

    int      step = 0;                            // next step to start
    uint8_t  arcActive = 0;
    float    trigger = steps[0].straight_mm;   // forward position at which the next arc starts

    float    x = 0.0f, y = 0.0f, path = 0.0f;
    uint32_t sampleCount = 0;
    uint32_t stillChecks = 0;

    uint32_t startTime = HAL_GetTick();
    uint32_t lastSample = startTime;
    uint32_t lastStillCheck = startTime;
    int32_t  deadL = startCntL;
    int32_t  deadR = startCntR;
    int32_t  stillPrevL = startCntL;
    int32_t  stillPrevR = startCntR;

    float arcStartX = 0.0f, arcStartY = 0.0f, arcStartPath = 0.0f;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // --- dead reckoning from encoders + gyro heading, every pass
        {
            int32_t l = Encoder_GetLeftCount();
            int32_t r = Encoder_GetRightCount();
            float ds = 0.5f * (((float)(l - deadL) * MM_PER_TICK_LEFT) + ((float)(r - deadR) * MM_PER_TICK_RIGHT));
            deadL = l;
            deadR = r;
            float h = BNO055_AngleError(g_currentHeading, startHeading) * PT_DEG2RAD;
            x += ds * cosf(h);
            y += ds * sinf(h);
            path += ds;
        }

        // --- an arc finished?
        if (arcActive && RampProfile_IsFinished(&rotationProfile))
        {
            PTArc *a = &arcs[step - 1];
            a->endMs = now - startTime;
            a->endHeading = BNO055_AngleError(g_currentHeading, startHeading);
            a->dx = x - arcStartX;
            a->dy = y - arcStartY;
            a->path = path - arcStartPath;
            a->done = 1;
            arcActive = 0;

            // next corner: its straight starts counting where this arc's path ends
            if (step < numSteps) trigger += PT_ARC_PATH_MM + steps[step].straight_mm;
        }

        // --- start the next arc: rotation profile on top of the running forward profile.
        //     Done with interrupts masked so SysTick never sees it half-set.
        if (!arcActive && step < numSteps && RampProfile_GetPosition(&forwardProfile) >= trigger)
        {
            float deg = steps[step].turn_deg;

            if (deg == 0.0f)
            {
                // no arc at this step: just move the trigger on to the next one
                step++;
                if (step < numSteps) trigger += steps[step].straight_mm;
            }
            else
            {
                float cmd = ((deg >= 0.0f) ? 1.0f : -1.0f) * (fabsf(deg) + PT_FINISH_COMP_DEG);

                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                RampProfile_SetMove(&rotationProfile, cmd, maxOmega);
                __set_PRIMASK(primask);

                arcs[step].startMs = now - startTime;
                arcs[step].startHeading = BNO055_AngleError(g_currentHeading, startHeading);
                arcStartX = x;
                arcStartY = y;
                arcStartPath = path;
                arcActive = 1;
                step++;
            }
        }

        // --- log into RAM (no printing while moving)
        if (((now - lastSample) >= PT_LOG_INTERVAL_MS) && (sampleCount < PT_MAX_SAMPLES))
        {
            int32_t cl = Encoder_GetLeftCount();
            int32_t cr = Encoder_GetRightCount();
            float distL = (float)(cl - startCntL) * MM_PER_TICK_LEFT;
            float distR = (float)(cr - startCntR) * MM_PER_TICK_RIGHT;
            float measPos = 0.5f * (distL + distR);
            float targetPos = RampProfile_GetPosition(&forwardProfile);

            PTSample *s = &ptSamples[sampleCount];
            s->t_ms         = (uint16_t)(now - startTime);
            s->turnsStarted = (uint8_t)step;
            s->arcActive    = arcActive;
            s->isDecel      = RampProfile_IsDecelerating(&forwardProfile);
            s->seeL         = IR_WallLeft_BG();
            s->seeR         = IR_WallRight_BG();
            s->cntL         = cl;
            s->cntR         = cr;
            s->targetPos    = targetPos;
            s->targetSpeed  = RampProfile_GetSpeed(&forwardProfile);
            s->measPos      = measPos;
            s->posError     = targetPos - measPos;
            s->heading      = g_currentHeading;
            s->rotError     = MotionController_GetLastRotError();
            s->posOutput    = MotionController_GetLastPosOutput();
            s->rotOutput    = MotionController_GetLastRotOutput();
            s->leftVolts    = MotionController_GetLastLeftVolts();
            s->rightVolts   = MotionController_GetLastRightVolts();
            s->irDistL      = IR_GetDistanceL_BG();
            s->irDistR      = IR_GetDistanceR_BG();
            s->steerAdj     = MotionController_GetLastSteeringAdjustment();
            s->kdAngleDeg   = MotionController_GetLastAngleEstimateDeg();
            s->rotTarget    = RampProfile_GetSpeed(&rotationProfile);
            s->relHeading   = BNO055_AngleError(g_currentHeading, startHeading);
            s->x_mm         = x;
            s->y_mm         = y;
            s->battV        = Battery_ReadVoltage();
            sampleCount++;

            lastSample = now;
        }

        // --- all arcs done and the forward profile finished: wait until the wheels are still
        if (step >= numSteps && !arcActive && RampProfile_IsFinished(&forwardProfile))
        {
            if ((now - lastStillCheck) >= 20)
            {
                int32_t l = Encoder_GetLeftCount();
                int32_t r = Encoder_GetRightCount();
                int32_t dl = l - stillPrevL; if (dl < 0) dl = -dl;
                int32_t dr = r - stillPrevR; if (dr < 0) dr = -dr;

                if (dl < 2 && dr < 2) stillChecks++;
                else                  stillChecks = 0;

                stillPrevL = l;
                stillPrevR = r;
                lastStillCheck = now;

                if (stillChecks >= 3) break;
            }
        }

        if ((now - startTime) > 9000)
        {
            BT_SendLine("===== PATH TEST TIMEOUT =====");
            break;
        }
    }

    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    MotionController_Reset();
    Motor_Stop(MOTOR_BRAKE);

    float finalHeadingAbs = g_currentHeading;
    float finalHeading = BNO055_AngleError(g_currentHeading, startHeading);

    // ---- print the log now that the mouse is stopped ----
    BT_Sendf("\r\n===== PATH TEST LOG (%d samples, %d ms apart) =====\r\n", (int)sampleCount, PT_LOG_INTERVAL_MS);
    BT_SendLine("time_ms,targetPos,targetSpeed,isDecel,measPos,posError,heading,rotError,posOutput,rotOutput,leftVolts,rightVolts,cntL,cntR,distL,distR,seeL,seeR,steerAdj,kdAngleDeg,rotTarget,turnsStarted,arc,relHeading,x_mm,y_mm,battV");
    for (uint32_t i = 0; i < sampleCount; i++)
    {
        PTSample *s = &ptSamples[i];
        BT_Sendf("%u,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%.2f,%.2f,%u,%u,%.3f,%.3f,",
                 s->t_ms, s->targetPos, s->targetSpeed, s->isDecel, s->measPos, s->posError,
                 s->heading, s->rotError, s->posOutput, s->rotOutput,
                 s->leftVolts, s->rightVolts, (long)s->cntL, (long)s->cntR,
                 s->irDistL, s->irDistR, s->seeL, s->seeR, s->steerAdj, s->kdAngleDeg);
        BT_Sendf("%.1f,%u,%u,%.2f,%.1f,%.1f,%.2f\r\n",
                 s->rotTarget, s->turnsStarted, s->arcActive, s->relHeading, s->x_mm, s->y_mm, s->battV);
    }
    if (sampleCount >= PT_MAX_SAMPLES) BT_SendLine("(log buffer filled - end of move not fully logged)");

    BT_Sendf("finalHeading=%.2f residualError=%.2f\r\n", finalHeadingAbs, NormalizeAngleDifference(finalHeadingAbs - startHeading));

    BT_SendLine("--- summary ---");
    float expectedTotal = 0.0f;
    for (int i = 0; i < numSteps; i++)
    {
        expectedTotal += steps[i].turn_deg;
        if (steps[i].turn_deg == 0.0f) continue;

        PTArc *a = &arcs[i];
        if (!a->done)
        {
            BT_Sendf("arc %d: did not finish\r\n", i + 1);
            continue;
        }

        // offsets in the arc's own frame: forward along where the mouse was heading,
        // sideways positive toward the turn direction
        float h0 = a->startHeading * PT_DEG2RAD;
        float fwd  = a->dx * cosf(h0) + a->dy * sinf(h0);
        float side = -a->dx * sinf(h0) + a->dy * cosf(h0);
        if (steps[i].turn_deg < 0.0f) side = -side;

        BT_Sendf("arc %d: %.0f ms, path %.1f mm, exit offset forward %.1f / sideways %.1f mm, heading %.1f -> %.1f (turned %.1f, asked %.1f)\r\n",
                 i + 1, (float)(a->endMs - a->startMs), a->path, fwd, side,
                 a->startHeading, a->endHeading,
                 BNO055_AngleError(a->endHeading, a->startHeading), steps[i].turn_deg);
    }
    BT_Sendf("final heading vs start: %.2f deg (route total %.1f, error %.2f)   final position: x=%.1f mm  y=%.1f mm\r\n",
             finalHeading, expectedTotal, BNO055_AngleError(finalHeading, expectedTotal), x, y);
    BT_SendLine("===== PATH TEST DONE =====");
}

static void Run_PathTest(void)
{
    Run_PathTestSteps("PATH TEST", PT_STEPS, PT_NUM_STEPS, PT_SPEED_MM_S, PT_MAX_OMEGA, PT_RUNOUT_MM);
}

// Same in-depth log as the path test, but for the single arc of the smooth turn test
// (uses that test's ST_ settings). Your original Run_SmoothTurnTest() is untouched.
static const PathStep ST_DETAIL_STEPS[] = { { ST_STRAIGHT_MM, ST_TURN_DEG } };

static void Run_SmoothTurnDetailTest(void)
{
    Run_PathTestSteps("SMOOTH TURN DETAIL", ST_DETAIL_STEPS, 1, ST_SPEED_MM_S, ST_TURN_MAX_OMEGA, ST_RUNOUT_MM);
}

// ============================================================
// SQUARE LOOP TEST (continuous, no stops)
// ============================================================
// Drives a square of LP_SIDE_MM per side, LP_LAPS times round, as one continuous
// forward move with a 90 deg arc at every corner (same arcs as the path test).
// Start at the CENTER of a corner cell, facing along a side; the mouse turns right
// (clockwise) at every corner and ends LP_RUNOUT_MM past where it started the last side.
//
// The straight lengths come from the measured arc: the arc exit sits about 229 mm
// ahead / 120 mm sideways of the point where it started on arc 1, and about 231 mm
// ahead / 102 mm sideways on later arcs, so
//   first straight  = side - 229
//   second straight = side - (120 + 231) = side - 350
//   every other     = side - (102 + 231) = side - 334
// The point of the test is to see how the heading and the position drift over
// 20 arcs: the log shows heading and x/y, and the summary prints a closure check
// after every lap (x,y should come back near 0,0 and the heading near 360 x lap).
#define LP_SIDE_MM         540.0f
#define LP_LAPS            5
#define LP_ARCS            (4 * LP_LAPS)
#define LP_TURN_DIR        1.0f     // +1 = right (clockwise), -1 = left
#define LP_FWD_FIRST_MM    229.0f
#define LP_SIDE_FIRST_MM   120.0f
#define LP_FWD_LATER_MM    231.0f
#define LP_SIDE_LATER_MM   102.0f
#define LP_RUNOUT_MM       360.0f
#define LP_LOG_INTERVAL_MS 25
#define LP_MAX_SAMPLES     1200     // x 26 bytes = 31 KB (30 s of log); lower it if the linker says RAM overflowed

// straight length before arc i (i = 0 .. LP_ARCS-1)
static float LP_Straight(int i)
{
    if (i == 0) return LP_SIDE_MM - LP_FWD_FIRST_MM;
    if (i == 1) return LP_SIDE_MM - (LP_SIDE_FIRST_MM + LP_FWD_LATER_MM);
    return LP_SIDE_MM - (LP_SIDE_LATER_MM + LP_FWD_LATER_MM);
}

// compact sample: the units are in the printed column names
typedef struct
{
    uint16_t t_ms;
    uint8_t  arcsStarted;
    uint8_t  arcActive;
    int16_t  relHead_d10;     // unwrapped heading since the start, 0.1 deg
    int16_t  rotErr_d10;      // rotation error, 0.1 deg
    int16_t  rotTarget_d10;   // rotation profile speed, 0.1 deg/s
    int16_t  x_d10;           // 0.1 mm
    int16_t  y_d10;           // 0.1 mm (+ = toward the turn side)
    int16_t  fwdSpeed_d10;    // forward profile speed, 0.1 mm/s
    int16_t  lV_c;            // volts x 100
    int16_t  rV_c;
    uint16_t batt_mV;
    uint8_t  seeL;
    uint8_t  seeR;
} LPSample;

static LPSample lpSamples[LP_MAX_SAMPLES];

static int16_t LP_Q(float v, float scale)
{
    float s = v * scale;
    if (s > 32767.0f) s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
}

static void Run_LoopTest(void)
{
    const float speed = PT_SPEED_MM_S;
    const float maxOmega = PT_MAX_OMEGA;

    float totalFwd = LP_RUNOUT_MM + (float)LP_ARCS * PT_ARC_PATH_MM;
    for (int i = 0; i < LP_ARCS; i++) totalFwd += LP_Straight(i);
    uint32_t timeoutMs = (uint32_t)(totalFwd / speed * 1000.0f) + 8000u;

    BT_SendLine("===== LOOP TEST =====");
    BT_Sendf("side %.0f mm, %d laps, %d arcs, speed %.0f mm/s, max turn rate %.0f deg/s, route %.0f mm, timeout %lu ms\r\n",
             LP_SIDE_MM, LP_LAPS, LP_ARCS, speed, maxOmega, totalFwd, (unsigned long)timeoutMs);

    g_rotationBraked = 0;
    g_spinTurnActive = 0;
    g_robotIsMoving = 1;

    Encoder_ResetAll();
    Encoder_Update();
    int32_t startCntL = Encoder_GetLeftCount();
    int32_t startCntR = Encoder_GetRightCount();

    MotionController_Reset();
    MotionController_ResetSteering();

    float startHeading = g_currentHeading;
    BT_Sendf("startHeading=%.2f\r\n", startHeading);

    RampProfile_SetMove(&forwardProfile, totalFwd, speed);
    g_motionControlEnabled = 1;

    PTArc    arcs[LP_ARCS];
    for (int i = 0; i < LP_ARCS; i++) arcs[i].done = 0;
    float    arcEndX[LP_ARCS], arcEndY[LP_ARCS];

    int      step = 0;
    uint8_t  arcActive = 0;
    float    trigger = LP_Straight(0);

    float    x = 0.0f, y = 0.0f, path = 0.0f;
    float    hUn = 0.0f;                       // unwrapped heading since the start, deg
    float    hPrev = startHeading;
    uint32_t sampleCount = 0;
    uint32_t stillChecks = 0;

    uint32_t startTime = HAL_GetTick();
    uint32_t lastSample = startTime;
    uint32_t lastStillCheck = startTime;
    int32_t  deadL = startCntL;
    int32_t  deadR = startCntR;
    int32_t  stillPrevL = startCntL;
    int32_t  stillPrevR = startCntR;

    float arcStartX = 0.0f, arcStartY = 0.0f, arcStartPath = 0.0f;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // --- dead reckoning from encoders + unwrapped gyro heading, every pass
        {
            float gh = g_currentHeading;
            hUn += BNO055_AngleError(gh, hPrev);
            hPrev = gh;

            int32_t l = Encoder_GetLeftCount();
            int32_t r = Encoder_GetRightCount();
            float ds = 0.5f * (((float)(l - deadL) * MM_PER_TICK_LEFT) + ((float)(r - deadR) * MM_PER_TICK_RIGHT));
            deadL = l;
            deadR = r;
            float h = hUn * PT_DEG2RAD;
            x += ds * cosf(h);
            y += ds * sinf(h);
            path += ds;
        }

        // --- an arc finished?
        if (arcActive && RampProfile_IsFinished(&rotationProfile))
        {
            PTArc *a = &arcs[step - 1];
            a->endMs = now - startTime;
            a->endHeading = hUn;
            a->dx = x - arcStartX;
            a->dy = y - arcStartY;
            a->path = path - arcStartPath;
            a->done = 1;
            arcEndX[step - 1] = x;
            arcEndY[step - 1] = y;
            arcActive = 0;

            if (step < LP_ARCS) trigger += PT_ARC_PATH_MM + LP_Straight(step);
        }

        // --- start the next arc (interrupts masked while the profile is set)
        if (!arcActive && step < LP_ARCS && RampProfile_GetPosition(&forwardProfile) >= trigger)
        {
            float cmd = LP_TURN_DIR * (90.0f + PT_FINISH_COMP_DEG);

            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            RampProfile_SetMove(&rotationProfile, cmd, maxOmega);
            __set_PRIMASK(primask);

            arcs[step].startMs = now - startTime;
            arcs[step].startHeading = hUn;
            arcStartX = x;
            arcStartY = y;
            arcStartPath = path;
            arcActive = 1;
            step++;
        }

        // --- log into RAM (no printing while moving)
        if (((now - lastSample) >= LP_LOG_INTERVAL_MS) && (sampleCount < LP_MAX_SAMPLES))
        {
            LPSample *s = &lpSamples[sampleCount];
            s->t_ms          = (uint16_t)(now - startTime);
            s->arcsStarted   = (uint8_t)step;
            s->arcActive     = arcActive;
            s->relHead_d10   = LP_Q(hUn, 10.0f);
            s->rotErr_d10    = LP_Q(MotionController_GetLastRotError(), 10.0f);
            s->rotTarget_d10 = LP_Q(RampProfile_GetSpeed(&rotationProfile), 10.0f);
            s->x_d10         = LP_Q(x, 10.0f);
            s->y_d10         = LP_Q(y, 10.0f);
            s->fwdSpeed_d10  = LP_Q(RampProfile_GetSpeed(&forwardProfile), 10.0f);
            s->lV_c          = LP_Q(MotionController_GetLastLeftVolts(), 100.0f);
            s->rV_c          = LP_Q(MotionController_GetLastRightVolts(), 100.0f);
            s->batt_mV       = (uint16_t)(Battery_ReadVoltage() * 1000.0f);
            s->seeL          = IR_WallLeft_BG();
            s->seeR          = IR_WallRight_BG();
            sampleCount++;

            lastSample = now;
        }

        // --- all arcs done and the forward profile finished: wait until the wheels are still
        if (step >= LP_ARCS && !arcActive && RampProfile_IsFinished(&forwardProfile))
        {
            if ((now - lastStillCheck) >= 20)
            {
                int32_t l = Encoder_GetLeftCount();
                int32_t r = Encoder_GetRightCount();
                int32_t dl = l - stillPrevL; if (dl < 0) dl = -dl;
                int32_t dr = r - stillPrevR; if (dr < 0) dr = -dr;

                if (dl < 2 && dr < 2) stillChecks++;
                else                  stillChecks = 0;

                stillPrevL = l;
                stillPrevR = r;
                lastStillCheck = now;

                if (stillChecks >= 3) break;
            }
        }

        if ((now - startTime) > timeoutMs)
        {
            BT_SendLine("===== LOOP TEST TIMEOUT =====");
            break;
        }
    }

    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    MotionController_Reset();
    Motor_Stop(MOTOR_BRAKE);

    float finalHeadingAbs = g_currentHeading;

    // ---- print the log now that the mouse is stopped ----
    BT_Sendf("\r\n===== LOOP TEST LOG (%d samples, %d ms apart) =====\r\n", (int)sampleCount, LP_LOG_INTERVAL_MS);
    BT_SendLine("time_ms,arcsStarted,arc,relHeading,rotError,rotTarget,x_mm,y_mm,fwdSpeed,leftVolts,rightVolts,battV,seeL,seeR");
    for (uint32_t i = 0; i < sampleCount; i++)
    {
        LPSample *s = &lpSamples[i];
        BT_Sendf("%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%u,%u\r\n",
                 s->t_ms, s->arcsStarted, s->arcActive,
                 (float)s->relHead_d10 / 10.0f, (float)s->rotErr_d10 / 10.0f, (float)s->rotTarget_d10 / 10.0f,
                 (float)s->x_d10 / 10.0f, (float)s->y_d10 / 10.0f, (float)s->fwdSpeed_d10 / 10.0f,
                 (float)s->lV_c / 100.0f, (float)s->rV_c / 100.0f, (float)s->batt_mV / 1000.0f,
                 s->seeL, s->seeR);
    }
    if (sampleCount >= LP_MAX_SAMPLES) BT_SendLine("(log buffer filled - end of route not fully logged)");

    BT_Sendf("finalHeading=%.2f residualError=%.2f\r\n", finalHeadingAbs, NormalizeAngleDifference(finalHeadingAbs - startHeading));

    BT_SendLine("--- summary (heading is unwrapped; ideal after arc n is 90 x n) ---");
    for (int i = 0; i < LP_ARCS; i++)
    {
        PTArc *a = &arcs[i];
        if (!a->done)
        {
            BT_Sendf("arc %d: did not finish\r\n", i + 1);
            continue;
        }

        float h0 = a->startHeading * PT_DEG2RAD;
        float fwd  = a->dx * cosf(h0) + a->dy * sinf(h0);
        float side = -a->dx * sinf(h0) + a->dy * cosf(h0);
        if (LP_TURN_DIR < 0.0f) side = -side;

        float ideal = LP_TURN_DIR * 90.0f * (float)(i + 1);
        BT_Sendf("arc %d (lap %d): %.0f ms, path %.1f, exit fwd %.1f side %.1f, turned %.1f, heading %.1f (ideal %.0f, err %.1f), end x=%.0f y=%.0f\r\n",
                 i + 1, i / 4 + 1, (float)(a->endMs - a->startMs), a->path, fwd, side,
                 a->endHeading - a->startHeading, a->endHeading, ideal, a->endHeading - ideal,
                 arcEndX[i], arcEndY[i]);

        if ((i % 4) == 3)
        {
            BT_Sendf("  >> lap %d closure at the end of its 4th arc: heading error %.1f deg, x=%.0f y=%.0f mm\r\n",
                     i / 4 + 1, a->endHeading - ideal, arcEndX[i], arcEndY[i]);
        }
    }
    BT_Sendf("final position: x=%.1f mm  y=%.1f mm  (path %.0f mm), final unwrapped heading %.1f deg (ideal %.0f)\r\n",
             x, y, path, hUn, LP_TURN_DIR * 90.0f * (float)LP_ARCS);
    BT_SendLine("===== LOOP TEST DONE =====");
}

// ============================================================
// SQUARE TEST  (added - nothing above this line was changed)
//   One lap around the OUTSIDE of a 10 inch square made of four 10 inch walls,
//   box on the RIGHT, right turns (clockwise). The lane is ~127 mm out from the
//   box wall, so the four corner points make a 508 mm square (2 cells apart).
//   Start at the CENTER of the middle cell beside one box wall, ~127 mm from the
//   wall, facing along it. It ends back at the start point.
//
//   Straight rule (arc takes about 98 mm forward / 92 mm sideways at 300 mm/s;
//   the profile ends about 18 mm before the mouse is really at 90 deg):
//     first straight from a cell center = 254 * k + 156
//     straight between two turns         = 254 * k + 82
//     runout after the last arc          = 254 * n - 74
//   Uses the path test's settings (speed, turn rate, arc path). One lap is about
//   7 s: set PT_LOG_INTERVAL_MS to 40 or the end of the log is cut off.
// ============================================================
#define SQ_RUNOUT_MM   180.0f

static const PathStep SQ_STEPS[] =
{
    { 156.0f, 90.0f },   // start (middle of a side) -> corner 1
    { 336.0f, 90.0f },   // corner 1 -> corner 2
    { 336.0f, 90.0f },   // corner 2 -> corner 3
    { 336.0f, 90.0f },   // corner 3 -> corner 4, then back to the start point
};
#define SQ_NUM_STEPS ((int)(sizeof(SQ_STEPS) / sizeof(SQ_STEPS[0])))

static void Run_SquareTest(void)
{
    Run_PathTestSteps("SQUARE TEST", SQ_STEPS, SQ_NUM_STEPS, PT_SPEED_MM_S, PT_MAX_OMEGA, SQ_RUNOUT_MM);
}

// ============================================================
// MAIN CALIBRATION SWITCH
// ============================================================

void Calibration_Run(void)
{
    g_irSteerOffInArc = IR_OFF_IN_ARC;

#if TEST_ACC_FF
	Wait_For_Button();
	Encoder_Init();
	Motor_Init();
	Motor_Enable();
	HAL_Delay(500);
	BT_SendLine("===== ACC_FF TEST READY =====");
	Wait_For_Button();
	Run_AccFFTest();
#endif

#if TEST_VOLTAGE_SWEEP
	Wait_For_Button();
	Encoder_Init();
	Motor_Init();
	Motor_Enable();
	HAL_Delay(500);
	BT_SendLine("===== VOLTAGE SWEEP READY =====");
	Wait_For_Button();
	Run_VoltageSweepTest();
#endif

#if TEST_BATTERY_VOLTAGE
    BT_SendLine("===== BATTERY VOLTAGE TEST READY =====");
    Wait_For_Button();
    Run_BatteryVoltageTest();
#endif

#if TEST_BLUETOOTH_ONLY
    HAL_Delay(2000);
    while (1) { BT_SendLine("HELLO"); HAL_Delay(2000); }
#endif

#if TEST_ENCODER_ONLY
    Encoder_Init(); Encoder_ResetAll(); HAL_Delay(500); Wait_For_Button(); Encoder_ResetAll();
    while (1) { Encoder_Update(); rawL = Encoder_GetLeftRaw(); rawR = Encoder_GetRightRaw(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount(); HAL_Delay(10); }
#endif

#if TEST_MOTOR_ENCODER_FOREVER
    Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); HAL_Delay(500); Wait_For_Button(); Encoder_ResetAll(); Motor_SetRawPWM(700, 700);
    while (1) { Encoder_Update(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount(); HAL_Delay(10); }
#endif

#if TEST_BNO055_ONLY
    Wait_For_Button(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
#if BNO_TEST_MODE_IMUPLUS
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
#else
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_NDOF);
#endif
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(100); BT_SendLine("BNO ready. Press button."); Wait_For_Button(); BT_SendLine("===== BNO055 ONLY TEST STARTED =====");
    while (1)
    {
        bnoStatus = BNO055_ReadEuler(&hi2c1, &bnoEuler); bnoStatus = BNO055_ReadGyro(&hi2c1, &bnoGyro);
        if (bnoStatus == BNO055_OK) { bnoHeading = bnoEuler.heading_deg; bnoRoll = bnoEuler.roll_deg; bnoPitch = bnoEuler.pitch_deg; bnoGz = bnoGyro.z_dps; }
        BNO055_ReadCalibration(&hi2c1, &bnoCalib); BNO055_ReadStatus(&hi2c1, &bnoSysStat, &bnoSysErr);
        BT_Sendf("HEAD: %.2f | ROLL: %.2f | PITCH: %.2f\r\n", bnoHeading, bnoRoll, bnoPitch);
        BT_Sendf("GZ: %.2f deg/s\r\n", bnoGz);
        BT_Sendf("CAL SYS:%d GYR:%d ACC:%d MAG:%d | STAT:%d ERR:%d\r\n", bnoCalib.sys, bnoCalib.gyro, bnoCalib.accel, bnoCalib.mag, bnoSysStat, bnoSysErr);
        HAL_Delay(200);
    }
#endif

#if TEST_BNO055_WITH_MOTOR
    Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
#if BNO_TEST_MODE_IMUPLUS
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
#else
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_NDOF);
#endif
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500); Wait_For_Button(); Encoder_ResetAll(); Motor_SetRawPWM(700, 700); BT_SendLine("===== BNO055 + MOTOR + ENCODER TEST =====");
    while (1)
    {
        Encoder_Update(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount();
        bnoStatus = BNO055_ReadEuler(&hi2c1, &bnoEuler); bnoStatus = BNO055_ReadGyro(&hi2c1, &bnoGyro);
        if (bnoStatus == BNO055_OK) { bnoHeading = bnoEuler.heading_deg; bnoGz = bnoGyro.z_dps; }
        BT_Sendf("HEAD: %.2f  GZ: %.2f  L: %ld  R: %ld\r\n", bnoHeading, bnoGz, cntL, cntR);
        HAL_Delay(200);
    }
#endif

#if TEST_ENCODER_CPR_CALIB
    Encoder_Init(); Encoder_ResetAll(); Wait_For_Button(); Encoder_ResetAll();
    BT_SendLine("===== ENCODER CPR CALIBRATION =====");
    while (1) { Encoder_Update(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount(); BT_Sendf("LEFT: %ld  RIGHT: %ld\r\n", cntL, cntR); HAL_Delay(100); }
#endif

#if TEST_MOTOR_SPEED_CALIB
    Wait_For_Button(); Encoder_Init(); Motor_Init(); Motor_Enable(); HAL_Delay(500);
    BT_SendLine("===== MOTOR SPEED CALIBRATION READY ====="); Wait_For_Button(); BT_SendLine("===== MOTOR SPEED CALIBRATION STARTED =====");
    for (int16_t p = 400; p <= 1000; p += 50) Run_MotorSpeedTest(p, 0);
    for (int16_t p = 400; p <= 1000; p += 50) Run_MotorSpeedTest(0, p);
    Motor_SetRawPWM(0, 0); BT_SendLine("===== MOTOR SPEED CALIBRATION DONE =====");
    while (1) { HAL_Delay(1000); }
#endif

#if TEST_DEADZONE
    Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable();
    BT_SendLine("===== DEADZONE TEST ====="); Wait_For_Button(); Run_DeadzoneTest(500); HAL_Delay(3000);
    BT_SendLine("===== DEADZONE TEST DONE ====="); while (1);
#endif

#if TEST_TURN
    Motor_Init(); Motor_Enable(); Encoder_Init(); Encoder_ResetAll(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500);
    BT_SendLine("===== TURN TEST READY =====");
    BT_SendLine("Place robot on ground. Press button to turn.");
    Wait_For_Button();
    TurnResult result = Turn_Execute(&hi2c1, 180.0f, TURN_LEFT);
    if (result == TURN_OK) BT_SendLine("===== TURN OK =====");
    else if (result == TURN_TIMEOUT) BT_SendLine("===== TURN TIMEOUT =====");
    else BT_SendLine("===== TURN IMU ERROR =====");
    Motor_Stop(MOTOR_BRAKE);
    while (1) { HAL_Delay(1000); }
#endif

#if TEST_TURN_SIGN
    Motor_Init(); Motor_Enable(); Encoder_Init(); Encoder_ResetAll(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500);
    Run_TurnSignTest();
#endif

#if TEST_TURN_PROFILE_CALIB
    Motor_Init(); Motor_Enable(); Encoder_Init(); Encoder_ResetAll(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500);
    Run_TurnProfileCalibration();
#endif

#if TEST_IR_LIVE
    IR_Init();
    IR_BackgroundReset();
    Wait_For_Button();
    while (1)
    {
        dbg_irL   = IR_GetL_BG();
        dbg_irF   = IR_GetF_BG();
        dbg_irR   = IR_GetR_BG();
        dbg_distL = IR_GetDistanceL_BG();
        dbg_distF = IR_GetDistanceF_BG();
        dbg_distR = IR_GetDistanceR_BG();
        dbg_wallL = IR_WallLeft_BG();
        dbg_wallF = IR_WallFront_BG();
        dbg_wallR = IR_WallRight_BG();
        dbg_appL  = IR_WallApproachingLeft_BG();
        dbg_appF  = IR_WallApproachingFront_BG();
        dbg_appR  = IR_WallApproachingRight_BG();
        HAL_Delay(20);
    }
#endif

#if TEST_IR_BLUETOOTH
    IR_Init();
    IR_BackgroundReset();
    HAL_Delay(500);
    BT_SendLine("===== IR BLUETOOTH TEST =====");
    BT_SendLine("Press button to start.");
    Wait_For_Button();
    BT_SendLine("rawL,rawF,rawR,distL,distF,distR,wallL,wallF,wallR,appL,appF,appR");
    uint32_t lastPrint = HAL_GetTick();
    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 100)
        {
            BT_Sendf("%u,%u,%u,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u\r\n",
                     IR_GetL_BG(),
                     IR_GetF_BG(),
                     IR_GetR_BG(),
                     IR_GetDistanceL_BG(),
                     IR_GetDistanceF_BG(),
                     IR_GetDistanceR_BG(),
                     IR_WallLeft_BG(),
                     IR_WallFront_BG(),
                     IR_WallRight_BG(),
                     IR_WallApproachingLeft_BG(),
                     IR_WallApproachingFront_BG(),
                     IR_WallApproachingRight_BG());
            lastPrint = now;
        }
    }
#endif

#if TEST_IR_STATUS
    {
        IR_Init();
        IR_BackgroundReset();
        HAL_Delay(500);

        BT_SendLine("===== IR STATUS MONITOR =====");
        BT_SendLine("Watching for changes...");

        float prevDistL = 999.0f, prevDistF = 999.0f, prevDistR = 999.0f;
        uint8_t prevWallL = 0, prevWallF = 0, prevWallR = 0;
        uint8_t prevAppL  = 0, prevAppF  = 0, prevAppR  = 0;
        uint8_t firstPrint = 1;

        while (1)
        {
            HAL_Delay(50);

            float dL = IR_GetDistanceL_BG();
            float dF = IR_GetDistanceF_BG();
            float dR = IR_GetDistanceR_BG();
            uint8_t wL = IR_WallLeft_BG();
            uint8_t wF = IR_WallFront_BG();
            uint8_t wR = IR_WallRight_BG();
            uint8_t aL = IR_WallApproachingLeft_BG();
            uint8_t aF = IR_WallApproachingFront_BG();
            uint8_t aR = IR_WallApproachingRight_BG();

            uint8_t changed = firstPrint ||
                (wL != prevWallL) || (wF != prevWallF) || (wR != prevWallR) ||
                (aL != prevAppL)  || (aF != prevAppF)  || (aR != prevAppR)  ||
                (dL < 999.0f && fabsf(dL - prevDistL) > 0.2f) ||
                (dF < 999.0f && fabsf(dF - prevDistF) > 0.2f) ||
                (dR < 999.0f && fabsf(dR - prevDistR) > 0.2f);

            if (!changed) continue;

            firstPrint = 0;
            prevDistL = dL; prevDistF = dF; prevDistR = dR;
            prevWallL = wL; prevWallF = wF; prevWallR = wR;
            prevAppL  = aL; prevAppF  = aF; prevAppR  = aR;

            uint8_t anything = wL || wF || wR;

            if (!anything)
            {
                BT_SendLine("[ ALL CLEAR ]");
                continue;
            }

            BT_SendLine("------------------------------");
            if (wL)
            {
                if (aL)
                    BT_Sendf("  LEFT:  APPROACHING  %.1fin\r\n", dL);
                else
                    BT_Sendf("  LEFT:  wall detected  %.1fin\r\n", dL);
            }
            if (wF)
            {
                if (aF)
                    BT_Sendf("  FRONT: APPROACHING  %.1fin\r\n", dF);
                else
                    BT_Sendf("  FRONT: wall detected  %.1fin\r\n", dF);
            }
            if (wR)
            {
                if (aR)
                    BT_Sendf("  RIGHT: APPROACHING  %.1fin\r\n", dR);
                else
                    BT_Sendf("  RIGHT: wall detected  %.1fin\r\n", dR);
            }
            BT_SendLine("------------------------------");
        }
    }
#endif

#if TEST_IR_CALIB
    {
        const float distances[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f,
                                   6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
        const int numDist    = 10;
        const int numSamples = 20;

        IR_Init();
        IR_BackgroundReset();
        HAL_Delay(500);

        BT_SendLine("===== IR CALIBRATION =====");

        BT_SendLine("\r\n--- LEFT SENSOR ---");
        BT_SendLine("Point LEFT side at wall. Press button at each distance.");
        BT_SendLine("dist_in,avgRaw,minRaw,maxRaw");
        for (int i = 0; i < numDist; i++)
        {
            BT_Sendf("Place LEFT sensor %.0fin from wall. Press button.\r\n", distances[i]);
            Wait_For_Button();
            uint32_t sum = 0;
            uint16_t mn = 65535, mx = 0, r;
            for (int s = 0; s < numSamples; s++)
            {
                HAL_Delay(50);
                r = IR_GetL_BG();
                sum += r;
                if (r < mn) mn = r;
                if (r > mx) mx = r;
            }
            BT_Sendf("%.0f,%u,%u,%u\r\n", distances[i], (uint16_t)(sum/numSamples), mn, mx);
        }
        BT_SendLine("--- LEFT SENSOR DONE ---");

        BT_SendLine("\r\n--- FRONT SENSOR ---");
        BT_SendLine("Point FRONT at wall. Press button at each distance.");
        BT_SendLine("dist_in,avgRaw,minRaw,maxRaw");
        for (int i = 0; i < numDist; i++)
        {
            BT_Sendf("Place FRONT sensor %.0fin from wall. Press button.\r\n", distances[i]);
            Wait_For_Button();
            uint32_t sum = 0;
            uint16_t mn = 65535, mx = 0, r;
            for (int s = 0; s < numSamples; s++)
            {
                HAL_Delay(50);
                r = IR_GetF_BG();
                sum += r;
                if (r < mn) mn = r;
                if (r > mx) mx = r;
            }
            BT_Sendf("%.0f,%u,%u,%u\r\n", distances[i], (uint16_t)(sum/numSamples), mn, mx);
        }
        BT_SendLine("--- FRONT SENSOR DONE ---");

        BT_SendLine("\r\n--- RIGHT SENSOR ---");
        BT_SendLine("Point RIGHT side at wall. Press button at each distance.");
        BT_SendLine("dist_in,avgRaw,minRaw,maxRaw");
        for (int i = 0; i < numDist; i++)
        {
            BT_Sendf("Place RIGHT sensor %.0fin from wall. Press button.\r\n", distances[i]);
            Wait_For_Button();
            uint32_t sum = 0;
            uint16_t mn = 65535, mx = 0, r;
            for (int s = 0; s < numSamples; s++)
            {
                HAL_Delay(50);
                r = IR_GetR_BG();
                sum += r;
                if (r < mn) mn = r;
                if (r > mx) mx = r;
            }
            BT_Sendf("%.0f,%u,%u,%u\r\n", distances[i], (uint16_t)(sum/numSamples), mn, mx);
        }
        BT_SendLine("--- RIGHT SENSOR DONE ---");

        BT_SendLine("\r\n===== IR CALIBRATION DONE =====");
        BT_SendLine("Use avgRaw values to rebuild lookup table in sensors.c");
        while (1) { HAL_Delay(1000); }
    }
#endif

#if TEST_POSITION_CONTROL
    Wait_For_Button();

    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();

    BT_SendLine("===== POSITION CONTROL READY =====");
    BT_SendLine("Place robot safely on ground. Press button.");
    Wait_For_Button();

    Run_PositionControlTest(-76.2f, 800.0f);
#endif

#if TEST_BNO_ASYNC
    Wait_For_Button();
    BT_SendLine("===== BNO ASYNC TEST READY =====");
    Wait_For_Button();
    Run_BnoAsyncTest();
#endif


#if TEST_STEP2
    Wait_For_Button();
    BT_SendLine("===== STEP 2 TEST READY =====");
    Wait_For_Button();
    Run_Step2Test();
#endif

#if TEST_STEP3
    Wait_For_Button();
    BT_SendLine("===== STEP 3 TEST READY =====");
    Wait_For_Button();
    Run_Step3Test();
#endif

#if TEST_STEP4
    Wait_For_Button();
    BT_SendLine("===== STEP 4 TEST READY =====");
    Wait_For_Button();
    Run_Step4Test();
#endif

#if TEST_BNO_SYSTICK_WATCH
    Wait_For_Button();
    BT_Sendf("g_systickBnoEnabled=%u at Calibration_Run start\r\n", g_systickBnoEnabled);
    BT_SendLine("===== BNO SYSTICK WATCH TEST READY =====");
    BT_SendLine("Get ready to rotate the robot by hand during the test.");
    Wait_For_Button();
    Run_BnoSysTickWatchTest();
#endif

#if TEST_HEADING_DIRECTION
    Wait_For_Button();
    BT_SendLine("===== HEADING DIRECTION TEST READY =====");
    Wait_For_Button();
    Run_HeadingDirectionTest();
#endif

#if TEST_ROTATION_SWEEP
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== ROTATION SWEEP READY =====");
    BT_SendLine("Place robot on ground with room to spin. Press button.");
    Wait_For_Button();
    Run_RotationSweepTest();
#endif

#if TEST_GYRO_STEP2
    Wait_For_Button();
    BT_SendLine("===== GYRO STEP 2 TEST READY =====");
    Wait_For_Button();
    Run_GyroStep2Test();
#endif

#if TEST_BNO_CYCLE_WATCH
    Wait_For_Button();
    BT_SendLine("===== BNO CYCLE WATCH READY =====");
    BT_SendLine("Rotate the robot by hand during this test.");
    Wait_For_Button();
    Run_BnoCycleWatchTest();
#endif

#if TEST_GYRO_STEP3
    Wait_For_Button();
    g_systickBnoEnabled = 0;
    BT_SendLine("===== GYRO STEP 3 TEST READY =====");
    Wait_For_Button();
    Run_GyroStep3Test();
#endif

#if TEST_BNO_ALTERNATE
    Wait_For_Button();
    g_systickBnoEnabled = 0;
    BT_SendLine("===== BNO ALTERNATE TEST READY =====");
    Wait_For_Button();
    Run_BnoAlternateTest();
#endif

#if TEST_SPIN_TURN
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== SPIN TURN READY =====");
    Wait_For_Button();
    Run_SpinTurnTest(180.0f);
#endif

#if TEST_ROTATION_DECEL
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== ROTATION DECEL TEST READY =====");
    Wait_For_Button();
    Run_RotationAccelDecelTest(TURN_RIGHT);
#endif

#if TEST_STRAIGHT_TURN_STRESS
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== STRAIGHT + TURN STRESS TEST READY =====");
    BT_SendLine("Make sure robot has room to keep moving/turning indefinitely.");
    Wait_For_Button();
    Run_StraightAndTurnStressTest();
#endif

#if TEST_STEERING_ADJUSTMENT
    Wait_For_Button();
    BT_SendLine("===== STEERING TEST READY =====");
    Wait_For_Button();
    Run_SteeringAdjustmentTest();
#endif

#if TEST_IR_LIVE_MONITOR
    Wait_For_Button();
    BT_SendLine("===== IR LIVE MONITOR READY =====");
    Wait_For_Button();
    Run_IRLiveMonitorTest();
#endif

#if TEST_SYSTICK_TIMING
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== SYSTICK TIMING READY =====");
    Wait_For_Button();
    Run_SysTickTimingTest();
#endif

#if TEST_FLASHLIGHT_RECEIVER
    Wait_For_Button();
    BT_SendLine("===== FLASHLIGHT TEST READY =====");
    Wait_For_Button();
    Run_FlashlightReceiverTest();
#endif

#if TEST_TEN_INCH_LOOP
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== TEN INCH LOOP READY =====");
    Wait_For_Button();
    Run_TenInchLoopTest(10);
#endif

#if TEST_TEN_TURN_LOOP
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== TEN TURN LOOP READY =====");
    Wait_For_Button();
    Run_TenTurnLoopTest(170.0f, 12);   // or your empirically-corrected 87.0f
#endif

#if TEST_MOVE_THEN_TURN
    Wait_For_Button();

    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();

    BT_SendLine("===== MOVE + TURN READY =====");
    BT_SendLine("Place robot safely on ground. Press button.");
    Wait_For_Button();

    // 1. Run the straight move (captures s_residualHeadingErrorDeg)
    Run_PositionControlTest(254.0f, 800.0f);

    g_motionControlEnabled = 0;
    HAL_Delay(200);

    // 2. Base turn target (Left = -90.0f)
    float baseTurnAngle = -86.0f;
    float turnTarget = baseTurnAngle;

    // Only compensate if the heading error exceeds the spin tolerance
    if (fabsf(s_residualHeadingErrorDeg) > ROT_FINAL_ANGLE_TOLERANCE_DEG)
    {
        turnTarget = baseTurnAngle - s_residualHeadingErrorDeg;
        BT_Sendf("Residual exceeds tolerance (%.2f > %.2f) -> compensated target: %.2f\r\n",
                 fabsf(s_residualHeadingErrorDeg), ROT_FINAL_ANGLE_TOLERANCE_DEG, turnTarget);
    }
    else
    {
        BT_Sendf("Residual within tolerance (%.2f <= %.2f) -> nominal target: %.2f\r\n",
                 fabsf(s_residualHeadingErrorDeg), ROT_FINAL_ANGLE_TOLERANCE_DEG, turnTarget);
    }

    // 3. Run the spin turn
    Run_SpinTurnTest(turnTarget);
#endif

#if TEST_EMITTER_HOLD_ON
    Wait_For_Button();
    BT_SendLine("===== EMITTER HOLD-ON READY =====");
    Wait_For_Button();
    Run_EmitterHoldOnTest();
#endif

#if TEST_MOVE_THEN_RESQUARE
    Wait_For_Button();
    Encoder_Init();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== MOVE + RESQUARE TEST READY =====");
    Wait_For_Button();
    Run_MoveThenResquareTest();
#endif

#if TEST_FAST_HANDOFF
    Wait_For_Button();
    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== FAST HANDOFF READY =====");
    BT_SendLine("Clear ~50 cm of straight floor ahead. Press button.");
    Wait_For_Button();
    Run_FastHandoffTest();
#endif

#if TEST_SMOOTH_TURN
    Wait_For_Button();
    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== SMOOTH TURN READY =====");
    BT_SendLine("Clear ~1.5 m ahead and ~0.6 m to the turn side, no walls. Press button.");
    Wait_For_Button();
    Run_SmoothTurnTest();
#endif

#if TEST_SMOOTH_TURN_DETAIL
    Wait_For_Button();
    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== SMOOTH TURN DETAIL READY =====");
    BT_SendLine("Clear ~1.5 m ahead and ~0.6 m to the turn side, no walls. Press button.");
    Wait_For_Button();
    Run_SmoothTurnDetailTest();
#endif

#if TEST_PATH
    Wait_For_Button();
    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== PATH TEST READY =====");
    BT_SendLine("Clear ~0.9 m x 0.9 m ahead and to the turn side, no walls. Press button.");
    Wait_For_Button();
    Run_PathTest();
#endif

#if TEST_PATH_LOOP
    Wait_For_Button();
    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== LOOP TEST READY =====");
    BT_SendLine("Clear ~1.0 m x 1.0 m, no walls. Start at a corner-cell center facing along a side. Press button.");
    Wait_For_Button();
    Run_LoopTest();
#endif

#if TEST_SQUARE
    Wait_For_Button();
    Encoder_Init();
    Encoder_ResetAll();
    Motor_Init();
    Motor_Enable();
    HAL_Delay(500);
    BT_SendLine("===== SQUARE TEST READY =====");
    BT_SendLine("Box on the RIGHT, ~127 mm from its wall, at the center of the middle cell of a side, facing along it. Press button.");
    Wait_For_Button();
    Run_SquareTest();
#endif
}
