#include "main.h"
#include "sensors.h"
#include "motion_controller.h"

extern ADC_HandleTypeDef hadc1;

// ── Pin mapping (from IOC) ────────────────────────────────────────────────────
// IR_EMITTER_L  → PA5  (GPIO output)
// IR_EMITTER_F  → PA4  (GPIO output)
// IR_EMITTER_R  → PC13 (GPIO output)
// IR_RECEIVER_L → PA7  (ADC1 channel 7)
// IR_RECEIVER_F → PA4  (ADC1 channel 4)
// IR_RECEIVER_R → PA1  (ADC1 channel 1)
// ─────────────────────────────────────────────────────────────────────────────

#define IR_FRONT_RECEIVER_ADC_CH  ADC_CHANNEL_4   // PA4
#define IR_LEFT_RECEIVER_ADC_CH   ADC_CHANNEL_7   // PA7
#define IR_RIGHT_RECEIVER_ADC_CH  ADC_CHANNEL_1   // PA1

// ── Emitter pin mapping — symbolic, so left/right can be swapped as a unit ────
// for diagnostic testing. Currently UNSWAPPED (matches true physical wiring).
// ─────────────────────────────────────────────────────────────────────────────
#define IR_LEFT_EMITTER_PORT   IR_EMITTER_L_GPIO_Port
#define IR_LEFT_EMITTER_PIN    IR_EMITTER_L_Pin
#define IR_RIGHT_EMITTER_PORT  IR_EMITTER_R_GPIO_Port
#define IR_RIGHT_EMITTER_PIN   IR_EMITTER_R_Pin

// ── Per-sensor calibration data ───────────────────────────────────────────────
// Calibrated 2026-10-23 on the styrofoam maze walls (right 10 in point estimated = 21). Raw ADC (ambient-subtracted) at each distance.
// Each entry: { raw_adc, distance_inches }
// ─────────────────────────────────────────────────────────────────────────────

typedef struct { uint16_t raw; float dist_in; } IR_CalPoint;

static const IR_CalPoint cal_left[] = {
    { 2282, 1.0f },
    { 1045, 2.0f },
    {  495, 3.0f },
    {  145, 4.0f },
    {   66, 5.0f },
    {   40, 6.0f },
    {   21, 7.0f },
    {   11, 8.0f },
    {    5, 9.0f },
    {    1, 10.0f },
};

static const IR_CalPoint cal_front[] = {
    { 2572, 1.0f },
    { 1296, 2.0f },
    {  666, 3.0f },
    {  438, 4.0f },
    {  215, 5.0f },
    {  183, 6.0f },
    {  132, 7.0f },
    {  110, 8.0f },
    {   99, 9.0f },
    {   94, 10.0f },
};

static const IR_CalPoint cal_right[] = {
    {  277, 1.0f },
    {  132, 2.0f },
    {   75, 3.0f },
    {   52, 4.0f },
    {   42, 5.0f },
    {   32, 6.0f },
    {   28, 7.0f },
    {   26, 8.0f },
    {   23, 9.0f },
    {   21, 10.0f },
};

#define CAL_SIZE  10
#define IR_BG_FILTER_ALPHA  0.30f

// "No wall" comes out of IR_Interpolate as 999. Averaging a 999 into the filtered
// distance makes it take ~85 mm of travel to come back down, so the wall flag was
// late. Cap what goes INTO the filter. Keep this above IR_WALL_OFF_IN.
#define IR_FILTER_CAP_IN    12.0f

// ── Fast release (side sensors only) ──────────────────────────────────────────
// The slow filter above is good on a straight but holds on to a wall for 2-4
// samples after the wall has ended, so the wall flag is still set at the next
// cell's decision point. With this on, when a side sensor reads "no wall"
// (at or above IR_SIDE_WALL_OFF_IN) on IR_BG_RELEASE_SAMPLES samples in a row,
// the filtered reading jumps IR_BG_RELEASE_ALPHA of the way to the new reading.
// Walls approaching, or present, still use the slow filter.
// Set IR_BG_FAST_RELEASE to 0 to go back to the old behaviour.
#define IR_BG_FAST_RELEASE      1
#define IR_BG_RELEASE_ALPHA     0.80f
#define IR_BG_RELEASE_SAMPLES   2

static float IR_Interpolate(uint16_t raw, const IR_CalPoint *cal, int n)
{
    if (raw <= cal[n-1].raw) return 999.0f;
    if (raw >= cal[0].raw) return cal[0].dist_in;

    for (int i = 0; i < n - 1; i++)
    {
        if (raw <= cal[i].raw && raw >= cal[i+1].raw)
        {
            float t = (float)(cal[i].raw - raw) /
                      (float)(cal[i].raw - cal[i+1].raw);
            return cal[i].dist_in + t * (cal[i+1].dist_in - cal[i].dist_in);
        }
    }

    return 999.0f;
}

static IR_Readings readings    = {0};
static IR_Readings bg_readings = {0};

static float    bg_dist_l = 999.0f;
static float    bg_dist_f = 999.0f;
static float    bg_dist_r = 999.0f;

static uint16_t bg_active_l = 0;
static uint16_t bg_active_f = 0;
static uint16_t bg_active_r = 0;

// Count of consecutive "no wall" samples, per side sensor (fast release).
static uint8_t bg_open_count_l = 0;
static uint8_t bg_open_count_r = 0;

static uint8_t wall_left_state    = 0;
static uint8_t wall_front_state   = 0;
static uint8_t wall_right_state   = 0;
static uint8_t approach_left_state  = 0;
static uint8_t approach_front_state = 0;
static uint8_t approach_right_state = 0;

static uint8_t bg_wall_left_state    = 0;
static uint8_t bg_wall_front_state   = 0;
static uint8_t bg_wall_right_state   = 0;
static uint8_t bg_approach_left_state  = 0;
static uint8_t bg_approach_front_state = 0;
static uint8_t bg_approach_right_state = 0;

static inline void emitter_on(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static inline void emitter_off(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void all_emitters_off(void)
{
    emitter_off(IR_LEFT_EMITTER_PORT, IR_LEFT_EMITTER_PIN);
    emitter_off(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);
    emitter_off(IR_RIGHT_EMITTER_PORT, IR_RIGHT_EMITTER_PIN);
}

static volatile uint16_t adc_result = 0;
static volatile uint8_t adc_conversion_pending = 0;

static void adc_start(uint32_t channel){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();

	adc_conversion_pending = 1;
	HAL_ADC_Start_IT(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc->Instance == hadc1.Instance){
		adc_result = (uint16_t)HAL_ADC_GetValue(&hadc1);
		adc_conversion_pending = 0;
	}
}

static uint8_t hysteresis_update(float dist, uint8_t prev,
                                 float on_thresh, float off_thresh)
{
    if (prev)
        return (dist >= off_thresh) ? 0U : 1U;
    else
        return (dist <= on_thresh)  ? 1U : 0U;
}

static float bg_filter(float old_val, float new_val)
{
    return ((1.0f - IR_BG_FILTER_ALPHA) * old_val) +
           (IR_BG_FILTER_ALPHA * new_val);
}

// Side-sensor filter with fast release (see the block of defines above).
// openCount is that sensor's own counter of consecutive "no wall" samples.
static float bg_filter_side(float old_val, float new_val, uint8_t *openCount)
{
#if IR_BG_FAST_RELEASE
    if (new_val >= IR_SIDE_WALL_OFF_IN)
    {
        if (*openCount < 255U) (*openCount)++;
        if (*openCount >= IR_BG_RELEASE_SAMPLES)
        {
            return ((1.0f - IR_BG_RELEASE_ALPHA) * old_val) +
                   (IR_BG_RELEASE_ALPHA * new_val);
        }
    }
    else
    {
        *openCount = 0U;
    }
#else
    (void)openCount;
#endif
    return bg_filter(old_val, new_val);
}

float IR_ToDistance(uint16_t raw)
{
    return IR_Interpolate(raw, cal_left, CAL_SIZE);
}

float IR_ToDistanceL(uint16_t raw)
{
    return IR_Interpolate(raw, cal_left, CAL_SIZE);
}

float IR_ToDistanceF(uint16_t raw)
{
    return IR_Interpolate(raw, cal_front, CAL_SIZE);
}

float IR_ToDistanceR(uint16_t raw)
{
    return IR_Interpolate(raw, cal_right, CAL_SIZE);
}

void IR_Init(void)
{
    all_emitters_off();

    wall_left_state = wall_front_state = wall_right_state = 0;
    approach_left_state = approach_front_state = approach_right_state = 0;

    bg_wall_left_state = bg_wall_front_state = bg_wall_right_state = 0;
    bg_approach_left_state = bg_approach_front_state = bg_approach_right_state = 0;

    readings.l = readings.f = readings.r = 0;
    bg_readings.l = bg_readings.f = bg_readings.r = 0;

    bg_dist_l = bg_dist_f = bg_dist_r = 999.0f;
    bg_active_l = bg_active_f = bg_active_r = 0;
    bg_open_count_l = bg_open_count_r = 0;

    IR_BackgroundReset();
}

static uint16_t adc_read_blocking(uint32_t channel)
{
    adc_start(channel);
    while (adc_conversion_pending) { }
    return adc_result;
}

void IR_ScanAll(IR_Readings *out)
{
    uint16_t ambL = adc_read_blocking(IR_LEFT_RECEIVER_ADC_CH);
    uint16_t ambF = adc_read_blocking(IR_FRONT_RECEIVER_ADC_CH);
    uint16_t ambR = adc_read_blocking(IR_RIGHT_RECEIVER_ADC_CH);

    emitter_on(IR_LEFT_EMITTER_PORT, IR_LEFT_EMITTER_PIN);
    HAL_Delay(2);
    uint16_t litL = adc_read_blocking(IR_LEFT_RECEIVER_ADC_CH);
    emitter_off(IR_LEFT_EMITTER_PORT, IR_LEFT_EMITTER_PIN);

    emitter_on(IR_EMITTER_F_GPIO_Port,  IR_EMITTER_F_Pin);
    HAL_Delay(2);
    uint16_t litF = adc_read_blocking(IR_FRONT_RECEIVER_ADC_CH);
    emitter_off(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);

    emitter_on(IR_RIGHT_EMITTER_PORT, IR_RIGHT_EMITTER_PIN);
    HAL_Delay(2);
    uint16_t litR = adc_read_blocking(IR_RIGHT_RECEIVER_ADC_CH);
    emitter_off(IR_RIGHT_EMITTER_PORT, IR_RIGHT_EMITTER_PIN);

    readings.l = (litL > ambL) ? (litL - ambL) : 0;
    readings.f = (litF > ambF) ? (litF - ambF) : 0;
    readings.r = (litR > ambR) ? (litR - ambR) : 0;

    if (out) *out = readings;

    float dl = IR_ToDistanceL(readings.l);
    float df = IR_ToDistanceF(readings.f);
    float dr = IR_ToDistanceR(readings.r);

    wall_left_state  = hysteresis_update(dl, wall_left_state,  IR_SIDE_WALL_ON_IN, IR_SIDE_WALL_OFF_IN);
    wall_front_state = hysteresis_update(df, wall_front_state, IR_WALL_ON_IN, IR_WALL_OFF_IN);
    wall_right_state = hysteresis_update(dr, wall_right_state, IR_SIDE_WALL_ON_IN, IR_SIDE_WALL_OFF_IN);

    approach_left_state  = hysteresis_update(dl, approach_left_state,  IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
    approach_front_state = hysteresis_update(df, approach_front_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
    approach_right_state = hysteresis_update(dr, approach_right_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
}

typedef enum {
    IR_BG_LEFT_EMIT_ON = 0,
    IR_BG_LEFT_START_LIT,
	IR_BG_LEFT_WAIT_LIT,
    IR_BG_LEFT_START_AMBIENT,
	IR_BG_LEFT_WAIT_AMBIENT,
    IR_BG_FRONT_EMIT_ON,
    IR_BG_FRONT_START_LIT,
	IR_BG_FRONT_WAIT_LIT,
    IR_BG_FRONT_START_AMBIENT,
	IR_BG_FRONT_WAIT_AMBIENT,
    IR_BG_RIGHT_EMIT_ON,
    IR_BG_RIGHT_START_LIT,
	IR_BG_RIGHT_WAIT_LIT,
    IR_BG_RIGHT_START_AMBIENT,
	IR_BG_RIGHT_WAIT_AMBIENT,
	IR_BG_EXTERNAL_START,
	IR_BG_EXTERNAL_WAIT
} IR_BG_State;

static volatile uint16_t externalReadResult = 0;
static volatile uint8_t externalReadPending = 0;
static volatile uint8_t externalReadRequested = 0;
static uint32_t externalReadChannel = 0;

static volatile IR_BG_State ir_bg_state = IR_BG_LEFT_EMIT_ON;

void IR_BackgroundReset(void)
{
    ir_bg_state = IR_BG_LEFT_EMIT_ON;
    all_emitters_off();
}

void IR_BackgroundServiceStep(void)
{
    switch (ir_bg_state)
    {
    case IR_BG_LEFT_EMIT_ON:
        emitter_on(IR_LEFT_EMITTER_PORT, IR_LEFT_EMITTER_PIN);
        ir_bg_state = IR_BG_LEFT_START_LIT;
        break;

    case IR_BG_LEFT_START_LIT:
        adc_start(IR_LEFT_RECEIVER_ADC_CH);
        ir_bg_state = IR_BG_LEFT_WAIT_LIT;
        break;

    case IR_BG_LEFT_WAIT_LIT:
    {
    	if (adc_conversion_pending) break;
    	bg_active_l = adc_result;
    	emitter_off(IR_LEFT_EMITTER_PORT, IR_LEFT_EMITTER_PIN);
    	ir_bg_state = IR_BG_LEFT_START_AMBIENT;
    	break;
    }

    case IR_BG_LEFT_START_AMBIENT:
		adc_start(IR_LEFT_RECEIVER_ADC_CH);
		ir_bg_state = IR_BG_LEFT_WAIT_AMBIENT;
		break;

    case IR_BG_LEFT_WAIT_AMBIENT:
	{
		if (adc_conversion_pending) break;
		uint16_t amb = adc_result;
		bg_readings.l = (bg_active_l > amb) ? (bg_active_l - amb) : 0;
		float d_l = IR_ToDistanceL(bg_readings.l);
		if (d_l > IR_FILTER_CAP_IN) d_l = IR_FILTER_CAP_IN;
		bg_dist_l = bg_filter_side(bg_dist_l, d_l, &bg_open_count_l);
		bg_wall_left_state    = hysteresis_update(bg_dist_l, bg_wall_left_state,    IR_SIDE_WALL_ON_IN, IR_SIDE_WALL_OFF_IN);
		bg_approach_left_state = hysteresis_update(bg_dist_l, bg_approach_left_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
		ir_bg_state = IR_BG_FRONT_EMIT_ON;
		break;
	}

    case IR_BG_FRONT_EMIT_ON:
        emitter_on(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);
        ir_bg_state = IR_BG_FRONT_START_LIT;
        break;

    case IR_BG_FRONT_START_LIT:
        adc_start(IR_FRONT_RECEIVER_ADC_CH);
        ir_bg_state = IR_BG_FRONT_WAIT_LIT;
        break;

    case IR_BG_FRONT_WAIT_LIT:
    	if (adc_conversion_pending) break;
    	bg_active_f = adc_result;
    	emitter_off(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin);
    	ir_bg_state = IR_BG_FRONT_START_AMBIENT;
    	break;

    case IR_BG_FRONT_START_AMBIENT:
		adc_start(IR_FRONT_RECEIVER_ADC_CH);
		ir_bg_state = IR_BG_FRONT_WAIT_AMBIENT;
		break;

    case IR_BG_FRONT_WAIT_AMBIENT:
    {
    	if (adc_conversion_pending) break;
		uint16_t amb = adc_result;
		bg_readings.f = (bg_active_f > amb) ? (bg_active_f - amb) : 0;
		float d_f = IR_ToDistanceF(bg_readings.f);
		if (d_f > IR_FILTER_CAP_IN) d_f = IR_FILTER_CAP_IN;
		bg_dist_f = bg_filter(bg_dist_f, d_f);
		bg_wall_front_state    = hysteresis_update(bg_dist_f, bg_wall_front_state,    IR_WALL_ON_IN,     IR_WALL_OFF_IN);
		bg_approach_front_state = hysteresis_update(bg_dist_f, bg_approach_front_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);
		ir_bg_state = IR_BG_RIGHT_EMIT_ON;
		break;
    }

    case IR_BG_RIGHT_EMIT_ON:
        emitter_on(IR_RIGHT_EMITTER_PORT, IR_RIGHT_EMITTER_PIN);
        ir_bg_state = IR_BG_RIGHT_START_LIT;
        break;

    case IR_BG_RIGHT_START_LIT:
        adc_start(IR_RIGHT_RECEIVER_ADC_CH);
        ir_bg_state = IR_BG_RIGHT_WAIT_LIT;
        break;

    case IR_BG_RIGHT_WAIT_LIT:
    	if (adc_conversion_pending) break;
		bg_active_r = adc_result;
		emitter_off(IR_RIGHT_EMITTER_PORT, IR_RIGHT_EMITTER_PIN);
		ir_bg_state = IR_BG_RIGHT_START_AMBIENT;
		break;

    case IR_BG_RIGHT_START_AMBIENT:
    {
    	adc_start(IR_RIGHT_RECEIVER_ADC_CH);
		ir_bg_state = IR_BG_RIGHT_WAIT_AMBIENT;
		break;
    }

    case IR_BG_RIGHT_WAIT_AMBIENT:
	{
		if (adc_conversion_pending) break;
		uint16_t amb = adc_result;
		bg_readings.r = (bg_active_r > amb) ? (bg_active_r - amb) : 0;
		float d_r = IR_ToDistanceR(bg_readings.r);
		if (d_r > IR_FILTER_CAP_IN) d_r = IR_FILTER_CAP_IN;
		bg_dist_r = bg_filter_side(bg_dist_r, d_r, &bg_open_count_r);
		bg_wall_right_state    = hysteresis_update(bg_dist_r, bg_wall_right_state,    IR_SIDE_WALL_ON_IN, IR_SIDE_WALL_OFF_IN);
		bg_approach_right_state = hysteresis_update(bg_dist_r, bg_approach_right_state, IR_APPROACH_ON_IN, IR_APPROACH_OFF_IN);

		// Both left and right distances are as fresh as they can be together
		// at this point in the cycle — this is the correct moment to feed
		// steering's derivative calculation, rather than every 1ms SysTick
		// tick (which would mostly see stale, unchanged cached values).
		MotionController_OnFreshIRReading(bg_dist_l, bg_dist_r,
		                                    bg_wall_left_state, bg_wall_right_state,
		                                    g_forwardSpeed_mm_s);

		ir_bg_state = IR_BG_EXTERNAL_START;
		break;
	}

    case IR_BG_EXTERNAL_START:
    	if (externalReadRequested){
    		adc_start(externalReadChannel);
    		externalReadRequested = 0;
    		externalReadPending = 1;
    		ir_bg_state = IR_BG_EXTERNAL_WAIT;
    	}
    	else{
    		ir_bg_state = IR_BG_LEFT_EMIT_ON;
    	}
    	break;

    case IR_BG_EXTERNAL_WAIT:
    	if (adc_conversion_pending) break;
    	externalReadResult = adc_result;
    	externalReadPending = 0;
    	ir_bg_state = IR_BG_LEFT_EMIT_ON;
    	break;

    default:
        ir_bg_state = IR_BG_LEFT_EMIT_ON;
        all_emitters_off();
        break;
    }
}

uint8_t IR_RequestExternalRead(uint32_t channel){
	if (externalReadRequested || externalReadPending){
		return 0;
	}
	externalReadChannel = channel;
	externalReadRequested = 1;
	return 1;
}

uint8_t IR_ExternalReadReady(void){
	return (!externalReadPending && !externalReadRequested);
}

uint16_t IR_GetExternalReadResult(void){
	return externalReadResult;
}

uint16_t IR_GetL(void) { return readings.l; }
uint16_t IR_GetF(void) { return readings.f; }
uint16_t IR_GetR(void) { return readings.r; }

uint8_t IR_WallLeft(void)           { return wall_left_state; }
uint8_t IR_WallFront(void)          { return wall_front_state; }
uint8_t IR_WallRight(void)          { return wall_right_state; }
uint8_t IR_WallApproachingLeft(void)  { return approach_left_state; }
uint8_t IR_WallApproachingFront(void) { return approach_front_state; }
uint8_t IR_WallApproachingRight(void) { return approach_right_state; }

uint16_t IR_GetL_BG(void) { return bg_readings.l; }
uint16_t IR_GetF_BG(void) { return bg_readings.f; }
uint16_t IR_GetR_BG(void) { return bg_readings.r; }

float    IR_GetDistanceL_BG(void) { return bg_dist_l; }
float    IR_GetDistanceF_BG(void) { return bg_dist_f; }
float    IR_GetDistanceR_BG(void) { return bg_dist_r; }

uint8_t IR_WallLeft_BG(void)           { return bg_wall_left_state; }
uint8_t IR_WallFront_BG(void)          { return bg_wall_front_state; }
uint8_t IR_WallRight_BG(void)          { return bg_wall_right_state; }
uint8_t IR_WallApproachingLeft_BG(void)  { return bg_approach_left_state; }
uint8_t IR_WallApproachingFront_BG(void) { return bg_approach_front_state; }
uint8_t IR_WallApproachingRight_BG(void) { return bg_approach_right_state; }

uint32_t IR_GetLeftChannel(void)  { return IR_LEFT_RECEIVER_ADC_CH; }
uint32_t IR_GetRightChannel(void) { return IR_RIGHT_RECEIVER_ADC_CH; }

// sensors.c
void IR_EmitterFrontOn(void)  { emitter_on(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin); }
void IR_EmitterFrontOff(void) { emitter_off(IR_EMITTER_F_GPIO_Port, IR_EMITTER_F_Pin); }
void IR_EmitterLeftOn(void)   { emitter_on(IR_LEFT_EMITTER_PORT, IR_LEFT_EMITTER_PIN); }
void IR_EmitterLeftOff(void)  { emitter_off(IR_LEFT_EMITTER_PORT, IR_LEFT_EMITTER_PIN); }
void IR_EmitterRightOn(void)  { emitter_on(IR_RIGHT_EMITTER_PORT, IR_RIGHT_EMITTER_PIN); }
void IR_EmitterRightOff(void) { emitter_off(IR_RIGHT_EMITTER_PORT, IR_RIGHT_EMITTER_PIN); }
