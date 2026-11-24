#include "bno055.h"

#ifndef GYRO_RST_GPIO_Port
#error "GYRO_RST_GPIO_Port not defined. MQKE SURE PA12 is labeled GYRO_RST in CubeMX."
#endif

#ifndef GYRO_RST_Pin
#error "GYRO_RST_Pin not defined. Make sure PA12 is labeled GYRO_RST in CubeMX."
#endif

static BNO055_Status BNO055_WriteByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value){
	if(HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY) != HAL_OK){
		return BNO055_ERR_I2C;
	}
	return BNO055_OK;
}

static BNO055_Status BNO055_ReadBytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len){
	if (HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) != HAL_OK){
		return BNO055_ERR_I2C;
	}
	return BNO055_OK;
}

BNO055_Status BNO055_Reset(void){
	HAL_GPIO_WritePin(GYRO_RST_GPIO_Port, GYRO_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GYRO_RST_GPIO_Port, GYRO_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(700);

	return BNO055_OK;
}

BNO055_Status BNO055_ReadChipID(I2C_HandleTypeDef *hi2c, uint8_t *chip_id){
	return BNO055_ReadBytes(hi2c, BNO055_REG_CHIP_ID, chip_id, 1);
}

BNO055_Status BNO055_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode){
	BNO055_Status st;

	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
	if (st != BNO055_OK) return st;
	HAL_Delay(25);

	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, mode);
	if (st != BNO055_OK) return st;
	HAL_Delay(30);

	return BNO055_OK;
}

BNO055_Status BNO055_Init(I2C_HandleTypeDef *hi2c){
	BNO055_Status st;
	uint8_t chip_id = 0;

	HAL_GPIO_WritePin(GYRO_RST_GPIO_Port, GYRO_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	st = BNO055_ReadChipID(hi2c, &chip_id);
	if (st != BNO055_OK) return st;
	if (chip_id != BNO055_CHIP_ID_VALUE) return BNO055_ERR_CHIP_ID;

	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
	if (st != BNO055_OK) return st;
	HAL_Delay(25);

	st = BNO055_WriteByte(hi2c, BNO055_REG_PAGE_ID, 0X00);
	if (st != BNO055_OK) return st;

	st = BNO055_WriteByte(hi2c, BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL);
	if (st != BNO055_OK) return st;
	HAL_Delay(10);

	st = BNO055_WriteByte(hi2c, BNO055_REG_UNIT_SEL, 0x00);
	if (st != BNO055_OK) return st;

	st = BNO055_WriteByte(hi2c, BNO055_REG_SYS_TRIGGER, 0x00);
	if (st != BNO055_OK) return st;
	HAL_Delay(10);

	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, BNO055_MODE_NDOF);
	if (st != BNO055_OK) return st;
	HAL_Delay(30);

	return BNO055_OK;
}

BNO055_Status BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, BNO055_Euler *euler){
	uint8_t buf[6];
	int16_t raw_heading;
	int16_t raw_roll;
	int16_t raw_pitch;
	BNO055_Status st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_EUL_HEADING_LSB, buf, 6);
	if (st != BNO055_OK) return st;

	raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
	raw_roll = (int16_t)((buf[3] << 8) | buf[2]);
	raw_pitch = (int16_t)((buf[5] << 8) | buf[4]);

	euler->heading_deg = raw_heading / 16.0f;
	euler->roll_deg = raw_roll / 16.0f;
	euler->pitch_deg = raw_pitch / 16.0f;

	return BNO055_OK;
}

BNO055_Status BNO055_ReadHeading(I2C_HandleTypeDef *hi2c, float *heading_deg){
	BNO055_Euler euler;
	BNO055_Status st = BNO055_ReadEuler(hi2c, &euler);

	if (st != BNO055_OK) return st;

	*heading_deg = euler.heading_deg;
	return BNO055_OK;
}

BNO055_Status BNO055_ReadCalibration(I2C_HandleTypeDef *hi2c, BNO055_Calib *calib){
	uint8_t raw = 0;
	BNO055_Status st = BNO055_ReadBytes(hi2c, BNO055_REG_CALIB_STAT, &raw, 1);

	if (st != BNO055_OK) return st;

	calib->raw = raw;
	calib->sys = (raw >> 6) & 0x03;
	calib->gyro = (raw >> 4) & 0x03;
	calib->accel = (raw >> 2) & 0x03;
	calib->mag = raw & 0x03;

	return BNO055_OK;
}

BNO055_Status BNO055_ReadGyro(I2C_HandleTypeDef *hi2c, BNO055_Gyro *gyro){
	uint8_t buf[6];
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;
	BNO055_Status st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_GYR_DATA_X_LSB, buf, 6);
	if (st != BNO055_OK) return st;

	raw_x = (int16_t)((buf[1] << 8) | buf[0]);
	raw_y = (int16_t)((buf[3] << 8) | buf[2]);
	raw_z = (int16_t)((buf[5] << 8) | buf[4]);

	gyro->x_dps = raw_x / 16.0f;
	gyro->y_dps = raw_y / 16.0f;
	gyro->z_dps = raw_z / 16.0f;

	return BNO055_OK;
}

BNO055_Status BNO055_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *sys_stat, uint8_t *sys_err){
	BNO055_Status st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_SYS_STAT, sys_stat, 1);
	if (st != BNO055_OK) return st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_SYS_ERR, sys_err, 1);
	if (st != BNO055_OK) return st;

	return BNO055_OK;
}

float BNO055_AngleError(float target_deg, float current_deg){
	float err = target_deg - current_deg;

	while (err > 180.0f) err -= 360.0f;
	while (err < -180.0f) err += 360.0f;

	return err;
}

/* ── Non-blocking async heading + gyro read, with request tracking, watchdog + bus recovery ───────── */

typedef enum { BNO_REQUEST_NONE, BNO_REQUEST_HEADING, BNO_REQUEST_GYRO } BnoRequestType;
static volatile BnoRequestType currentBnoRequest = BNO_REQUEST_NONE;

static volatile uint8_t  asyncHeadingPending = 0;
static uint8_t            asyncHeadingBuf[6];
static volatile float     asyncLatestHeading = 0.0f;
static I2C_HandleTypeDef *asyncHi2c = NULL;
static volatile uint32_t  asyncErrorCount = 0;
static volatile uint32_t  asyncRecoveryCount = 0;
static volatile uint32_t  asyncStartTick = 0;
static volatile uint32_t  asyncStartAttempts = 0;
static volatile uint32_t  asyncStartFailures = 0;
static volatile uint32_t  asyncCallbackFires = 0;

static volatile uint8_t  asyncGyroPending = 0;
static uint8_t            asyncGyroBuf[6];
static volatile float     asyncLatestGyroZ = 0.0f;
static I2C_HandleTypeDef *asyncGyroHi2c = NULL;
static volatile uint32_t  asyncGyroStartTick = 0;

#define ASYNC_I2C_TIMEOUT_MS 20

void BNO055_RecoverI2CBus(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_DeInit(hi2c);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = GYRO_SCL_Pin;
    HAL_GPIO_Init(GYRO_SCL_GPIO_Port, &gpio);

    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GYRO_SCL_GPIO_Port, GYRO_SCL_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GYRO_SCL_GPIO_Port, GYRO_SCL_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    HAL_I2C_Init(hi2c);
    asyncRecoveryCount++;
    currentBnoRequest = BNO_REQUEST_NONE;
}

BNO055_Status BNO055_StartAsyncHeadingRead(I2C_HandleTypeDef *hi2c)
{
    if (asyncHeadingPending)
    {
        if ((HAL_GetTick() - asyncStartTick) > ASYNC_I2C_TIMEOUT_MS)
        {
            BNO055_RecoverI2CBus(hi2c);
            asyncHeadingPending = 0;
        }
        else
        {
            return BNO055_OK;
        }
    }

    asyncHi2c = hi2c;
    asyncHeadingPending = 1;
    asyncStartTick = HAL_GetTick();
    asyncStartAttempts++;
    currentBnoRequest = BNO_REQUEST_HEADING;

    if (HAL_I2C_Mem_Read_IT(hi2c, BNO055_I2C_ADDR, BNO055_REG_EUL_HEADING_LSB,
                             I2C_MEMADD_SIZE_8BIT, asyncHeadingBuf, 6) != HAL_OK)
    {
        asyncHeadingPending = 0;
        asyncStartFailures++;
        currentBnoRequest = BNO_REQUEST_NONE;
        return BNO055_ERR_I2C;
    }

    return BNO055_OK;
}

BNO055_Status BNO055_StartAsyncGyroRead(I2C_HandleTypeDef *hi2c)
{
    if (asyncGyroPending)
    {
        if ((HAL_GetTick() - asyncGyroStartTick) > ASYNC_I2C_TIMEOUT_MS)
        {
            BNO055_RecoverI2CBus(hi2c);
            asyncGyroPending = 0;
        }
        else
        {
            return BNO055_OK;
        }
    }

    asyncGyroHi2c = hi2c;
    asyncGyroPending = 1;
    asyncGyroStartTick = HAL_GetTick();
    currentBnoRequest = BNO_REQUEST_GYRO;

    if (HAL_I2C_Mem_Read_IT(hi2c, BNO055_I2C_ADDR, BNO055_REG_GYR_DATA_X_LSB,
                             I2C_MEMADD_SIZE_8BIT, asyncGyroBuf, 6) != HAL_OK)
    {
        asyncGyroPending = 0;
        currentBnoRequest = BNO_REQUEST_NONE;
        return BNO055_ERR_I2C;
    }

    return BNO055_OK;
}

uint8_t BNO055_AsyncHeadingReady(void) { return !asyncHeadingPending; }
float   BNO055_GetAsyncHeading(void)   { return asyncLatestHeading; }
uint8_t BNO055_AsyncGyroReady(void)    { return !asyncGyroPending; }
float   BNO055_GetAsyncGyroZ(void)     { return asyncLatestGyroZ; }

uint32_t BNO055_GetAsyncErrorCount(void)      { return asyncErrorCount; }
uint32_t BNO055_GetAsyncRecoveryCount(void)   { return asyncRecoveryCount; }
uint32_t BNO055_GetAsyncStartAttempts(void)   { return asyncStartAttempts; }
uint32_t BNO055_GetAsyncStartFailures(void)   { return asyncStartFailures; }
uint32_t BNO055_GetAsyncCallbackFires(void)   { return asyncCallbackFires; }

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (currentBnoRequest == BNO_REQUEST_HEADING)
    {
        int16_t raw_heading = (int16_t)((asyncHeadingBuf[1] << 8) | asyncHeadingBuf[0]);
        asyncLatestHeading = raw_heading / 16.0f;
        asyncHeadingPending = 0;
        asyncCallbackFires++;
    }
    else if (currentBnoRequest == BNO_REQUEST_GYRO)
    {
        int16_t raw_z = (int16_t)((asyncGyroBuf[5] << 8) | asyncGyroBuf[4]);
        asyncLatestGyroZ = raw_z / 16.0f;
        asyncGyroPending = 0;
    }
    currentBnoRequest = BNO_REQUEST_NONE;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (currentBnoRequest == BNO_REQUEST_HEADING)
    {
        asyncErrorCount++;
        asyncHeadingPending = 0;
    }
    else if (currentBnoRequest == BNO_REQUEST_GYRO)
    {
        asyncErrorCount++;
        asyncGyroPending = 0;
    }
    currentBnoRequest = BNO_REQUEST_NONE;
}
