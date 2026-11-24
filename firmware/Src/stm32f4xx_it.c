/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensors.h"
#include "motion_guard.h"
#include "battery.h"
#include "ramp_profile.h"
#include "motion_controller.h"
#include "motion_state.h"
#include "encoder.h"
#include "bno055.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern RampProfile forwardProfile;
extern RampProfile rotationProfile;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
#define ROT_KINETIC_MIN_SPEED          23.0f   // matches rotationProfile's kineticMinSpeed
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  uint32_t cycleStart = DWT->CYCCNT;

  g_systickTickCount++;

  // RUN CLOCK: counts from the button press (main.c). When the limit passes, latch g_runExpired.
  if (g_runClockArmed && !g_runExpired)
  {
      if ((HAL_GetTick() - g_runStartTick) >= RUN_TIME_LIMIT_MS)
      {
          g_runExpired = 1;
      }
  }
  IR_BackgroundServiceStep();
  MotionGuard_Update();
  Encoder_Update();

  RampProfile_Update(&forwardProfile);
  RampProfile_Update(&rotationProfile);

  Battery_Update();

  typedef enum { BNO_CYCLE_HEADING, BNO_CYCLE_GYRO } BnoCycleState;
  static BnoCycleState bnoCycleState = BNO_CYCLE_HEADING;

  if (g_systickBnoEnabled)
  {
      if (bnoCycleState == BNO_CYCLE_HEADING)
      {
          if (BNO055_AsyncHeadingReady())
          {
              g_currentHeading = BNO055_GetAsyncHeading();
              bnoCycleState = BNO_CYCLE_GYRO;
              BNO055_StartAsyncGyroRead(&hi2c1);
          }
          else
          {
              BNO055_StartAsyncHeadingRead(&hi2c1);
          }
      }
      else
      {
          if (BNO055_AsyncGyroReady())
          {
              g_currentGyroZ = BNO055_GetAsyncGyroZ();
              bnoCycleState = BNO_CYCLE_HEADING;
              BNO055_StartAsyncHeadingRead(&hi2c1);
          }
          else
          {
              BNO055_StartAsyncGyroRead(&hi2c1);
          }
      }
  }



  // --- Rotation: keep pushing at kinetic speed until REAL measured angle
  // confirms the turn is done (not just RampProfile's ideal "finished" flag),
  // then apply a real hard brake at the transition. Fully inert unless
  // g_spinTurnActive is set, so straight-line motion is never affected.
  static uint8_t rotationOverridePrevActive = 0;
  uint8_t rotationOverrideActive = 0;

  float rotSpeed = RampProfile_GetSpeed(&rotationProfile);

  if (g_spinTurnActive && rotSpeed == 0.0f && RampProfile_IsFinished(&rotationProfile))
  {
      float remainingReal = BNO055_AngleError(g_rotationTargetHeading, g_currentHeading);

      if (fabsf(remainingReal) > ROT_FINAL_ANGLE_TOLERANCE_DEG)
      {
          rotSpeed = (remainingReal > 0.0f) ? ROT_KINETIC_MIN_SPEED : -ROT_KINETIC_MIN_SPEED;
          rotationOverrideActive = 1;
          g_rotationBraked = 0;
      }
  }

  if (rotationOverridePrevActive && !rotationOverrideActive)
  {
      Motor_Stop(MOTOR_BRAKE);
      g_rotationBraked = 1;
  }
  rotationOverridePrevActive = rotationOverrideActive;

  float steeringAdjustment = 0.0f;

  g_forwardSpeed_mm_s = RampProfile_GetSpeed(&forwardProfile);

  if (!g_spinTurnActive && !(g_irSteerOffInArc && !RampProfile_IsFinished(&rotationProfile)))
  {
      steeringAdjustment = MotionController_CalculateSteeringAdjustment();
  }

  // RUN CLOCK: time is up -> cut the motion control and brake, every tick. This is a latch: the solver switches
  // g_motionControlEnabled back on when it starts a move, and this switches it off again on the next tick.
  if (g_runExpired)
  {
      g_motionControlEnabled = 0;
      g_robotIsMoving = 0;
      Motor_Stop(MOTOR_BRAKE);
  }

  if (g_motionControlEnabled && !g_rotationBraked)
  {
      MotionController_Update(RampProfile_GetSpeed(&forwardProfile),
                               RampProfile_GetPosition(&forwardProfile),
                               rotSpeed,
                               RampProfile_GetPosition(&rotationProfile),
                               g_currentHeading,
                               steeringAdjustment,
                               g_robotIsMoving);
  }


  uint32_t cycleEnd = DWT->CYCCNT;
  g_systickLastCycles = cycleEnd - cycleStart;
  if (g_systickLastCycles > g_systickMaxCycles){
	  g_systickMaxCycles = g_systickLastCycles;
  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GYRO_INT_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
