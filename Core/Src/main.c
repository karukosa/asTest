/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * This code was created by Vu Nam Hung aka Karukosa
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button_input.h"
#include "max31865.h"
#include "tm1637.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static ButtonInput buttonHeater;
static ButtonInput buttonPump;
static ButtonInput buttonVale;
static ButtonInput buttonAuto;
static ButtonInput buttonStop;

static uint8_t autoRunning = 0U;
static uint8_t heaterOn = 0U;
static uint8_t pumpOn = 0U;
static uint8_t valeOn = 0U;

static Max31865Handle pt100;
static TM1637Handle tm1637;
static uint32_t lastTempReadTick = 0U;
static uint8_t tempSensorReady = 0U;
static int16_t latestTemperatureTenths = 0;
static uint8_t latestTemperatureValid = 0U;

typedef enum {
  AUTO_PHASE_IDLE = 0,
  AUTO_PHASE_FILL_WATER,
  AUTO_PHASE_AIR_REMOVAL,
  AUTO_PHASE_HEATING_RAMP,
  AUTO_PHASE_STERILIZATION_HOLD,
  AUTO_PHASE_EXHAUST,
  AUTO_PHASE_DRYING,
  AUTO_PHASE_COMPLETE
} AutoPhase;

static AutoPhase autoPhase = AUTO_PHASE_IDLE;
static uint32_t autoPhaseStartTick = 0U;
static uint32_t autoLastToggleTick = 0U;
static uint8_t autoPulseCount = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void SetHeater(uint8_t on);
static void SetPump(uint8_t on);
static void SetVale(uint8_t on);
static void SetAutoIndicator(uint8_t on);
static void SetStopIndicator(uint8_t on);
static void UpdateActuatorIndicators(void);
static void HandleManualMode(void);
static void HandleAutoMode(uint32_t now);
static void UpdateTemperatureDisplay(uint32_t now);
static void AutoEnterPhase(AutoPhase nextPhase, uint32_t now);
static void AutoResetCycle(void);
static uint8_t IsStartAutoRequested(void);
static uint8_t IsStopRequested(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SetHeater(uint8_t on)
{
  GPIO_PinState pinState = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(SSR_HEATER_GPIO_Port, SSR_HEATER_Pin, pinState);
  HAL_GPIO_WritePin(LED_HEATER_GPIO_Port, LED_HEATER_Pin, pinState);
}

static void SetPump(uint8_t on)
{
  GPIO_PinState pinState = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(RELAY_PUMP_GPIO_Port, RELAY_PUMP_Pin, pinState);
  HAL_GPIO_WritePin(LED_PUMP_GPIO_Port, LED_PUMP_Pin, pinState);
}

static void SetVale(uint8_t on)
{
  GPIO_PinState pinState = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(RELAY_VALE_GPIO_Port, RELAY_VALE_Pin, pinState);
  HAL_GPIO_WritePin(LED_VALE_GPIO_Port, LED_VALE_Pin, pinState);
}

static void SetAutoIndicator(uint8_t on)
{
  HAL_GPIO_WritePin(LED_AUTO_GPIO_Port, LED_AUTO_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SetStopIndicator(uint8_t on)
{
  HAL_GPIO_WritePin(LED_STOP_GPIO_Port, LED_STOP_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void UpdateActuatorIndicators(void)
{
  HAL_GPIO_WritePin(LED_HEATER_GPIO_Port, LED_HEATER_Pin, heaterOn != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PUMP_GPIO_Port, LED_PUMP_Pin, pumpOn != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_VALE_GPIO_Port, LED_VALE_Pin, valeOn != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void HandleManualMode(void)
{
  if (ButtonInput_ConsumePressed(&buttonHeater) != 0U) {
    heaterOn = (heaterOn == 0U) ? 1U : 0U;
    SetHeater(heaterOn);
    SetStopIndicator(0U);
  }

  if (ButtonInput_ConsumePressed(&buttonPump) != 0U) {
    pumpOn = (pumpOn == 0U) ? 1U : 0U;
    SetPump(pumpOn);
    SetStopIndicator(0U);
  }

  if (ButtonInput_ConsumePressed(&buttonVale) != 0U) {
    valeOn = (valeOn == 0U) ? 1U : 0U;
    SetVale(valeOn);
    SetStopIndicator(0U);
  }
}

static void HandleAutoMode(uint32_t now)
{
  const uint32_t fillWaterDurationMs = 5000U;
  const uint32_t airRemovalPulseMs = 1000U;
  const uint8_t airRemovalCycles = 3U;
  const uint32_t heatingRampDurationMs = 10000U;
  const uint32_t sterilizationHoldDurationMs = 20000U;
  const uint32_t holdHeaterPeriodMs = 1000U;
  const uint32_t holdHeaterOnMs = 600U;
  const uint32_t exhaustDurationMs = 6000U;
  const uint32_t dryingPulseMs = 1200U;
  const uint8_t dryingCycles = 3U;
  uint32_t elapsed = now - autoPhaseStartTick;
  uint32_t holdPeriodPos = 0U;

  switch (autoPhase) {
    case AUTO_PHASE_IDLE:
      heaterOn = 0U;
      pumpOn = 0U;
      valeOn = 0U;
      break;

    case AUTO_PHASE_FILL_WATER:
      heaterOn = 0U;
      pumpOn = 0U;
      valeOn = 1U;
      if (elapsed >= fillWaterDurationMs) {
        AutoEnterPhase(AUTO_PHASE_AIR_REMOVAL, now);
      }
      break;

    case AUTO_PHASE_AIR_REMOVAL:
      pumpOn = 1U;
      valeOn = 0U;
      if ((now - autoLastToggleTick) >= airRemovalPulseMs) {
        autoLastToggleTick = now;
        heaterOn = (heaterOn == 0U) ? 1U : 0U;
        if (heaterOn == 0U) {
          autoPulseCount++;
          if (autoPulseCount >= airRemovalCycles) {
            AutoEnterPhase(AUTO_PHASE_HEATING_RAMP, now);
          }
        }
      }
      break;

    case AUTO_PHASE_HEATING_RAMP:
      heaterOn = 1U;
      pumpOn = 0U;
      valeOn = 0U;
      if (elapsed >= heatingRampDurationMs) {
        AutoEnterPhase(AUTO_PHASE_STERILIZATION_HOLD, now);
      }
      break;

    case AUTO_PHASE_STERILIZATION_HOLD:
      pumpOn = 0U;
      valeOn = 0U;
      holdPeriodPos = elapsed % holdHeaterPeriodMs;
      heaterOn = (holdPeriodPos < holdHeaterOnMs) ? 1U : 0U;
      if (elapsed >= sterilizationHoldDurationMs) {
        AutoEnterPhase(AUTO_PHASE_EXHAUST, now);
      }
      break;

    case AUTO_PHASE_EXHAUST:
      heaterOn = 0U;
      pumpOn = 1U;
      valeOn = 0U;
      if (elapsed >= exhaustDurationMs) {
        AutoEnterPhase(AUTO_PHASE_DRYING, now);
      }
      break;

    case AUTO_PHASE_DRYING:
      pumpOn = 1U;
      valeOn = 0U;
      if ((now - autoLastToggleTick) >= dryingPulseMs) {
        autoLastToggleTick = now;
        heaterOn = (heaterOn == 0U) ? 1U : 0U;
        if (heaterOn == 0U) {
          autoPulseCount++;
          if (autoPulseCount >= dryingCycles) {
            AutoEnterPhase(AUTO_PHASE_COMPLETE, now);
          }
        }
      }
      break;

    case AUTO_PHASE_COMPLETE:
    default:
      heaterOn = 0U;
      pumpOn = 0U;
      valeOn = 0U;
      autoRunning = 0U;
      SetAutoIndicator(0U);
      SetStopIndicator(1U);
      break;
  }

  SetHeater(heaterOn);
  SetPump(pumpOn);
  SetVale(valeOn);
  UpdateActuatorIndicators();
  if (autoRunning != 0U) {
      SetAutoIndicator(1U);
      SetStopIndicator(0U);
  }
}

static void AutoEnterPhase(AutoPhase nextPhase, uint32_t now)
{
  autoPhase = nextPhase;
  autoPhaseStartTick = now;
  autoLastToggleTick = now;
  autoPulseCount = 0U;

  if (nextPhase == AUTO_PHASE_AIR_REMOVAL || nextPhase == AUTO_PHASE_DRYING) {
    heaterOn = 1U;
  }
  else {
    heaterOn = 0U;
  }
}

static void AutoResetCycle(void)
{
  autoPhase = AUTO_PHASE_IDLE;
  autoPhaseStartTick = 0U;
  autoLastToggleTick = 0U;
  autoPulseCount = 0U;
  autoRunning = 0U;
  heaterOn = 0U;
  pumpOn = 0U;
  valeOn = 0U;
  SetHeater(0U);
  SetPump(0U);
  SetVale(0U);
  SetAutoIndicator(0U);
  SetStopIndicator(0U);
}

static void StartAutoCycle(uint32_t now)
{
  AutoResetCycle();
  autoRunning = 1U;
  AutoEnterPhase(AUTO_PHASE_FILL_WATER, now);
  SetAutoIndicator(1U);
  SetStopIndicator(0U);
}

static void StopAutoCycle(void)
{
  AutoResetCycle();
  SetStopIndicator(1U);
}

static uint8_t IsStartAutoRequested(void)
{
  if (ButtonInput_ConsumePressed(&buttonAuto) != 0U) {
    return 1U;
  }

  if (ButtonInput_ConsumeRepeat(&buttonAuto) != 0U) {
    return 1U;
  }

  return 0U;
}

static uint8_t IsStopRequested(void)
{
  if (ButtonInput_ConsumePressed(&buttonStop) != 0U) {
    return 1U;
  }

  if (ButtonInput_ConsumeRepeat(&buttonStop) != 0U) {
    return 1U;
  }

  return 0U;
}

static void UpdateTemperatureDisplay(uint32_t now)
{
  const uint32_t tempReadPeriodMs = 500U;
  int16_t temperatureTenths = 0;

  if ((now - lastTempReadTick) < tempReadPeriodMs) {
    return;
  }

  lastTempReadTick = now;

  if (tempSensorReady == 0U) {
	latestTemperatureValid = 0U;
    tm1637DisplayDecimal(&tm1637, 0, 0);
    return;
  }

  if (Max31865_ReadTemperatureTenthsC(&pt100, &temperatureTenths) != 0U) {
	latestTemperatureTenths = temperatureTenths;
	latestTemperatureValid = 1U;
    tm1637DisplayDecimalTenths(&tm1637, (int)temperatureTenths);
  }
  else {
	latestTemperatureValid = 0U;
    tm1637DisplayDecimal(&tm1637, 0, 0);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  ButtonInput_Init(&buttonHeater, B_HEATER_GPIO_Port, B_HEATER_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonPump, B_PUMP_GPIO_Port, B_PUMP_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonVale, B_VALE_GPIO_Port, B_VALE_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonAuto, B_AUTO_GPIO_Port, B_AUTO_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonStop, B_STOP_GPIO_Port, B_STOP_Pin, GPIO_PIN_SET);

  AutoResetCycle();
  tm1637Init(&tm1637, TM1637_DISPLAY_1);
  tm1637SetBrightness(&tm1637, 7);

  Max31865_Init(&pt100, &hspi1, CS_MAX_GPIO_Port, CS_MAX_Pin, 430.0f, 100.0f);
  tempSensorReady = Max31865_Begin(&pt100, MAX31865_3WIRE, 1U);
  if (tempSensorReady == 0U) {
    tm1637Clear(&tm1637);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    const uint32_t debounceMs = 30U;
    const uint32_t longPressMs = 600U;
    const uint32_t repeatMs = 200U;
    const int16_t emergencyStopTemperatureTenths = 1350;

    /* Temperature sampling/display is always executed independently of mode. */
    UpdateTemperatureDisplay(now);

    ButtonInput_Update(&buttonHeater, now, debounceMs, longPressMs, repeatMs);
    ButtonInput_Update(&buttonPump, now, debounceMs, longPressMs, repeatMs);
    ButtonInput_Update(&buttonVale, now, debounceMs, longPressMs, repeatMs);
    ButtonInput_Update(&buttonAuto, now, debounceMs, longPressMs, repeatMs);
    ButtonInput_Update(&buttonStop, now, debounceMs, longPressMs, repeatMs);

    if (autoRunning == 0U && IsStartAutoRequested() != 0U) {
          StartAutoCycle(now);
    }

    if (IsStopRequested() != 0U) {
          StopAutoCycle();
    }

    if (autoRunning != 0U &&
        latestTemperatureValid != 0U &&
        latestTemperatureTenths >= emergencyStopTemperatureTenths) {
        StopAutoCycle();
    }

    if (autoRunning != 0U) {
        HandleAutoMode(now);
    }
    else {
        HandleManualMode();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|SSR_HEATER_Pin|RELAY_PUMP_Pin|RELAY_VALE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_MAX_GPIO_Port, CS_MAX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |LED_HEATER_Pin|LED_PUMP_Pin|LED_VALE_Pin|LED_AUTO_Pin
                          |Audio_RST_Pin|LED_STOP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Pin|DIO_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin SSR_HEATER_Pin RELAY_PUMP_Pin RELAY_VALE_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|SSR_HEATER_Pin|RELAY_PUMP_Pin|RELAY_VALE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin CS_MAX_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|CS_MAX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_PUMP_Pin B_VALE_Pin B_AUTO_Pin B_STOP_Pin
                           B_HEATER_Pin */
  GPIO_InitStruct.Pin = B_PUMP_Pin|B_VALE_Pin|B_AUTO_Pin|B_STOP_Pin
                          |B_HEATER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           LED_HEATER_Pin LED_PUMP_Pin LED_VALE_Pin LED_AUTO_Pin
                           Audio_RST_Pin LED_STOP_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |LED_HEATER_Pin|LED_PUMP_Pin|LED_VALE_Pin|LED_AUTO_Pin
                          |Audio_RST_Pin|LED_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin DIO_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|DIO_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
