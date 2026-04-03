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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button_input.h"
#include "tm1637.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t steamTempTenths;
  uint8_t sterilizeMinutes;
  uint8_t dryMinutes;
} ProgramConfig;

typedef enum {
  APP_MODE_IDLE = 0,
  APP_MODE_READY,
  APP_MODE_RUN_PROGRAM,
  APP_MODE_USER_EDIT
} AppMode;

typedef enum {
  USER_FIELD_TEMP = 0,
  USER_FIELD_STERILIZE,
  USER_FIELD_DRY
} UserField;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_DEBOUNCE_MS 40U
#define BUTTON_LONG_PRESS_MS 650U
#define BUTTON_REPEAT_MS 120U
#define BLINK_PERIOD_MS 350U
#define DISPLAY_SWAP_MS 1200U
#define BUZZER_SHORT_MS 300U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static TM1637Handle display1;
static TM1637Handle display2;
static ButtonInput programButtons[6];
static ButtonInput buttonUser;
static ButtonInput buttonStart;
static ButtonInput buttonSet;
static ButtonInput buttonUp;
static ButtonInput buttonDown;

static const ProgramConfig programPresets[6] = {
    {1210U, 25U, 15U}, {1250U, 30U, 20U}, {1280U, 35U, 20U},
    {1320U, 40U, 25U}, {1340U, 45U, 25U}, {1360U, 50U, 30U}};

static ProgramConfig userConfig = {1210U, 25U, 15U};
static ProgramConfig activeConfig = {1210U, 25U, 15U};
static AppMode appMode = APP_MODE_IDLE;
static UserField selectedUserField = USER_FIELD_TEMP;
static uint8_t activeProgramIndex = 0xFFU;
static uint32_t lastDisplaySwapTick = 0U;
static uint32_t programStartTick = 0U;
static uint32_t programDurationMs = 0U;

static uint8_t buzzerActive = 0U;
static uint8_t buzzerPhaseIsOn = 0U;
static uint8_t buzzerPhasesRemaining = 0U;
static uint32_t buzzerPhaseDurationMs = 0U;
static uint32_t buzzerPhaseTick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void App_InitUi(void);
static void App_UpdateButtons(void);
static void App_HandleInput(uint32_t now);
static void App_UpdateDisplay(uint32_t now);
static void App_UpdateLeds(uint32_t now);
static void App_StartProgram(uint8_t index, const ProgramConfig *cfg);
static void App_BeginRun(void);
static void App_AdjustUserField(int16_t delta);
static uint8_t App_BlinkState(uint32_t now);
static void App_DisplayStValue(uint8_t minutes);
static void App_DisplayDrValue(uint8_t minutes);
static uint8_t App_EncodeSegmentChar(char c);
static void App_RequestShortBeep(void);
static void App_RequestPatternBeep(uint8_t blinks, uint32_t phaseMs);
static void App_UpdateBuzzer(uint32_t now);
static void App_UpdateRunState(uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  App_InitUi();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    App_UpdateButtons();
    App_HandleInput(HAL_GetTick());
    App_UpdateRunState(HAL_GetTick());
    App_UpdateDisplay(HAL_GetTick());
    App_UpdateLeds(HAL_GetTick());
    App_UpdateBuzzer(HAL_GetTick());
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
  HAL_GPIO_WritePin(GPIOE, LD_C3_Pin|LD_C4_Pin|LD_C5_Pin|LD_C6_Pin
                          |LD_C7_Pin|LD_Alarm_Pin|LD_LW_Pin|LD_HW_Pin
                          |SSR_Heater_Pin|SSR_HResistor_Pin|Relay_Valve_1_Pin|Relay_Valve_2_Pin
                          |Relay_Valve_3_Pin|Relay_Valve_4_Pin|LD_C1_Pin|LD_C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRDY_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|Relay_Valve_5_Pin|CLK1_Pin|DIO1_Pin
                          |CLK2_Pin|DIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Relay_Pump_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|LD_P1_Pin|LD_P2_Pin|LD_P3_Pin
                          |LD_P4_Pin|LD_P5_Pin|LD_P6_Pin|LD_Start_Pin
                          |LD_User_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_C3_Pin LD_C4_Pin LD_C5_Pin LD_C6_Pin
                           LD_C7_Pin LD_Alarm_Pin LD_LW_Pin LD_HW_Pin
                           SSR_Heater_Pin SSR_HResistor_Pin Relay_Valve_1_Pin Relay_Valve_2_Pin
                           Relay_Valve_3_Pin Relay_Valve_4_Pin LD_C1_Pin LD_C2_Pin */
  GPIO_InitStruct.Pin = LD_C3_Pin|LD_C4_Pin|LD_C5_Pin|LD_C6_Pin
                          |LD_C7_Pin|LD_Alarm_Pin|LD_LW_Pin|LD_HW_Pin
                          |SSR_Heater_Pin|SSR_HResistor_Pin|Relay_Valve_1_Pin|Relay_Valve_2_Pin
                          |Relay_Valve_3_Pin|Relay_Valve_4_Pin|LD_C1_Pin|LD_C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_P1_Pin B_P2_Pin B_P3_Pin B_P4_Pin
                           B_P5_Pin B_P6_Pin B_Start_Pin B_Set_Pin
                           B_Up_Pin B_Down_Pin B_User_Pin */
  GPIO_InitStruct.Pin = B_P1_Pin|B_P2_Pin|B_P3_Pin|B_P4_Pin
                          |B_P5_Pin|B_P6_Pin|B_Start_Pin|B_Set_Pin
                          |B_Up_Pin|B_Down_Pin|B_User_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRDY_Pin CS_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin Relay_Valve_5_Pin CLK1_Pin DIO1_Pin
                           CLK2_Pin DIO2_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|Relay_Valve_5_Pin|CLK1_Pin|DIO1_Pin
                          |CLK2_Pin|DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L_Switch_Pin */
  GPIO_InitStruct.Pin = L_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(L_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_Pump_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD6_Pin LD_P1_Pin LD_P2_Pin LD_P3_Pin
                           LD_P4_Pin LD_P5_Pin LD_P6_Pin LD_Start_Pin
                           LD_User_Pin */
  GPIO_InitStruct.Pin = Relay_Pump_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|LD_P1_Pin|LD_P2_Pin|LD_P3_Pin
                          |LD_P4_Pin|LD_P5_Pin|LD_P6_Pin|LD_Start_Pin
                          |LD_User_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_SD_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void App_InitUi(void)
{
  tm1637Init(&display1, TM1637_DISPLAY_1);
  tm1637Init(&display2, TM1637_DISPLAY_2);
  tm1637SetBrightness(&display1, 8);
  tm1637SetBrightness(&display2, 8);

  ButtonInput_Init(&programButtons[0], B_P1_GPIO_Port, B_P1_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[1], B_P2_GPIO_Port, B_P2_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[2], B_P3_GPIO_Port, B_P3_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[3], B_P4_GPIO_Port, B_P4_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[4], B_P5_GPIO_Port, B_P5_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[5], B_P6_GPIO_Port, B_P6_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonUser, B_User_GPIO_Port, B_User_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonStart, B_Start_GPIO_Port, B_Start_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonSet, B_Set_GPIO_Port, B_Set_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonUp, B_Up_GPIO_Port, B_Up_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonDown, B_Down_GPIO_Port, B_Down_Pin, GPIO_PIN_SET);

  activeConfig = programPresets[0];
  App_UpdateDisplay(HAL_GetTick());
}

static void App_UpdateButtons(void)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0U; i < 6U; ++i) {
    ButtonInput_Update(&programButtons[i], now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  }

  ButtonInput_Update(&buttonUser, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonStart, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonSet, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonUp, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonDown, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
}

static void App_HandleInput(uint32_t now)
{
  for (uint8_t i = 0U; i < 6U; ++i) {
    if (ButtonInput_ConsumePressed(&programButtons[i]) != 0U) {
      App_StartProgram(i, &programPresets[i]);
      App_RequestShortBeep();
    }
  }

  if (ButtonInput_ConsumePressed(&buttonUser) != 0U) {
    appMode = APP_MODE_USER_EDIT;
    selectedUserField = USER_FIELD_TEMP;
    activeProgramIndex = 0xFFU;
    activeConfig = userConfig;
    lastDisplaySwapTick = now;
    App_RequestShortBeep();
  }

  if (ButtonInput_ConsumePressed(&buttonStart) != 0U) {
    if (appMode == APP_MODE_READY || appMode == APP_MODE_USER_EDIT) {
      if (appMode == APP_MODE_USER_EDIT) {
        userConfig = activeConfig;
      }
      App_BeginRun();
      App_RequestPatternBeep(2U, 500U);
    }
    else {
      App_RequestShortBeep();
    }
  }

  if (appMode == APP_MODE_USER_EDIT) {
    if (ButtonInput_ConsumePressed(&buttonSet) != 0U) {
      selectedUserField = (UserField)(((uint8_t)selectedUserField + 1U) % 3U);
      App_RequestShortBeep();
    }

    if (ButtonInput_ConsumePressed(&buttonUp) != 0U) {
      App_AdjustUserField(1);
      App_RequestShortBeep();
    }
    if (ButtonInput_ConsumeRepeat(&buttonUp) != 0U) {
      App_AdjustUserField(10);
    }

    if (ButtonInput_ConsumePressed(&buttonDown) != 0U) {
      App_AdjustUserField(-1);
      App_RequestShortBeep();
    }
    if (ButtonInput_ConsumeRepeat(&buttonDown) != 0U) {
      App_AdjustUserField(-10);
    }

    userConfig = activeConfig;
  }
}

static void App_UpdateDisplay(uint32_t now)
{
  uint8_t blinkState = App_BlinkState(now);
  uint8_t showSterilize;

  tm1637DisplayDecimalTenths(&display2, activeConfig.steamTempTenths);

  if (appMode == APP_MODE_USER_EDIT) {
    if (selectedUserField == USER_FIELD_STERILIZE) {
      if (blinkState != 0U) {
        App_DisplayStValue(activeConfig.sterilizeMinutes);
      }
      else {
        tm1637Clear(&display1);
      }
    }
    else if (selectedUserField == USER_FIELD_DRY) {
      if (blinkState != 0U) {
        App_DisplayDrValue(activeConfig.dryMinutes);
      }
      else {
        tm1637Clear(&display1);
      }
    }
    else {
      App_DisplayStValue(activeConfig.sterilizeMinutes);
    }

    if (selectedUserField == USER_FIELD_TEMP && blinkState == 0U) {
      tm1637Clear(&display2);
    }
    return;
  }

  showSterilize = (((now - lastDisplaySwapTick) / DISPLAY_SWAP_MS) % 2U) == 0U;
  if (showSterilize != 0U) {
    App_DisplayStValue(activeConfig.sterilizeMinutes);
  }
  else {
    App_DisplayDrValue(activeConfig.dryMinutes);
  }
}

static void App_UpdateLeds(uint32_t now)
{
  GPIO_TypeDef *programPorts[6] = {LD_P1_GPIO_Port, LD_P2_GPIO_Port, LD_P3_GPIO_Port,
                                   LD_P4_GPIO_Port, LD_P5_GPIO_Port, LD_P6_GPIO_Port};
  uint16_t programPins[6] = {LD_P1_Pin, LD_P2_Pin, LD_P3_Pin, LD_P4_Pin, LD_P5_Pin, LD_P6_Pin};
  GPIO_PinState blink = App_BlinkState(now) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState isUserEdit = (appMode == APP_MODE_USER_EDIT) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  for (uint8_t i = 0U; i < 6U; ++i) {
    GPIO_PinState state = GPIO_PIN_RESET;
    if (activeProgramIndex == i) {
      state = (appMode == APP_MODE_USER_EDIT) ? blink : GPIO_PIN_SET;
    }
    HAL_GPIO_WritePin(programPorts[i], programPins[i], state);
  }

  HAL_GPIO_WritePin(LD_User_GPIO_Port, LD_User_Pin, (appMode == APP_MODE_USER_EDIT) ? blink : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LD_C1_GPIO_Port, LD_C1_Pin,
                    (selectedUserField == USER_FIELD_TEMP && appMode == APP_MODE_USER_EDIT) ? blink : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD_C2_GPIO_Port, LD_C2_Pin,
                    (selectedUserField == USER_FIELD_STERILIZE && appMode == APP_MODE_USER_EDIT) ? blink : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD_C3_GPIO_Port, LD_C3_Pin,
                    (selectedUserField == USER_FIELD_DRY && appMode == APP_MODE_USER_EDIT) ? blink : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LD_Start_GPIO_Port, LD_Start_Pin, (appMode == APP_MODE_RUN_PROGRAM) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (isUserEdit == GPIO_PIN_SET) {
    HAL_GPIO_WritePin(LD_Start_GPIO_Port, LD_Start_Pin, blink);
  }
}

static void App_StartProgram(uint8_t index, const ProgramConfig *cfg)
{
  if (cfg == NULL || index >= 6U) {
    return;
  }

  activeProgramIndex = index;
  activeConfig = *cfg;
  appMode = APP_MODE_READY;
  lastDisplaySwapTick = HAL_GetTick();
}

static void App_BeginRun(void)
{
  programStartTick = HAL_GetTick();
  programDurationMs = ((uint32_t)activeConfig.sterilizeMinutes + (uint32_t)activeConfig.dryMinutes) * 60000U;
  appMode = APP_MODE_RUN_PROGRAM;
  lastDisplaySwapTick = programStartTick;
}

static void App_AdjustUserField(int16_t delta)
{
  int32_t nextValue;

  if (selectedUserField == USER_FIELD_TEMP) {
    nextValue = (int32_t)activeConfig.steamTempTenths + delta;
    if (nextValue < 1050) {
      nextValue = 1050;
    }
    if (nextValue > 1450) {
      nextValue = 1450;
    }
    activeConfig.steamTempTenths = (uint16_t)nextValue;
  }
  else if (selectedUserField == USER_FIELD_STERILIZE) {
    nextValue = (int32_t)activeConfig.sterilizeMinutes + delta;
    if (nextValue < 0) {
      nextValue = 0;
    }
    if (nextValue > 99) {
      nextValue = 99;
    }
    activeConfig.sterilizeMinutes = (uint8_t)nextValue;
  }
  else {
    nextValue = (int32_t)activeConfig.dryMinutes + delta;
    if (nextValue < 0) {
      nextValue = 0;
    }
    if (nextValue > 99) {
      nextValue = 99;
    }
    activeConfig.dryMinutes = (uint8_t)nextValue;
  }
}

static uint8_t App_BlinkState(uint32_t now)
{
  return (((now / BLINK_PERIOD_MS) % 2U) == 0U) ? 1U : 0U;
}

static void App_DisplayStValue(uint8_t minutes)
{
  uint8_t segments[4] = {0};
  segments[0] = App_EncodeSegmentChar('S');
  segments[1] = App_EncodeSegmentChar('t');
  segments[2] = App_EncodeSegmentChar((char)('0' + ((minutes / 10U) % 10U)));
  segments[3] = App_EncodeSegmentChar((char)('0' + (minutes % 10U)));
  tm1637DisplaySegments(&display1, segments);
}

static void App_DisplayDrValue(uint8_t minutes)
{
  uint8_t segments[4] = {0};
  segments[0] = App_EncodeSegmentChar('D');
  segments[1] = App_EncodeSegmentChar('r');
  segments[2] = App_EncodeSegmentChar((char)('0' + ((minutes / 10U) % 10U)));
  segments[3] = App_EncodeSegmentChar((char)('0' + (minutes % 10U)));
  tm1637DisplaySegments(&display1, segments);
}

static uint8_t App_EncodeSegmentChar(char c)
{
  switch (c) {
    case '0': return 0x3f;
    case '1': return 0x06;
    case '2': return 0x5b;
    case '3': return 0x4f;
    case '4': return 0x66;
    case '5': return 0x6d;
    case '6': return 0x7d;
    case '7': return 0x07;
    case '8': return 0x7f;
    case '9': return 0x6f;
    case 'S': return 0x6d;
    case 't': return 0x78;
    case 'D': return 0x5e;
    case 'r': return 0x50;
    default: return 0x00;
  }
}

static void App_RequestShortBeep(void)
{
  App_RequestPatternBeep(1U, BUZZER_SHORT_MS);
}

static void App_RequestPatternBeep(uint8_t blinks, uint32_t phaseMs)
{
  if (blinks == 0U || phaseMs == 0U) {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    buzzerActive = 0U;
    return;
  }

  buzzerActive = 1U;
  buzzerPhaseIsOn = 1U;
  buzzerPhasesRemaining = (uint8_t)(blinks * 2U);
  buzzerPhaseDurationMs = phaseMs;
  buzzerPhaseTick = HAL_GetTick();
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
}

static void App_UpdateBuzzer(uint32_t now)
{
  if (buzzerActive == 0U) {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    return;
  }

  if ((now - buzzerPhaseTick) < buzzerPhaseDurationMs) {
    return;
  }

  buzzerPhaseTick = now;
  if (buzzerPhasesRemaining > 0U) {
    --buzzerPhasesRemaining;
  }

  if (buzzerPhasesRemaining == 0U) {
    buzzerActive = 0U;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    return;
  }

  buzzerPhaseIsOn = (uint8_t)(1U - buzzerPhaseIsOn);
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, (buzzerPhaseIsOn != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_UpdateRunState(uint32_t now)
{
  if (appMode != APP_MODE_RUN_PROGRAM) {
    return;
  }

  if (programDurationMs == 0U || (now - programStartTick) >= programDurationMs) {
    appMode = APP_MODE_READY;
    App_RequestPatternBeep(3U, 1000U);
  }
}

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
