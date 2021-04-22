g/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "DHT.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMPERATURE_THRESHOLD 30
#define HUMID_THRESHOLD 60
#define TRUE 1
#define FALSE 0
#define TIMEOUT 1
#define SEC_IT 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//-- For measuring humidity and Temperature --
DHT_DataTypedef DHT11_Data;
double Temperature, Humidity;
//--------------------------------------------
enum TempFSMEnum {initTemp, heater, heatPump, non};
/*
 * init: initializes everything
 * heater: run the heater and fan 1 (LED 1 and 4 on; LED 2 and 5 off)
 * heatPump: run the heatPump and fan 2 (LED 2 and 5 on; LED 1 and 4 off)
 * non: runs nothing and turns off all 4 LEDs
 */
enum TempFSMEnum TempFSMState;
long heatEffect; // A counter to add fake heat into the measurement to simulate change in temp
double frequency; // The speed of Fan 1
//---- For period related things ----
uint8_t period; // The time it takes for heater and heat pump to turn on after they are off and vice versa
uint8_t periodInput; // Flag indicates if period is being input
uint16_t heatCounter; // Counts the heater and heat pump time
uint16_t humidityCounter; // Counts the blinking period of LED 3 (Fan 1)
//-----------------------------------
//------------ For serial printing -----------
uint8_t buf[50];
char uart_buf[100];
int uart_buf_len;
//--------------------------------------------
//---------------- For buttons ---------------
enum buttonStateEnum {notPressed, pressed};
/*
 * notPressed: button is released
 * pressed: considered as single pressing
 */
enum buttonStateEnum buttonPeriod;
enum buttonStateEnum buttonIncrease;
enum buttonStateEnum buttonDecrease;
__IO uint8_t firstRead[3] = {0};
__IO uint8_t secondRead[3] = {0};
//--------------------------------------------
char lcd_buf[50]; // Printing on LCD
uint8_t changeLCDMode; // Flag indicates a change from showing data to showing period
uint8_t timeoutFlag; // Flag indicates the beginning of the end of the timeout
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void initialize(void); // Initialize certain variables at the beginning of the run

void turnOnLED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin); // Turn on a specific LED
void turnOffLED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin); // Turn off a specific LED
void toggleLED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin); // Toggle a specific LED

void readButtonPeriod(void); // Read the 1st button (Uses for getting in and out of period editing mode)
void readButtonIncrease(void); // Read the 2nd button (Uses for increasing the period)
void readButtonDecrease(void); // Read the 3rd button (Uses for decreasing the period)

void adjustPeriod(void); // Let the user to adjust the period

void readData(void); // Read Temperature and Humidity
void printDataTerminal(void); // Print Temperature and Humidity readings on the Terminal
void printDataLCD(void); // Print Temperature and Humidity readings on the LCD

void FSM_Temperature(void); // FSM for Temperature
void runHeater(void); // Runs the heater
void runHeatPump(void); // Runs the heat pump

void Fan1(void); // Runs fan 1
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initialize(void) {
	buttonPeriod = notPressed;
	buttonIncrease = notPressed;
	buttonDecrease = notPressed;
	TempFSMState = initTemp;
	periodInput = TRUE;
	humidityCounter = 0;
	period = 0;
	frequency = 100;
	heatEffect = 0;
	timeoutFlag = FALSE;
	turnOffLED(Heater_GPIO_Port, Heater_Pin);
	turnOffLED(Heat_pump_GPIO_Port, Heat_pump_Pin);
	turnOffLED(Fan_1_GPIO_Port, Fan_1_Pin);
	turnOffLED(Fan_2_GPIO_Port, Fan_2_Pin);
	turnOffLED(Fan_3_GPIO_Port, Fan_3_Pin);
}

void turnOnLED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void turnOffLED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void toggleLED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

void readButtonPeriod(void) {
	firstRead[0] = secondRead[0];
	secondRead[0] = !(HAL_GPIO_ReadPin(Button_Period_GPIO_Port, Button_Period_Pin));
	if (!firstRead[0] && secondRead[0]) {
		buttonPeriod = pressed;
	} else {
		buttonPeriod = notPressed;
	}
}

void readButtonIncrease(void) {
	firstRead[1] = secondRead[1];
	secondRead[1] = !(HAL_GPIO_ReadPin(Button_Increase_GPIO_Port, Button_Increase_Pin));
	if (!firstRead[1] && secondRead[1]) {
		buttonIncrease = pressed;
	} else {
		buttonIncrease = notPressed;
	}
}

void readButtonDecrease(void) {
	firstRead[2] = secondRead[2];
	secondRead[2] = !(HAL_GPIO_ReadPin(Button_Decrease_GPIO_Port, Button_Decrease_Pin));
	if (!firstRead[2] && secondRead[2]) {
		buttonDecrease = pressed;
	} else {
		buttonDecrease = notPressed;
	}
}

void adjustPeriod(void) {
	TempFSMState = initTemp;
	readButtonIncrease();
	readButtonDecrease();
	if (buttonIncrease == pressed) {
		period++;
	} else if (buttonDecrease == pressed) {
		if (period > 0) {
			period--;
		}
	} else if (buttonPeriod == pressed) {
		if (period > 0) {
			periodInput = FALSE;
		}
	}
}

void readData(void) {
	DHT_GetData(&DHT11_Data);
	Temperature = DHT11_Data.Temperature;
	Humidity = DHT11_Data.Humidity;
	humidityCounter = 0;
	Temperature += heatEffect * 0.0001; // Final result would be +0.4/2s for heater and -0.2/2s for heat pump
}

void printDataTerminal(void) {
	if (DHT11_Data.Error != TIMEOUT) {
		uart_buf_len = sprintf(uart_buf, "Temp: %.2f%cC\033[0K\r\nHumidity: %.2f%%\033[0K\r\nFan 1 speed: %.2f%%\033[J\033[H", Temperature , 186, Humidity, 100 - frequency);
		HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	}
}

void printDataLCD(void) {
	if (DHT11_Data.Error != TIMEOUT) {
		if (periodInput == FALSE) {
			changeLCDMode = FALSE;
			sprintf(lcd_buf, "Temp: %.2f%cC", Temperature, 223);
			lcd_goto_XY(1, 0);
			lcd_send_string(lcd_buf);
			sprintf(lcd_buf, "Humidity: %.2f%%", Humidity);
			lcd_goto_XY(2, 0);
			lcd_send_string(lcd_buf);
		} else {
			if (changeLCDMode == FALSE) {
				lcd_clear_display();
				changeLCDMode = TRUE;
			}
			sprintf(lcd_buf, "Period: %u", period);
			lcd_goto_XY(1, 0);
			lcd_send_string(lcd_buf);
		}
	} else {
		//lcd_clear_display();
		sprintf(lcd_buf, "TIME OUT        ");
		lcd_goto_XY(1, 0);
		lcd_send_string(lcd_buf);
		sprintf(lcd_buf, "                ");
		lcd_goto_XY(2, 0);
		lcd_send_string(lcd_buf);
	}

}

void FSM_Temperature(void) {
	switch (TempFSMState) {
	case initTemp:
		heatCounter = 0;
		turnOffLED(Heater_GPIO_Port, Heater_Pin);
		turnOffLED(Heat_pump_GPIO_Port, Heat_pump_Pin);
		turnOffLED(Fan_2_GPIO_Port, Fan_2_Pin);
		turnOffLED(Fan_3_GPIO_Port, Fan_3_Pin);
		TempFSMState = heater;
		break;
	case heater:
		runHeater();
		break;
	case heatPump:
		runHeatPump();
		break;
	case non:
		turnOffLED(Heater_GPIO_Port, Heater_Pin);
		turnOffLED(Heat_pump_GPIO_Port, Heat_pump_Pin);
		turnOffLED(Fan_2_GPIO_Port, Fan_2_Pin);
		turnOffLED(Fan_3_GPIO_Port, Fan_3_Pin);
		break;
	}
	heatCounter++;
	if (heatCounter >=0 && heatCounter < period * SEC_IT && Temperature < TEMPERATURE_THRESHOLD) {
		TempFSMState = heater;
	} else if (heatCounter >= period * SEC_IT && heatCounter <= period * SEC_IT * 2) {
		TempFSMState = heatPump;
		if (heatCounter == period * SEC_IT * 2) {
			heatCounter = 0;
		}
	} else {
		TempFSMState = non;
	}
}

void runHeater(void) {
	turnOnLED(Heater_GPIO_Port, Heater_Pin);
	turnOffLED(Heat_pump_GPIO_Port, Heat_pump_Pin);
	turnOnLED(Fan_2_GPIO_Port, Fan_2_Pin);
	turnOffLED(Fan_3_GPIO_Port, Fan_3_Pin);
	heatEffect += 2;
}

void runHeatPump(void) {
	turnOnLED(Heat_pump_GPIO_Port, Heat_pump_Pin);
	turnOffLED(Heater_GPIO_Port, Heater_Pin);
	turnOnLED(Fan_3_GPIO_Port, Fan_3_Pin);
	turnOffLED(Fan_2_GPIO_Port, Fan_2_Pin);
	heatEffect--;
}

void Fan1(void) {
	frequency = 100 - ((100 * (Humidity - HUMID_THRESHOLD)) / (100 - HUMID_THRESHOLD));
	if (Humidity > HUMID_THRESHOLD) {
		if (humidityCounter == frequency * SEC_IT / 100) {
			toggleLED(Fan_1_GPIO_Port, Fan_1_Pin);
			humidityCounter = 0;
		} else {
			humidityCounter++;
		}
	} else {
		frequency = 100;
		humidityCounter = 0;
		turnOffLED(Fan_1_GPIO_Port, Fan_1_Pin);
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
  initialize();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  lcd_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printDataLCD();
	  if (DHT11_Data.Error == TIMEOUT) {
		  timeoutFlag = TRUE;
		  turnOffLED(Heater_GPIO_Port, Heater_Pin);
		  turnOffLED(Heat_pump_GPIO_Port, Heat_pump_Pin);
		  turnOffLED(Fan_1_GPIO_Port, Fan_1_Pin);
		  turnOffLED(Fan_2_GPIO_Port, Fan_2_Pin);
		  turnOffLED(Fan_3_GPIO_Port, Fan_3_Pin);
	  } else {
		  if (timeoutFlag == TRUE) {
			  timeoutFlag = FALSE;
			  initialize();
			  lcd_init();
			  TempFSMState = initTemp;
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Fan_3_Pin|Fan_2_Pin|Heater_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Heat_pump_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Fan_1_GPIO_Port, Fan_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Fan_3_Pin Fan_2_Pin Heater_Pin */
  GPIO_InitStruct.Pin = Fan_3_Pin|Fan_2_Pin|Heater_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_Decrease_Pin Button_Period_Pin Button_Increase_Pin */
  GPIO_InitStruct.Pin = Button_Decrease_Pin|Button_Period_Pin|Button_Increase_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Heat_pump_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Heat_pump_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Fan_1_Pin */
  GPIO_InitStruct.Pin = Fan_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Fan_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3 && DHT11_Data.Error != TIMEOUT) {
		readButtonPeriod();
		if (periodInput == FALSE) {
			if (buttonPeriod == pressed) {
				periodInput = TRUE;
				lcd_clear_display();
			} else {
				FSM_Temperature();
				Fan1();
			}
		} else {
			adjustPeriod();
		}
	}
	if (htim == &htim2) {
		readData();
		printDataTerminal();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
