/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Rx_data[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	int step = 10;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);
	while (1) {
		step += 10;
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		HAL_Delay(100 + step % 100);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		HAL_Delay(100 + step % 100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV64;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == 0xcafe) {
		return;
	} else {
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xcafe);
	}
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x16;
  sTime.Minutes = 0x49;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x19;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MENU_Pin */
  GPIO_InitStruct.Pin = MENU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MENU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

int state = 1;

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};

int menu_state = 0;

#define LINEMAX 200 // Maximal allowed/expected line length
volatile char line_buffer[LINEMAX + 1]; // Holding buffer with space for terminating NUL
volatile int line_valid = 0;
static char rx_buffer[LINEMAX];   // Local holding buffer to build line
static int rx_index = 0;

void invalidate_buffer() {
	line_valid = 0;
	rx_index = 0;
}
char menu[] =
		"Options:\r\n1. Show current time\r\n2. Change date/time\r\n3. Exit\r\n";

void display_main_menu() {
	HAL_UART_Transmit(&huart1, (uint8_t*) menu, sizeof(menu), 10);
	menu_state = 0;
	invalidate_buffer();
}

char message[100];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BUTTON_Pin) {
		if (state) {
			state = 0;
			HAL_TIM_Base_Start_IT(&htim1);

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			sprintf(message,
					"%s\tDate: %02d.%02d.%02d\tTime: %02d.%02d.%02d\r\n",
					"button pressed", sDate.Date, sDate.Month, sDate.Year,
					sTime.Hours, sTime.Minutes, sTime.Seconds);

			HAL_UART_Transmit(&huart1, (uint8_t*) message, sizeof(message), 10); // Sending in normal mode
		}
	} else if (GPIO_Pin == MENU_Pin) {
		display_main_menu();
	}
}

uint8_t deactivate_message[] = "button released\r\n";

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET) {
		state = 1;
		HAL_TIM_Base_Stop_IT(&htim1);
		HAL_UART_Transmit(&huart1, deactivate_message, sizeof(deactivate_message), 10);
	}
}

int change_time_menu_option = 0;
char change_time_menu[] = "Change:\r\n1. Day\r\n2. Month\r\n3. Year\r\n4. Hour\r\n5. Minute\r\n6. Seconds\r\n7. Exit\r\n";
char unkown_option[] = "unknown option\r\n";
char enter_new_value [] = "Please enter new value: ";
char current_time_message[100];

void display_current_time() {
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(current_time_message,
			"%s\tDate: %02d.%02d.%02d\tTime: %02d.%02d.%02d\r\n",
			"current date/time", sDate.Date, sDate.Month, sDate.Year,
			sTime.Hours, sTime.Minutes, sTime.Seconds);
	HAL_UART_Transmit(&huart1, (uint8_t*) current_time_message,
			sizeof(current_time_message), 10);
}

void display_change_time_menu() {
	HAL_UART_Transmit(&huart1, (uint8_t*) change_time_menu,
			sizeof(change_time_menu), 10);
	menu_state = 1;
}

void update_date_time(int selected_option, int value) {
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	switch (selected_option) {
	case 1:
		sDate.Date = value;
		break;
	case 2:
		sDate.Month = value;
		break;
	case 3:
		sDate.Year = value;
		break;
	case 4:
		sTime.Hours = value;
		break;
	case 5:
		sTime.Minutes = value;
		break;
	case 6:
		sTime.Seconds = value;
		break;
	}
	if (selected_option == 1 || selected_option == 2 || selected_option == 3) {
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
	} else if (selected_option == 4 || selected_option == 5
			|| selected_option == 6) {
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
	}
}

void handle_received_char(char rx) {
	if ((rx == '\r') || (rx == '\n')) {
		if (rx_index != 0) {
			memcpy((void*) line_buffer, rx_buffer, rx_index); // Copy to static line buffer from dynamic receive buffer
			line_buffer[rx_index] = 0; // Add terminating NUL
			line_valid = 1; // flag new line valid for processing
			rx_index = 0; // Reset content pointer
		}
	} else {
		if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
			rx_index = 0;
		rx_buffer[rx_index++] = rx; // Copy to buffer and increment
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	int selected_option = atoi((const char*) Rx_data);
	if (menu_state == 1) {
		switch (selected_option) {
		case 7:
			display_main_menu();
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			break;
		default:
			change_time_menu_option = selected_option;
			HAL_UART_Transmit(&huart1, (uint8_t*) enter_new_value,
					sizeof(enter_new_value), 10);
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			menu_state = 2;
		}
		return;
	} else if (menu_state == 0) {
		switch (selected_option) {
		case 1:
			display_current_time();
			display_main_menu();
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			return;
		case 2:
			display_change_time_menu();
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			return;
		default:
			HAL_UART_Transmit(&huart1, (uint8_t*) unkown_option,
					sizeof(unkown_option), 10);
			display_main_menu();
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			return;
		}
	}
	handle_received_char((char) Rx_data[0]);
	if (line_valid) {
		HAL_UART_Transmit(&huart1, (uint8_t*) &"\r\n", 2, 10);
		selected_option = (int) atoi((const char*) line_buffer);
		if (menu_state == 2) {
			update_date_time(change_time_menu_option, selected_option);
			display_current_time();
			display_change_time_menu();
		}
		invalidate_buffer();
	} else {
		HAL_UART_Transmit(&huart1, (uint8_t*) Rx_data, sizeof(Rx_data), 10);
	}
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);
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
	while (1) {
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
