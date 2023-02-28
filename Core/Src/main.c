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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
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

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Rx_data[1];

uint32_t read = 0;
uint32_t command = 0;

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */
void read_from_address(uint32_t);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
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

		if (command == 1) {
			read_from_address(read * 4);
		} else if (command == 2) {
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			char current_time_message_flash[100];
			sprintf(current_time_message_flash,
					"%s\tDate: %02d.%02d.%02d\tTime: %02d.%02d.%02d\r\n",
					"log event", sDate.Date, sDate.Month, sDate.Year,
					sTime.Hours, sTime.Minutes, sTime.Seconds);
			if (f_mount(&SDFatFS, SDPath, 0) == FR_OK) {
				if (f_open(&SDFile, "readme.txt", FA_CREATE_ALWAYS | FA_WRITE)
						== FR_OK) {
					uint32_t read_bytes = 0;
					if (f_write(&SDFile, current_time_message_flash,
							strlen(current_time_message_flash), &read_bytes)
							== FR_OK) {
						f_close(&SDFile);
					}
				}
			}
			command = 9;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 255;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : F_CS_Pin */
  GPIO_InitStruct.Pin = F_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(F_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
long get_epoch_time(){
    RTC_DateTypeDef rtcDate;
    RTC_TimeTypeDef rtcTime;
    HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
    uint8_t hh = rtcTime.Hours;
    uint8_t mm = rtcTime.Minutes;
    uint8_t ss = rtcTime.Seconds;
    uint8_t d = rtcDate.Date;
    uint8_t m = rtcDate.Month;
    uint16_t y = rtcDate.Year;
    uint16_t yr = (uint16_t)(y+2000-1900);
    time_t currentTime = {0};
    struct tm tim = {0};
    tim.tm_year = yr;
    tim.tm_mon = m - 1;
    tim.tm_mday = d;
    tim.tm_hour = hh;
    tim.tm_min = mm;
    tim.tm_sec = ss;
    currentTime = mktime(&tim);
    return currentTime;
}

void flash_erase_chip() {
	uint8_t Write_Enable = 0x06;
	uint8_t Erase_Chip = 0xC7;

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, RESET);     // CS to low
	HAL_SPI_Transmit(&hspi1, &Write_Enable, 1, 1000); // Write Enable Command
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, SET);       // CS to high

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, RESET);     // CS to low
	HAL_SPI_Transmit(&hspi1, &Erase_Chip, 1, 1000);   // Erase Chip Command
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, SET);       // CS to high

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0);
}

char temp_time [20];
uint8_t send_address[3];
#define buf_size 100
long temp_date[buf_size];
uint32_t items;



uint32_t to_read;

void flash_read_data() {
	read = 0;
	to_read = 0;
	command = 1 ;
	uint32_t address = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	items = address / 4;
	sprintf(temp_time, "logged events: %d\r\n", items);
	HAL_UART_Transmit(&huart1, (uint8_t*) temp_time, strlen(temp_time), 10);
}

void read_from_address(uint32_t start_address) {
	if (items - read > buf_size) {
		to_read = buf_size;
	} else {
		to_read = items - read;
	}
	if (to_read > 0) {
		uint8_t Read_Data = 0x03;
		HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, RESET);
		HAL_SPI_Transmit(&hspi1, &Read_Data, 1, 1000);  // Read Command

		send_address[2] = start_address;
		send_address[1] = start_address >> 8;
		send_address[0] = start_address >> 16;

		HAL_SPI_Transmit(&hspi1, send_address, 3, 1000);    // Write Address
		if (HAL_SPI_Receive_DMA(&hspi1, temp_date, to_read * 4) != HAL_OK) {
			HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, SET);
			Error_Handler();
			command = 0;
		}
	} else {
		command = 0;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, SET);
	for (int i = 0; i < to_read; ++i) {
		sprintf(temp_time, "%d\r\n", temp_date[i]);
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_time, strlen(temp_time), 10);
	}
	read += to_read;
}

void flash_write_data() {
	uint8_t Write_Enable = 0x06;
	uint8_t Page_Program = 0x02;
	uint32_t address = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &Write_Enable, 1, 1000); // Write Enable Command
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, SET);

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &Page_Program, 1, 1000); // Page Program Command


	send_address[2] = address;
	send_address[1] = address >> 8;
	send_address[0] = address >> 16;

	HAL_SPI_Transmit(&hspi1, &send_address, 3, 1000);
	long temp_time = get_epoch_time();
	HAL_SPI_Transmit(&hspi1, &temp_time, 4, 1000);
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, SET);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, address + 4);
}

int state = 1;


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
		"Options:\r\n1. Show current time\r\n2. Change date/time\r\n3. Log event\r\n4. Read all\r\n5. Write to SD card\r\n6. Erase all from Flash\r\n";

void display_main_menu() {
	HAL_UART_Transmit(&huart1, (uint8_t*) menu, strlen(menu), 10);
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
					"%s\tDate: %02d.%02d.%02d\tTime: %02d.%02d.%02d (%d)\r\n",
					"button pressed (event logged)", sDate.Date, sDate.Month, sDate.Year,
					sTime.Hours, sTime.Minutes, sTime.Seconds, get_epoch_time());

			flash_write_data();

			HAL_UART_Transmit(&huart1, (uint8_t*) message, strlen(message), 10); // Sending in normal mode
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
		HAL_UART_Transmit(&huart1, deactivate_message, strlen(deactivate_message), 10);
	}
}

int change_time_menu_option = 0;
char change_time_menu[] = "Change:\r\n1. Day\r\n2. Month\r\n3. Year\r\n4. Hour\r\n5. Minute\r\n6. Seconds\r\n7. Exit\r\n";
char unkown_option[] = "unknown option\r\n";
char enter_new_value [] = "Please enter new value: ";
char current_time_message[100];
char current_time_message_flash[100];
void display_current_time() {
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(current_time_message,
			"%s\tDate: %02d.%02d.%02d\tTime: %02d.%02d.%02d\r\n",
			"current date/time", sDate.Date, sDate.Month, sDate.Year,
			sTime.Hours, sTime.Minutes, sTime.Seconds);
	HAL_UART_Transmit(&huart1, (uint8_t*) current_time_message,
			strlen(current_time_message), 10);
}

void display_change_time_menu() {
	HAL_UART_Transmit(&huart1, (uint8_t*) change_time_menu,
			strlen(change_time_menu), 10);
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

unsigned char towrite [] = "checkme";
unsigned char toread [100];
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
					strlen(enter_new_value), 10);
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
		case 3:
			flash_write_data();

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			sprintf(current_time_message_flash,
					"%s\tDate: %02d.%02d.%02d\tTime: %02d.%02d.%02d\r\n",
					"log event", sDate.Date, sDate.Month, sDate.Year, sTime.Hours,
					sTime.Minutes, sTime.Seconds);

			HAL_UART_Transmit(&huart1, (uint8_t*) current_time_message_flash, strlen(current_time_message_flash), 10);
			display_main_menu();
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			return;
		case 4:
			flash_read_data();
			display_main_menu();
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			return;
		case 5:
			command = 2;
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			return;
		case 6:
			flash_erase_chip();
			display_main_menu();
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			return;
		default:
			HAL_UART_Transmit(&huart1, (uint8_t*) unkown_option,
					strlen(unkown_option), 10);
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
		HAL_UART_Transmit(&huart1, (uint8_t*) Rx_data, strlen(Rx_data), 10);
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
