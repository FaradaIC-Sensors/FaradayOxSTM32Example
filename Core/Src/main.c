/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "registers.h"
#include "version.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Leave at least 60 sec between sensor measurements
#define SENSOR_MEASUREMENT_DELAY_MS 60000
#define USER_UART_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t rx_buffer[REGISTER_MAP_SIZE + 1]; // To fit in the whole register map + '\n'

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void sensor_get_metadata(void);
static void sensor_measurement(void);
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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("Faraday-Ox Example v%u.%u\r\n", APP_VER_MAJOR, APP_VER_MINOR);
	sensor_get_metadata();
	sensor_measurement();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
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
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 100);
	return ch;
}

int _write(int file, char *ptr, int len)
{
	(void) file;
	for (int i = 0; i < len; i++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}

static void sensor_get_metadata(void)
{
	uint8_t cmd[CMD_SIZE] = { 0 };

	// 1. Read the whole Register Map
	// Command size if 4 bytes.
	// Read 0x60 bytes starting from address 0x00

	cmd[0] = CMD_READ;
	cmd[1] = 0x00;
	cmd[2] = REGISTER_MAP_SIZE;
	cmd[3] = CMD_NEWLINE;

	HAL_UART_Transmit(&huart1, cmd, CMD_SIZE, 100);
	HAL_UART_Receive(&huart1, rx_buffer, REGISTER_MAP_SIZE + 1, 1000);

	// 2. Get Register Map Version
	uint8_t reg_map_ver_minor = rx_buffer[REG_MAP_VER_LSB];
	uint8_t reg_map_ver_major = rx_buffer[REG_MAP_VER_MSB];

	// 3. Get Module ID
	uint32_t module_id = *(uint32_t *)&rx_buffer[REG_FRONTEND_ID_LLSB];

	printf("Module ID: %lu, Register Ver: %u.%u\r\n", module_id,
			reg_map_ver_major, reg_map_ver_minor);

	return;
}

static void sensor_measurement(void)
{
	// 1. Start the measurement
	uint8_t cmd[4] = { 0 };
	cmd[0] = CMD_WRITE;
	cmd[1] = REG_CONTROL;
	cmd[2] = REG_CONTROL_START_MEASUREMENT;
	cmd[3] = CMD_NEWLINE;

	HAL_UART_Transmit(&huart1, cmd, CMD_SIZE, 100); // Initiate measurement
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, rx_buffer,
			CMD_WRITE_ACK_SIZE, 100); // Get ACK
	if (status != HAL_OK)
	{
		return;
	}

	HAL_Delay(500); // Wait 500ms for the measurement to finish

	memset(rx_buffer, 0, REGISTER_MAP_SIZE + 1);
	cmd[0] = CMD_READ;
	cmd[1] = 0x00;
	cmd[2] = REGISTER_MAP_SIZE;
	cmd[3] = CMD_NEWLINE;

	HAL_UART_Transmit(&huart1, cmd, CMD_SIZE, 100);
	HAL_UART_Receive(&huart1, rx_buffer, REGISTER_MAP_SIZE + 1, 1000);

	// 2. Check that Status register indicates that measurement was finished successfully
	if (rx_buffer[REG_STATUS] & REG_STATUS_MEASUREMENT_IN_PROGRESS)
	{
		printf("Measurement is still in progress\r\n");
		return;
	}

	if ((rx_buffer[REG_STATUS] & REG_STATUS_MEASUREMENT_FINISHED)
			&& (rx_buffer[REG_STATUS] & REG_STATUS_MEASUREMENT_ERROR))
	{
		printf("Sensor error\r\n");
		return;
	}

	if ((rx_buffer[REG_STATUS] & REG_STATUS_MEASUREMENT_FINISHED)
			&& (rx_buffer[REG_STATUS] & REG_STATUS_SHT4X_ERROR))
	{
		printf("SHT4x sensor error\r\n");
		return;
	}

	if (rx_buffer[REG_STATUS] == REG_STATUS_MEASUREMENT_FINISHED)
	{
		float concentration = *(float *)&rx_buffer[REG_CONCENTRATION];
		float temperature = *(float *)&rx_buffer[REG_TEMPERATURE];
		float humidity = *(float *)&rx_buffer[REG_HUMIDITY];

		printf("Concentration: %f, Temperature: %f, Humidity: %f\r\n",
				concentration, temperature, humidity);
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
