/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "DHT.h"
#include "string.h"
#include "stdio.h"
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId UARTToNodeHandle;
osThreadId getDataTaskHandle;
/* USER CODE BEGIN PV */

#define ALTITUDE 3.6; //height from sea level in meter
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
void SendDataToNodeThread(void const *argument);
void GetDataFromSensor(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char uartData[256];

char pmbuffer[150];

float altitude = ALTITUDE
;
// height from sea level in meter

BMP280_HandleTypedef bmp280;

bool bme280p;
float BMP_pressure, BMP_temperature, BMP_humidity;

/* DEFINE THE DHT DataTypedef
 * Look in the DHT.h for the definition
 */
DHT_DataTypedef DHT22_Data;
float DHT_temperature, DHT_humidity;

float cur_pressure, cur_temperature, cur_humidity = 0;
float prev_pressure, prev_temperature, prev_humidity = 0;
uint8_t z; // Prediction value

void ReadDataFromSensors() {
	// Set LED for Debugging
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	// GET DATA FROM DHT22
	sprintf(uartData, "\r\nDHT22 Sensor:\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
	DHT_GetData(&DHT22_Data);
	DHT_temperature = DHT22_Data.Temperature / 10;
	DHT_humidity = DHT22_Data.Humidity / 10;
	sprintf(uartData, "Temp (C) =\t %.1f\r\nHumidity (%%) =\t %.1f%%\r\n",
			DHT_temperature, DHT_humidity);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);

	HAL_Delay(1000); // WAIT

	// GET DATA FROM BMP-280
	sprintf(uartData, "BMP-280 Sensor:\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
	while (!bmp280_read_float(&bmp280, &BMP_temperature, &BMP_pressure,
			&BMP_humidity)) {
		sprintf(uartData, "Temperature/pressure reading failed\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
		HAL_Delay(2000);
	}

	sprintf(uartData, "Pressure: %.2f Pa, Temperature: %.2f C", BMP_pressure,
			BMP_temperature);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
	if (bme280p) {
		sprintf(uartData, ", Humidity: %.2f\r\n", BMP_humidity);
		HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
	}

	else {
		sprintf((char*) uartData, "\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
	}

	// Set LED for Debugging
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void ProcessData() {
	// Choose Data & Calibrate

	// Choose DHT22 over BMP-280 for temperature because it is more accurate
	if (DHT_temperature != 0) {
		cur_temperature = DHT_temperature;
	} else if (BMP_temperature != 0) {
		cur_temperature = BMP_temperature;
	} else {
		cur_temperature = prev_temperature;
	}

	// Humidity is DHT22-exclusive
	if (DHT_humidity != 0) {
		cur_humidity = DHT_humidity - 10; // Calibration
	} else {
		cur_humidity = prev_humidity;
	}

	// Pressure is BMP-280-exclusive
	if (BMP_pressure != 0) {
		cur_pressure = BMP_pressure;
	} else {
		cur_pressure = BMP_pressure;
	}

	// Process Prediction Algorithm

	// Calculate Pressure at Sea Level
	double temp1 = 1.0
			- (0.0065 * altitude)
					/ (cur_temperature + 0.0065 * altitude + 273.15);
	double temp2 = pow(temp1, -5.257);
	double p0 = cur_pressure / 100 * temp2;

	// Pressure is Rising
	if (cur_pressure > prev_pressure + 1.6) {
		z = 185 - 0.16 * p0;
	}
	// Pressure is Falling
	else if (cur_pressure < prev_pressure - 1.6) {
		z = 127 - 0.12 * p0;
	}
	// Pressure is Steady
	else {
		z = 144 - 0.13 * p0;
	}

	// TODO Adjust Z value

	sprintf(uartData, "\r\nDEBUG: %f %f %f\r\n", temp1, temp2, p0);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);

	sprintf(uartData, "\r\nForecast Number = %d\r\n", z);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);

	// Set Data History
	if (cur_pressure != 0) {
		prev_pressure = cur_pressure;
	}
	if (cur_temperature != 0) {
		prev_temperature = cur_temperature;
	}
	if (cur_humidity != 0) {
		prev_humidity = cur_humidity;
	}
}

void SendDataToNodeMCU() {

//	sprintf(uartData, "\r\n DEBUG: %f %f %f\r\n", cur_pressure, cur_temperature, cur_humidity);
//	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);

	// TODO after known the protocol
	// Send Code through UART
	// string code = ... TempC ... Humidity
	//HAL_UART_Transmit(&huart1, &code, 1, 1000);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;
	while (!bmp280_init(&bmp280, &bmp280.params)) {
		sprintf(uartData, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
		HAL_Delay(2000);
	}
	bme280p = bmp280.id == BME280_CHIP_ID;
	sprintf(uartData, "\r\nBMP280: found %s\r\n",
			bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
	HAL_Delay(1000); // WAIT
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of UARTToNode */
	osThreadDef(UARTToNode, SendDataToNodeThread, osPriorityNormal, 0, 128);
	UARTToNodeHandle = osThreadCreate(osThread(UARTToNode), NULL);

	/* definition and creation of getDataTask */
	osThreadDef(getDataTask, GetDataFromSensor, osPriorityIdle, 0, 128);
	getDataTaskHandle = osThreadCreate(osThread(getDataTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//		SendDataToNodeMCU();
//		sprintf(uartData, "while\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
//		HAL_Delay(2000);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 100 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA8 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_SendDataToNodeThread */
/**
 * @brief  Function implementing the UARTToNode thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_SendDataToNodeThread */
void SendDataToNodeThread(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
//		sprintf(uartData, "nok if\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
//
////		if (cur_pressure != 0 && cur_temperature != 0 && cur_humidity != 0) {
//
//		sprintf(uartData, "nai if\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*) uartData, strlen(uartData), 1000);
		/* Call read_PM25_sensor every 15 seconds */
		/* Then send all datas to ESP8266 is the format "s{dustval},{latval},{latdir},{lonval},{londir}"  (s to indicate the start of the data)*/

		// Decompose dustval to string (sprintf won't work with float)
		int pressure_before_decimal = (int) cur_pressure;
		int pressure_after_decimal = (int) (100
				* (cur_pressure - pressure_before_decimal));

		int temperature_before_decimal = (int) cur_temperature;
		int temperature_after_decimal = (int) (100
				* (cur_temperature - temperature_before_decimal));

		int humidity_before_decimal = (int) cur_humidity;
		int humidity_after_decimal = (int) (100
				* (cur_humidity - humidity_before_decimal));

		sprintf(pmbuffer, "s%d.%d,%d.%d,%d.%d,%d\n", pressure_before_decimal,
				pressure_after_decimal, temperature_before_decimal,
				temperature_after_decimal, humidity_before_decimal,
				humidity_after_decimal, z);
		//dummy
//		sprintf(pmbuffer, "s1.2,3.4,5.67,20\n");
		// Transmit the message to ESP8266 in the correct format
		HAL_UART_Transmit(&huart1, (uint8_t*) pmbuffer, strlen(pmbuffer),
		HAL_MAX_DELAY);

		// uncomment to debug (print the sent message to console (baudrate=115200))
		HAL_UART_Transmit(&huart2, (uint8_t*) pmbuffer, strlen(pmbuffer),
		HAL_MAX_DELAY);

//		}
		osDelay(20000);
	}

}
/* USER CODE END 5 */


/* USER CODE BEGIN Header_GetDataFromSensor */
/**
 * @brief Function implementing the getDataTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GetDataFromSensor */
void GetDataFromSensor(void const *argument) {
/* USER CODE BEGIN GetDataFromSensor */
/* Infinite loop */
for (;;) {
	ReadDataFromSensors();
	ProcessData();
	osDelay(4000);
}
/* USER CODE END GetDataFromSensor */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
