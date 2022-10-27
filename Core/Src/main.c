/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "drv_adxl345.h"
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

/* USER CODE BEGIN PV */
h_adxl345_t h_adxl345;
//on crée uns instance de la strcuture h_adxl345_t

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_acceleration(uint32_t period, float acc_x, float acc_y, float acc_z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
	return chr;
}

//interruption dans stm32 it on récupère
//la fonction de Callback qui nous permet de mette ce qui doit être fait dans l'interruption
//dans le main !
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==INT_Pin){
		adxl345_irq_cb(&h_adxl345);
	}
}

int spi_transmit(uint8_t address, uint8_t *p_data, uint16_t size){
	// Reading the DEVID, it should be 0xE5
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, &address, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi2, p_data, size, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

int spi_receive(uint8_t address, uint8_t *p_data, uint16_t size){
	// Reading the DEVID, it should be 0xE5
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, &address, 1, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi2, p_data, size, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
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
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("\r\n===== ADXL345 =====\r\n");
	printf("Connecter VCC sur le 5V et GND sur GND\r\n");
	printf("CS sur D3, SDO sur D12, SDA sur D11, SCL sur D13, INT1 sur D4\r\n");

	//on met des valeurs dans la structure
	h_adxl345.adxl345_rate=RATE_100_HZ;
	h_adxl345.adxl345_range=RANGE_2G;
	h_adxl345.adxl345_serial_drv.receive=spi_receive;
	//fonction sans les parenthèses = pointeur de fonction et nous donne l'adresse de la première instruction
	h_adxl345.adxl345_serial_drv.transmit=spi_transmit;

//	Que met-on dans la structure
	//	Fréquence de mesure (data rate)
	//	Range de mesure
	//	driver SPI


	//	x, y et z
	//	données disponible (data available)
	//	Skipped data
	//	DEV ID (11100101 valeur dans la datasheet)

//	le _t signifie pour ST toujours une structure

	//on appelle init en mettant en paramètre un pointeur de la structure
	adxl345_init(&h_adxl345);
	adxl345_start(&h_adxl345);

	adxl345_read_acceleration(&h_adxl345);

	while(1){
		//permet de créer une structure nous même
		if(adxl1345_data_available(&h_adxl345)){
			adxl1345_read_acceleration(&h_adxl345);

			float x= adxl345_get_x_acceleration(&h_adxl345);
			float y= adxl345_get_y_acceleration(&h_adxl345);
			float z= adxl345_get_z_acceleration(&h_adxl345);
			process_acceleration(100,x, y, z);
		}

	}

#if 0
	uint8_t receive_buffer = 0;
	uint8_t transmit_buffer = 0x80;

	// Reading the DEVID, it should be 0xE5
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, &receive_buffer, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

	printf("DEVID = 0x%02X\r\n", receive_buffer);
	if (receive_buffer != 0xE5)
	{
		Error_Handler();
	}

	// Write '1' to the DATA_READY bit in the INT_ENABLE register
	// Generate an interrupt when a new data is ready
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	transmit_buffer = 0x2E;
	HAL_SPI_Transmit(&hspi2, &transmit_buffer, 1, HAL_MAX_DELAY);
	transmit_buffer = 0x80;
	HAL_SPI_Transmit(&hspi2, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

	// Write '1' to the Measure bit in POWER_CTL register
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	transmit_buffer = 0x2D;
	HAL_SPI_Transmit(&hspi2, &transmit_buffer, 1, HAL_MAX_DELAY);
	transmit_buffer = 0x08;
	HAL_SPI_Transmit(&hspi2, &transmit_buffer, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

	uint8_t buffer[6];
	int16_t x_int;
	int16_t y_int;
	int16_t z_int;

	float x, y, z;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		// Wait the ADXL345 to finish its measure
		while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_RESET);

		// Reads registers DATAX0 to DATAZ1
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
		transmit_buffer = 0x80 + 0x40 + 0x32;
		HAL_SPI_Transmit(&hspi2, &transmit_buffer, 1, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi2, buffer, 6, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

		x_int = buffer[0] + (buffer[1] << 8);
		y_int = buffer[2] + (buffer[3] << 8);
		z_int = buffer[4] + (buffer[5] << 8);

		x = (float) x_int / 256.0;
		y = (float) y_int / 256.0;
		z = (float) z_int / 256.0;

		process_acceleration(100, (float)x, (float)y, (float)z);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
#endif
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
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void process_acceleration(uint32_t period, float acc_x, float acc_y, float acc_z)
{
	static uint32_t tick_last = 0;
	static uint32_t counter = 0;
	static float acc_x_average = 0.0f;
	static float acc_y_average = 0.0f;
	static float acc_z_average = 0.0f;
	uint32_t tick = HAL_GetTick();

	acc_x_average += acc_x;
	acc_y_average += acc_y;
	acc_z_average += acc_z;
	counter++;

	if ((tick - tick_last) >= period)
	{
		acc_x_average /= counter;
		acc_y_average /= counter;
		acc_z_average /= counter;

		printf("x=%1.2f, y=%1.2f, z=%1.2f\r", acc_x_average, acc_y_average, acc_z_average);
		fflush(stdout);

		acc_x_average = 0.0f;
		acc_y_average = 0.0f;
		acc_z_average = 0.0f;
		counter = 0;

		tick_last = tick;
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
