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
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "bno055.h"
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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
float get_distance_us015();

void BNO055_Init();
void BNO055_GetAccel(int16_t *x, int16_t *y, int16_t *z);

uint8_t GetAccelData(uint8_t* str);
uint8_t GetMagnetData(uint8_t* str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TRIG_PIN GPIO_PIN_0 // Trigピン (適切に変更)
#define ECHO_PIN GPIO_PIN_1 // Echoピン (適切に変更)
#define TRIG_PORT GPIOC     // Trigピンのポート
#define ECHO_PORT GPIOC     // Echoピンのポート

#define	IMU_NUMBER_OF_BYTES	18
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint8_t imu_readings[IMU_NUMBER_OF_BYTES];
  int16_t accel_data[3];
  float acc_x, acc_y, acc_z;

  uint8_t mag_readings[6];
  int16_t magnet_data[3];
  float mag_x, mag_y, mag_z;

  int flag = 3;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1); // タイマーをスタート
  BNO055_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(flag == 0){
		  printf("Hello World!!");
	  }

	  if(flag == 1){
		  float distance = get_distance_us015(); // US-015の距離取得
		  printf("Distance: %.2f", distance);
	  }

	  if(flag == 2){
		  GetAccelData((uint8_t*)imu_readings);
		  accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);
		  accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
		  accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
		  acc_x = ((float)(accel_data[0]))/100.0f; //m/s2
		  acc_y = ((float)(accel_data[1]))/100.0f;
		  acc_z = ((float)(accel_data[2]))/100.0f;

		  //BNO055_GetAccel(&ax, &ay, &az);
		  printf("X: %.2f Y: %.2f Z: %.2f", acc_x, acc_y, acc_z);
	  }

	  if(flag == 3){
		  GetMagnetData((uint8_t*)mag_readings);
		  magnet_data[0] = (((int16_t)((uint8_t *)(mag_readings))[1] << 8) | ((uint8_t *)(mag_readings))[0]);
		  magnet_data[1] = (((int16_t)((uint8_t *)(mag_readings))[3] << 8) | ((uint8_t *)(mag_readings))[2]);
		  magnet_data[2] = (((int16_t)((uint8_t *)(mag_readings))[5] << 8) | ((uint8_t *)(mag_readings))[4]);

		  mag_x = ((float)(magnet_data[0])) / 16.0f;  // μT (BNO055の地磁気データは16LSB/μT)
		  mag_y = ((float)(magnet_data[1])) / 16.0f;
		  mag_z = ((float)(magnet_data[2])) / 16.0f;

		  printf("Magnet X: %.2f μT Y: %.2f μT Z: %.2f μT\n", mag_x, mag_y, mag_z);
	  }
	  printf("\r\n");
	  HAL_Delay(300);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// printf
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
}


// 超音波センサ
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0); // タイマーをリセット
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

float get_distance_us015() {
    // Trigピンを10µs HIGHにする
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // EchoピンがHIGHになるまで待機
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET);

    // HIGHの間の時間を計測
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim1);
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET);
    uint32_t end = __HAL_TIM_GET_COUNTER(&htim1);

    // 距離を計算 (時間 × 音速 / 2)
    float distance = (end - start) * 0.0343 / 2.0;
    return distance;
}

//9軸センサ
void BNO055_Init() {
    uint8_t op_mode = 0x00; // CONFIGMODE
    HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR_LO<<1, 0x3D, 1, &op_mode, 1, 10);
    HAL_Delay(10);

    // オペレーションモードをNDOFモードに設定
    op_mode = 0x0C;
    HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR_LO<<1, 0x3D, 1, &op_mode, 1, 10);
    HAL_Delay(10);
}

void BNO055_GetAccel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6] = {0};
    HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO<<1, 0x08, 1, data, 6, HAL_MAX_DELAY);
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *x = (int16_t)((data[5] << 8) | data[4]);
}

uint8_t GetAccelData(uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO<<1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return status;
}

uint8_t GetMagnetData(uint8_t* str) {
    uint8_t status;
    status = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO << 1, BNO055_MAG_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, 6, 100);
    return status;
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
