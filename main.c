/* USER CODE BEGIN Header */
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
#include<string.h>
#include<stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t ICM_ADDR = 0x69 << 1;
static const uint8_t WHO_AM_I = 0x00;
static const uint8_t USER_CTRL = 0x03;
static const uint8_t PWR_MGMT_1 = 0x06;
static const uint8_t CNTL2 = 0x31;
//static const uint8_t ST1 = 0x10;
//static const uint8_t ST2 = 0x18;
static const uint8_t ACCEL_XOUT_H = 0x2D;
static const uint8_t ACCEL_XOUT_L = 0x2E;
static const uint8_t ACCEL_YOUT_H = 0x2F;
static const uint8_t ACCEL_YOUT_L = 0x30;
static const uint8_t ACCEL_ZOUT_H = 0x31;
static const uint8_t ACCEL_ZOUT_L = 0x32;
static const uint8_t GYRO_XOUT_H = 0x33;
static const uint8_t GYRO_XOUT_L = 0x34;
static const uint8_t GYRO_YOUT_H = 0x35;
static const uint8_t GYRO_YOUT_L = 0x36;
static const uint8_t GYRO_ZOUT_H = 0x37;
static const uint8_t GYRO_ZOUT_L = 0x38;
static const uint8_t HXL = 0x11;
static const uint8_t HXH = 0x12;
static const uint8_t HYL = 0x13;
static const uint8_t HYH = 0x14;
static const uint8_t HZL = 0x15;
static const uint8_t HZH = 0x16;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  HAL_StatusTypeDef ret;
  uint8_t buf[120];
  uint8_t accel[] = {ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L};
  uint8_t gyro[] = {GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L};
  uint8_t mag[] = {HXH, HXL, HYH, HYL, HZH, HZL};  //little endian >:(
  int16_t val[3];
  float a_dat[3];
  float g_dat[3];
  float m_dat[3];
  int i;
//  float pitch = 0;
//  float roll = 0;
//  float yaw = 0;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    buf[0] = WHO_AM_I;
  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	if (ret != HAL_OK) {
  		strcpy((char*)buf, "Error Tx\r\n");
  	} else {
  		ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  		if (ret != HAL_OK) {
  			strcpy((char*)buf, "Error Rx\r\n");
  		}
  	}

  	//wait for IMU to say hello
  	while(buf[0] != 0xEA) {
  		//GPIOC_ODR ^= GPIO_ODR_8;
  	}

  	buf[0] = PWR_MGMT_1;
  	buf[1] = 0x05;	//clear sleep bit & disable temp_sens
  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 2, HAL_MAX_DELAY);
  	if (ret != HAL_OK) {
  		strcpy((char*)buf, "Error Tx\r\n");
  	}

  	buf[0] = USER_CTRL;
  	buf[1] = 0xA0;  //enable DMP and FIFO
  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 2, HAL_MAX_DELAY);
  	if (ret != HAL_OK) {
  		strcpy((char*)buf, "Error Tx\r\n");
  	}

  	buf[0] = CNTL2;
  	buf[1] = 0x02;  //set continuous measurement mode 1 for mag
  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 2, HAL_MAX_DELAY);
  	if (ret != HAL_OK) {
  		strcpy((char*)buf, "Error Tx\r\n");
  	}


  	while (1)
    {
  	  for(i = 0; i < 6; i+=2) {
  		  buf[0] = accel[i];
  		  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  		  	if (ret != HAL_OK) {
  		  		strcpy((char*)buf, "Error Tx\r\n");
  		  	} else {
  		  		ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  		  		if (ret != HAL_OK) {
  		  			strcpy((char*)buf, "Error Rx\r\n");
  		  		}
  		  	}

  		  	val[i/2] = (int16_t)(buf[0]<<8);

  		  	buf[0] = accel[i+1];
  		  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  		  	if (ret != HAL_OK) {
  		  		strcpy((char*)buf, "Error Tx\r\n");
  		  	} else {
  		  		ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  		  		if (ret != HAL_OK) {
  		  			strcpy((char*)buf, "Error Rx\r\n");
  		  		}
  		  	}

  		  	val[i/2] |= (int16_t)buf[0];

  		  	a_dat[i/2] = (val[i/2] / 32768.0) * 19.62;
  	  }

  	  for(i = 0; i < 6; i+=2) {
  	  		  buf[0] = gyro[i];
  	  		  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	  		  	if (ret != HAL_OK) {
  	  		  		strcpy((char*)buf, "Error Tx\r\n");
  	  		  	} else {
  	  		  		ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	  		  		if (ret != HAL_OK) {
  	  		  			strcpy((char*)buf, "Error Rx\r\n");
  	  		  		}
  	  		  	}

  	  		  	val[i/2] = (int16_t)(buf[0]<<8);

  	  		  	buf[0] = gyro[i+1];
  	  		  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	  		  	if (ret != HAL_OK) {
  	  		  		strcpy((char*)buf, "Error Tx\r\n");
  	  		  	} else {
  	  		  		ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	  		  		if (ret != HAL_OK) {
  	  		  			strcpy((char*)buf, "Error Rx\r\n");
  	  		  		}
  	  		  	}

  	  		  	val[i/2] |= (int16_t)buf[0];

  	  		  	g_dat[i/2] = (val[i/2] / 32768.0) * 250;
  	  }


  	  //wait for data ready
  	//  while(buf[0] != 0x01) {
  	//	  buf[0] = ST1;
  	//	  ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	//	  if (ret != HAL_OK) {
  	//	  	strcpy((char*)buf, "Error Tx\r\n");
  	////		  } else {
  	////		  	ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	////		  	if (ret != HAL_OK) {
  	////				strcpy((char*)buf, "Error Rx\r\n");
  	////		  	}
  	////		  }
  	////	  }
  	//
  	////	  for(i = 0; i < 6; i+=2) {
  	////	  		  buf[0] = mag[i];
  	////	  		  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	////	  		  	if (ret != HAL_OK) {
  	////	  		  		strcpy((char*)buf, "Error Tx\r\n");
  	////	  		  	} else {
  	////	  		  		ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	////	  		  		if (ret != HAL_OK) {
  	////	  		  			strcpy((char*)buf, "Error Rx\r\n");
  	////	  		  		}
  	////	  		  	}
  	////
  	////	  		  	val[i/2] = (int16_t)(buf[0]<<8);
  	////
  	////	  		  	buf[0] = mag[i+1];
  	////	  		  	ret = HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	////	  		  	if (ret != HAL_OK) {
  	////	  		  		strcpy((char*)buf, "Error Tx\r\n");
  	////	  		  	} else {
  	////	  		  		ret = HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR, buf, 1, HAL_MAX_DELAY);
  	////	  		  		if (ret != HAL_OK) {
  	////	  		  			strcpy((char*)buf, "Error Rx\r\n");
  	////	  		  		}
  	////	  		  	}
  	////
  	////	  		  	val[i/2] |= (int16_t)buf[0];
  	////
  	////	  		  	m_dat[i/2] = (val[i/2] / 32768.0);
  	////	  }
  	//
  	////	  pitch = 180 * atan(a_dat[0] / sqrt(a_dat[1] * a_dat[1] + a_dat[2] * a_dat[2])) / M_PI;
  	////	  roll = 180 * atan(a_dat[1] / sqrt(a_dat[0] * a_dat[0] + a_dat[2] * a_dat[2])) / M_PI;
  	////
  	////	  yaw = 180 * atan(a_dat[2] / sqrt(a_dat[0] * a_dat[0] + a_dat[2] * a_dat[2])) / M_PI;

  	sprintf((char*)buf, "accel x: %1.2f m/s, accel y: %1.2f m/s, accel z: %1.2f m/s, gyro x: %1.2f dps, gyro y: %1.2f dps, gyro z: %1.2f dps\r\n", a_dat[0], a_dat[1], a_dat[2], g_dat[0], g_dat[1], g_dat[2]);

  	//HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
  	//HAL_Delay(500);
  	//sprintf((char*)buf, "accel_x: %1.2f m/s, accel_y: %1.2f m/s, accel_z: %1.2f m/s, gyro_x: %1.2f deg, gyro_y: %1.2f deg, gyro_z: %1.2f deg", a_dat[0], a_dat[1], a_dat[2], g_dat[0]);
  	HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
  	HAL_Delay(500);

  	    /* USER CODE BEGIN 3 */
  	}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00100409;
  hi2c1.Init.OwnAddress1 = 210;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  huart2.Init.BaudRate = 115200;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
