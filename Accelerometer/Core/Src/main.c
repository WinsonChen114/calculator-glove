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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define ACC_W_M 0x32
	#define ACC_R_M 0x33
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t total = 0;
  int16_t Zvalarr[] = {16000, 16000, 16000, 16000};
  uint32_t arraycount = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //CTRL REG 1
	  	  uint8_t ctrl_buf_1[2] = {0x20, 0b10010111};//CTRL REG 1 addr and data
	  	  ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, ctrl_buf_1 , 2, 1000);
	  	  if ( ret != HAL_OK ) {
	  			strcpy((char*)ctrl_buf_1, "Error Tx\r\n");
	  		 }

	  	  //Write default values to 2 - 6 to be safe
	  	  uint8_t ctrl_buf_2[2] = {0x21, 0b00000000};//CTRL REG 2 addr and data
	  	  ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, ctrl_buf_2, 2, 1000);
	  	  if ( ret != HAL_OK ) {
	  			strcpy((char*)ctrl_buf_2, "Error Tx\r\n");
	  		 }

	  	  uint8_t ctrl_buf_3[2] = {0x22, 0b00000000};//CTRL REG 3 addr and data
	  	  ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, ctrl_buf_3, 2, 1000);
	  	  if ( ret != HAL_OK ) {
	  			strcpy((char*)ctrl_buf_3, "Error Tx\r\n");
	  		 }

	  	  uint8_t ctrl_buf_4[2] = {0x23, 0b00000000};//CTRL REG 4 addr and data
	  	  ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, ctrl_buf_4, 2, 1000);
	  	  if ( ret != HAL_OK ) {
	  			strcpy((char*)ctrl_buf_4, "Error Tx\r\n");
	  		 }

	  	  uint8_t ctrl_buf_5[2] = {0x24, 0b00000000};//CTRL REG 5 addr and data
	  	  ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, ctrl_buf_5, 2, 1000);
	  	  if ( ret != HAL_OK ) {
	  			strcpy((char*)ctrl_buf_5, "Error Tx\r\n");
	  		 }

	  	  uint8_t ctrl_buf_6[2] = {0x25, 0b00000000};//CTRL REG 6 addr and data
	  	  ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, ctrl_buf_6, 2, 1000);
	  	  if ( ret != HAL_OK ) {
	  			strcpy((char*)ctrl_buf_6, "Error Tx\r\n");
	  		 }

	  	  //Read Data
	  	  int16_t Xval;
	  	  int16_t Yval;
	  	  int16_t Zval;

	  	  //Check slide 12 and internal register address is prob the buf values
	  	  uint8_t buf[6] = {0xA8};// [Zhigh, Zlow, YHigh, Ylow, Xhigh, Xlow]
	  	  ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, buf, 6, 1000);
	  	  if ( ret != HAL_OK ) {
	  			strcpy((char*)buf, "Error Tx\r\n");
	  		 }
	  	  else {
	  			 ret =  HAL_I2C_Master_Receive(&hi2c1, ACC_R_M, buf, 6, 1000); // [Zhigh, Zlow, YHigh, Ylow, Xhigh, Xlow]
	  		 }
	  	  Zval = ((buf[0]<<8) + buf[1]);
	  	  Zvalarr[arraycount%4] = Zval;
	  	  arraycount++;
	  	  int Zavg = 0;
	  	  for(int i = 0; i < 4; i++)
	  	  {
	  		  Zavg += Zvalarr[i];
	  	  }
	  	  Zavg = Zavg/4;

	  	  Yval = ((buf[2]<<8) + buf[3]);
	  	  Xval = ((buf[4]<<8) + buf[5]);

	  	  double ZGs = Zval/20224.00;
	  	  double YGs = Yval/20224.00;
	  	  double XGs = Xval/20224.00;

	  //Flex Sensor Convert
	  uint32_t ADC_VAL[5];
	  HAL_ADC_Start_DMA(&hadc1, ADC_VAL, 5);
//	uint8_t ch = ADC_VAL[0];
	//HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//	  HAL_ADC_Start(&hadc1);//start conversion
//	  HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);//wait for conversion to finish
//	  uint32_t ADC_VAL0 = ADC_VAL[0];//retrieve value
//	  uint32_t ADC_VAL2 = ADC_VAL[1];
//	  uint32_t ADC_VAL4 = ADC_VAL[2];
//	  uint32_t ADC_VAL6 = ADC_VAL[3];
//	  uint32_t ADC_VAL8 = ADC_VAL[4];



	  	  int pinky;
	  	  int ring;
	  	  int middle;
	  	  int index;
	  	  int thumb;

	  	  if(ADC_VAL[0] > 700){
	  		  pinky = 1;
	  	  }
	  	  else{
	  		  pinky = 0;
	  	  }
	  	if(ADC_VAL[1] > 700){
	  		  ring = 1;
	  	}
	  	else{
	  		  ring = 0;
	  	}
	  	if(ADC_VAL[2] > 700){
	  		  middle = 1;
	  	}
	  	else{
	  		  middle = 0;
	  	}
	  	if(ADC_VAL[3] > 700){
	  		  index = 1;
	  	}
	  	else{
	  		  index = 0;
	  	}
	  	if(ADC_VAL[4] > 700){
	  		  thumb = 5;
	  	}
	  	else{
	  		  thumb = 0;
	  	}

	  	int digit = ring + pinky + middle + index + thumb;

	  	if(Zavg < 0){
	  		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
	  		total = total*10;
	  		total += digit;
	  		uint8_t* val = &total;
		  	HAL_UART_Transmit(&huart1, val, 1, 0xFFFF);
	  		while (Zavg < 0)
	  		{
	  			uint8_t buf[6] = {0xA8};// [Zhigh, Zlow, YHigh, Ylow, Xhigh, Xlow]
				ret = HAL_I2C_Master_Transmit(&hi2c1, ACC_W_M, buf, 6, 1000);
				  if ( ret != HAL_OK ) {
						strcpy((char*)buf, "Error Tx\r\n");
					 }
				  else {
						 ret =  HAL_I2C_Master_Receive(&hi2c1, ACC_R_M, buf, 6, 1000); // [Zhigh, Zlow, YHigh, Ylow, Xhigh, Xlow]
					 }
				  Zval = ((buf[0]<<8) + buf[1]);
				  Zvalarr[arraycount%4] = Zval;
				  arraycount++;
				  Zavg = 0;
				  for(int i = 0; i < 4; i++)
				  {
				  	Zavg += Zvalarr[i];
				  }
				  Zavg = Zavg/4;
	  			//Only loop again once Zavgh is greater than 0
	  		}
	  	}
	  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

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
