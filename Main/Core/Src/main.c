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

#include <stdbool.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define LCD_ADDRESS 0x4E
#define INVALID_VAL 10

#define CLR 0 // Clear
#define ADD 1 // Add
#define SUB 2 // Subtract
#define MUL 3 // Multiply
#define DIV 4 // Divide
#define EQL 5 // Equal
#define NEG 6 // Negate
#define MOD 7 // Modulo
#define BSP 8 // Backspace

#define OVF_ERR 11 // Overflow Error
#define DIV_ERR 12 // Divide by 0 Error
#define INV_OP_ERR 13 //Invalid Operation Error
#define MOD_ERR 14 // Modulo by 0 Error

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//int curr_number = 0;
double first_num = 0; // Number on first row
int first_num_neg = 0; // 0: first_num is positive, 1: first_num is negative
double second_num = 0; // Number on second row
int second_num_neg = 0; // 0: second_num is positive, 1: second_num is negative
int current_num_update = 1; // Number glove is updating first or second number
int operation = ADD; // Operation
double result = 0; // Result
int result_ready = 0; // 0: Result is not ready to display, 1: Result is ready to display

uint8_t operation_val = INVALID_VAL;
uint8_t digit_val = INVALID_VAL;
/*
* reset_flag dictates how much of the board to reset
* 1 : All lines are reset, used to start new calculation
* 2 : Only second line and results are reset, used to chain operations
*/
int reset_flag = 0;

/* error_flag specifications
 * 0 = No Error
 * 11 = Overflow Error
 * 12 = Divide by 0 Error
 * 13 = Invalid Operation Error
 * 14 = Modulo by 0 Error
 */
int error_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void handle_reset();
void handle_negate();
void print_error();
void execute_calculation();
void handle_equal();
void handle_clear();
void handle_backspace();
void update_operation();
void update_digit();
void update_display();
void handle_operation_val();
void hangle_digit_val();
void handle_error();
void test();
void lcd_send_cmd(char);
void lcd_send_char(char);
void lcd_clear();
void lcd_send_string(char *);
void lcd_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Resets LCD Display
void handle_reset() {

	if(!reset_flag)
	{
		return;
	}
	lcd_clear();

	if(reset_flag == 1)
	{
		first_num = 0;
		first_num_neg = 0;
		current_num_update = 1;
		operation = ADD;
	}
	second_num = 0;
	second_num_neg = 0;
	result = 0;
	result_ready = 0;

	error_flag = 0;
	reset_flag = 0;
}

// Negates the current number
void handle_negate() {
	if(current_num_update == 1)
	{
		// Negate first number
		first_num_neg = !first_num_neg;
		first_num = -1 * first_num;
	}
	else
	{
		// Negate second number
		second_num_neg = !second_num_neg;
		second_num = -1 * second_num;
	}

}

// Print error to display
void print_error() {
	lcd_clear();
	if(error_flag == OVF_ERR)
	{
		lcd_send_cmd(0x00|0x80);
		lcd_send_string("Overflow Error");
	}
	if(error_flag == DIV_ERR)
	{
		lcd_send_cmd(0x00|0x80);
		lcd_send_string("Divide By 0 Error");
	}
	if(error_flag == INV_OP_ERR)
	{
		lcd_send_cmd(0x00|0x80);
		lcd_send_string("Invalid Operation");
		lcd_send_cmd(0x40|0x80);
		lcd_send_string("Error");
	}
	if(error_flag == MOD_ERR)
	{
		lcd_send_cmd(0x00|0x80);
		lcd_send_string("Modulo By 0 Error");
	}
	HAL_Delay(5000);
	reset_flag = 1;
	handle_reset();
}

//Executes calculations, and allows result to be displayed
void execute_calculation() {

	if (operation == ADD) {
		result = first_num + second_num;
	}
	else if (operation == SUB) {
		result = first_num - second_num;
	}
	else if (operation == MUL) {
		result = first_num * second_num;
	}
	else if (operation == DIV) {
		// Check for divide by 0 error
		if (second_num == 0) {
			error_flag = DIV_ERR;
			return;
		}
		result = first_num / second_num;
	}
	else if (operation == MOD){
		// Check for modulo by 0 error
		if (second_num == 0) {
			error_flag = MOD_ERR;
			return;
		}
		result = (long long)first_num % (long long)second_num;
	}
	result_ready = 1;

}

// Prepares display to be reset, and calls execute_calculations
void handle_equal() {
	reset_flag = 1;
	execute_calculation();
}

// Clears display
void handle_clear() {
	reset_flag = 1;
	handle_reset();
	lcd_send_cmd(0x00|0x80);
	lcd_send_string("Display Cleared");
	HAL_Delay(3000);
	lcd_clear();
}

//Deletes the ones digit of the current number
void handle_backspace() {

	if(current_num_update == 1)
	{
		// Update first_num
		int last_digit = first_num % 10.0;
		first_num = first_num - last_digit;
		first_num = first_num / 10.0;

	}
	else
	{
		// Update second_num
		int last_digit = second_num % 10.0;
		second_num = second_num - last_digit;
		second_num = second_num / 10.0;
	}
}

// Updates operation
void update_operation()
{
	//Chain operations
	if(reset_flag == 2)
	{
		execute_calculation();
		first_num = result;
		if(first_num >= 100000000000000000.0)
		{
			error_flag = OVF_ERR;
			return;
		}
		handle_reset();
	}

	// Update the operation
	operation = operation_val;

	// Start updating the second number
	current_num_update = 2;
}

//Updates digit
void update_digit() {

	if(current_num_update == 1)
	{
		// Update the first number
		first_num *= 10;
		if(first_num_neg == 0)
		{
			// First number is positive
			first_num += digit_val;
		}
		else
		{
			// First number is negative
			first_num -= digit_val;
		}
	}

	if(current_num_update == 2)
	{
		// Update the second number
		second_num *= 10;
		if(second_num_neg == 0)
		{
			// Second number is positive
			second_num += digit_val;
		}
		else
		{
			// Second number is negative
			second_num -= digit_val;
		}
	}
}

// Update display
void update_display()
{
	// Buffer to hold string representation of numbers
	char numstr[40];

	// Make space for potential sign of first number
	if(first_num_neg)
	{
		lcd_send_cmd(0x00|0x80);
	}
	else
	{
		lcd_send_cmd(0x01|0x80);
	}
	// Update first number
	snprintf(numstr, 18, "%.5f", first_num);
	lcd_send_string(numstr);

	// Update operation
	lcd_send_cmd(0x13|0x80);
	if(operation == ADD)
	{
		lcd_send_char('+');
	}
	else if(operation == SUB)
	{
		lcd_send_char('-');
	}
	else if(operation == MUL)
	{
		lcd_send_char('*');
	}
	else if(operation == DIV)
	{
		lcd_send_char('/');
	}
	else if(operation == MOD)
	{
		lcd_send_char('%');
	}
	else
	{
		lcd_send_char(' ');
	}

	// Make space for potential sign of second number
	if(second_num_neg)
	{
		lcd_send_cmd(0x40|0x80);
	}
	else
	{
		lcd_send_cmd(0x41|0x80);
	}
	snprintf(numstr, 18, "%.5f", second_num);
	lcd_send_string(numstr);

	//Display equal sign and result if result is ready
	if(result_ready)
	{
		lcd_send_cmd(0x53|0x80);
		lcd_send_char('=');

		snprintf(numstr, 40, "%.10f", result);
		// Break result into 2 parts to display on third and fourth line
		char res_front[20];
		char res_back[20];
		memcpy(res_front, numstr, 20*sizeof(*numstr));
		memcpy(res_back, &numstr[20], 20*sizeof(*numstr));

		// Display third line
		lcd_send_cmd(0x14|0x80);
		lcd_send_string(res_front);

		// Display fourth line
		if(res_back[0] != '\0')
		{
			lcd_send_cmd(0x54|0x80);
			lcd_send_string(res_back);
		}
	}
}

// Handles value received from operation glove
void handle_operation_val() {
	if (operation_val == NEG)
	{
		handle_negate();
	}
	else if(operation_val == EQL)
	{
		handle_equal();
	}
	else if(operation_val == CLR)
	{
		handle_clear();
	}
	else if(operation_val == BSP)
	{
		handle_backspace();
	}
	else if(operation_val > 8)
	{
		//Invalid Operation
		error_flag = INV_OP_ERR;
		return;
	}
	else
  	{
		update_operation();
  	}

	//Update display
	update_display();
}

// Handles value received from digit glove
void handle_digit_val() {

	//Clear screen to allow for next calculation if needed
	if(reset_flag == 1)
	{
		handle_reset();
	}
	if(current_num_update == 1)
	{
		// If first_number will overflow
		// can only display 18 digits
		if(first_num >= 100000000000000000.0)
		{
			error_flag = OVF_ERR;
			return;
		}
	}
	if(current_num_update == 2)
	{
		// If second_number will overflow
		// can only display 18 digits
		if(second_num >= 100000000000000000.0)
		{
			error_flag = OVF_ERR;
			return;
		}
		// If an operation is inputed after the second number,
		// do calculation and move second_num to first_num
		// to allow for chaining operations
		reset_flag = 2;
	}

	update_digit();

	//Update display
	update_display();
}

// Calls print error if necessary
void handle_error()
{
	if(error_flag)
	{
		print_error();
	}
}

//Test function
void test() {
	digit_val = 4;
	handle_digit_val();
}

/* Sends a command to the LCD display
 * Used for:
 * Changing cursor location
 * Changing display mode
 * Initializing display
 * etc.
 */
void lcd_send_cmd(char cmd) {
	char data_u, data_l;
	uint8_t ctrl_buf[4];

	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	ctrl_buf[0] = data_u|0x0C;
	ctrl_buf[1] = data_u|0x08;
	ctrl_buf[2] = data_l|0x0C;
	ctrl_buf[3] = data_l|0x08;

	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, ctrl_buf, 4, 100);
}

// Writes a character to the LCD display
void lcd_send_char(char cha) {
	char data_u, data_l;
	uint8_t ctrl_buf[4];

	data_u = (cha&0xf0);
	data_l = ((cha<<4)&0xf0);
	ctrl_buf[0] = data_u|0x0D;
	ctrl_buf[1] = data_u|0x09;
	ctrl_buf[2] = data_l|0x0D;
	ctrl_buf[3] = data_l|0x09;

	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, ctrl_buf, 4, 100);
}

// Clears LCD display
void lcd_clear() {
	lcd_send_cmd(0x80);
	for(int i = 0; i < 80; i++)
	{
		lcd_send_char(' ');
	}
}

// Helper function to display a string to the LCD display
void lcd_send_string(char* str){
	while(*str)
	{
		lcd_send_char(*str++);
	}
}

// Initializes LCD display according to datasheet
void lcd_init() {
	HAL_Delay(50);  // wait for >40ms after Vcc rises to 2.7V
	lcd_send_cmd (0x30);
	HAL_Delay(10);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	lcd_send_cmd (0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0F); // Display on/off control --> D, C, B= 1 Display, Cursor, and Blinking on
	HAL_Delay(1);
	lcd_send_cmd(0x00|0x80); // Reset cursor to beginning
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
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LCD
  lcd_init();

  //Testing the LCD
  // Uncomment to run tests
  //test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  operation_val = INVALID_VAL;
	  digit_val = INVALID_VAL;
//	  HAL_Delay(50);
	  HAL_UART_Receive(&huart1, &operation_val, 1, 100);
//	  HAL_Delay(50);
	  HAL_UART_Receive(&huart6, &digit_val, 1, 100);
//	  HAL_Delay(50);

	  if (operation_val != INVALID_VAL) {
		  handle_operation_val();
	  }
	  if (digit_val != INVALID_VAL) {
		  handle_digit_val();
	  }

	  handle_error();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
