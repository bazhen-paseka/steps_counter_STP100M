
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

	//	Crystal 8MHz
	//	PB0 - LED_RED
	//  PA9 - UART debug 38400
	//  PA0 - input 1
	//	PA1 - input 2
	//	I2C	- PB6 SCL, PB7 - SDA;

	#include <string.h>
	#include "lcd1602_fc113_sm.h"

	// next variables used in IRQ. See file stm32f1xx_it.c
	volatile  uint8_t	   time_to_show_u8 = 0 ;		// base on TIm3
	volatile uint32_t	step_counter_1_u32 = 0 ;		//	PA0
	volatile uint32_t	step_counter_2_u32 = 0 ;		//	PA1

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	uint32_t	step_buff_1_u32 = 1;
	uint32_t	step_buff_2_u32 = 1;

	char uart_buffer_c[100];
	sprintf(uart_buffer_c,"\r\nSteps counter STP100M\r\nUART1 for debug started. Speed 38400\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer_c, strlen(uart_buffer_c), 100);


	#define ADR_I2C_FC113 0x27

	lcd1602_fc113_struct h1_lcd1602_fc113 =
	{
		.i2c = &hi2c1,
		.device_i2c_address = ADR_I2C_FC113
	};

	LCD1602_Init(&h1_lcd1602_fc113);
	I2C_ScanBus(&h1_lcd1602_fc113);

	LCD1602_Clear(&h1_lcd1602_fc113);
	sprintf(uart_buffer_c,"Steps Counter\r\n");
	LCD1602_Print_Line(&h1_lcd1602_fc113, uart_buffer_c, strlen(uart_buffer_c));

	sprintf(uart_buffer_c," STP100M\r\n");
	LCD1602_Print_Line(&h1_lcd1602_fc113, uart_buffer_c, strlen(uart_buffer_c));

	sprintf(uart_buffer_c,"LCD1602 over FC113\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer_c, strlen(uart_buffer_c), 100);
	HAL_Delay(1000);
	//LCD1602_Clear(&h1_lcd1602_fc113);

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);

	LCD1602_Clear(&h1_lcd1602_fc113);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if (time_to_show_u8 == 1 )
	{
		time_to_show_u8 = 0;
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);	// blink red led on board

		if (   (step_buff_1_u32 != step_counter_1_u32)
			|| (step_buff_2_u32 != step_counter_2_u32) )
		{
			sprintf(uart_buffer_c,"L: %5d\n", (int)step_counter_1_u32);
			LCD1602_Print_Line(&h1_lcd1602_fc113, uart_buffer_c, strlen(uart_buffer_c));

			sprintf(uart_buffer_c,"R: %5d\n", (int)step_counter_2_u32);
			LCD1602_Print_Line(&h1_lcd1602_fc113, uart_buffer_c, strlen(uart_buffer_c));

			sprintf(uart_buffer_c,"left: %d   right: %d\r\n", (int)step_counter_1_u32, (int)step_counter_2_u32);
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer_c, strlen(uart_buffer_c), 100);

			step_buff_1_u32 = step_counter_1_u32;
			step_buff_2_u32 = step_counter_2_u32;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
