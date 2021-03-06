/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "commands.h"
#include "hx711.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//  	HAL_ADC_Start(&hadc1);

//  	uint16_t ts_cal1 = *((uint16_t*)0x1FFFF7B8);
//  	uint16_t ts_cal2 = *((uint16_t*)0x1FFFF7C2);
//  	float Avg_Slope = ((float)(ts_cal1 - ts_cal2)) / (110 - 30);
//  	uint16_t v25 = 1774;

	int empty[] = {0,0,0,0,0,0,0,0,0,0};

	//IC3
	hx1.gpioSck = DO_SCK_1_GPIO_Port;
	hx1.gpioData = DI_DATA_1_GPIO_Port;
	hx1.pinSck = DO_SCK_1_Pin;
	hx1.pinData = DI_DATA_1_Pin;
	hx1.gain = 3;
	hx1.offsetA = 0;
	hx1.offsetB = 0;
	hx1.readingA = 0;
	hx1.readingB = 0;
	memcpy(hx1.historyA, empty, sizeof hx1.historyA);
	memcpy(hx1.historyB, empty, sizeof hx1.historyB);
	HX711_Init(hx1);

	//IC2
	hx2.gpioSck = DO_SCK_2_GPIO_Port;
	hx2.gpioData = DI_DATA_2_GPIO_Port;
	hx2.pinSck = DO_SCK_2_Pin;
	hx2.pinData = DI_DATA_2_Pin;
	hx2.gain = 3;
	hx2.offsetA = 0;
	hx2.offsetB = 0;
	hx2.readingA = 0;
	hx2.readingB = 0;
	memcpy(hx2.historyA, empty, sizeof hx2.historyA);
	memcpy(hx2.historyB, empty, sizeof hx2.historyB);
	HX711_Init(hx2);
	readFlash();

	iTare = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if(iDFU)
	  {
		HAL_Delay(1000);
		NVIC_SystemReset();
	  }

//	  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
//	  {
//		  temperature = HAL_ADC_GetValue(&hadc1);
//		  temperature = ((v25 - temperature)/Avg_Slope) + 25;
//	  }

	  if (iTare)
	  {
		HAL_GPIO_WritePin(hx1.gpioSck, hx1.pinSck, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(hx2.gpioSck, hx2.pinSck, GPIO_PIN_RESET);

	    hx1.gain = 1;
	    hx2.gain = 1;

		HX711_Average_Value(hx1, 100);
		HX711_Average_Value(hx2, 100);

		hx1.offsetA = HX711_Average_Value(hx1, 20);
		hx2.offsetA = HX711_Average_Value(hx2, 20);

		hx1.gain = 2;
		hx2.gain = 2;

		HX711_Average_Value(hx1, 100);
		HX711_Average_Value(hx2, 100);

		hx1.offsetB = HX711_Average_Value(hx1, 20);
		hx2.offsetB = HX711_Average_Value(hx2, 20);

		iTare = 0;

	 	HAL_GPIO_WritePin(hx1.gpioSck, hx1.pinSck, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hx2.gpioSck, hx2.pinSck, GPIO_PIN_SET);
		HAL_Delay(50);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(hx1.gpioSck, hx1.pinSck, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(hx2.gpioSck, hx2.pinSck, GPIO_PIN_RESET);

		  hx1.gain = 1;
		  hx2.gain = 1;
		//  HX711_Average_Value(hx1, 1);
		//  HX711_Average_Value(hx2, 1);
		  hx1.readingA = (-(HX711_Average_Value(hx1, 1) - hx1.offsetA) * ((float)iCalibration/10000))/4000;
		  hx2.readingA = (-(HX711_Average_Value(hx2, 1) - hx2.offsetA) * ((float)iCalibration/10000))/4000;

		  hx1.gain = 2;
		  hx2.gain = 2;
		  HX711_Average_Value(hx1, 1);
		  HX711_Average_Value(hx2, 1);
		  hx1.readingB = (-(HX711_Average_Value(hx1, 1) - hx1.offsetB) * ((float)iCalibration/10000))/1000;
		  hx2.readingB = (-(HX711_Average_Value(hx2, 1) - hx2.offsetB) * ((float)iCalibration/10000))/1000;


		  HAL_GPIO_WritePin(hx1.gpioSck, hx1.pinSck, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(hx2.gpioSck, hx2.pinSck, GPIO_PIN_SET);

		  HAL_Delay(50);

		  HX711_Process_Values();
	  }


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_SCK_1_GPIO_Port, DO_SCK_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_SCK_2_GPIO_Port, DO_SCK_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 PA4 PA5 
                           PA6 PA7 PA9 PA10 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_SCK_1_Pin */
  GPIO_InitStruct.Pin = DO_SCK_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_SCK_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_DATA_1_Pin DI_DATA_2_Pin */
  GPIO_InitStruct.Pin = DI_DATA_1_Pin|DI_DATA_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB15 
                           PB3 PB4 PB5 PB6 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_SCK_2_Pin */
  GPIO_InitStruct.Pin = DO_SCK_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_SCK_2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
