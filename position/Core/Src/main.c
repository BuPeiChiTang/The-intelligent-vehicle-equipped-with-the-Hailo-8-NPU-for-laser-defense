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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"oled.h"
#include <stdbool.h>
#include<stdio.h>
#include"transmit.h"
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
     bool begin_flag=true;
     bool begin_game_flag=true;
     bool begin_transmit=false;
     extern DMA_HandleTypeDef hdma_usart1_rx;
 //    char temp[10];
     float temperature=28.0;
 	float speed;
int count;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim6){
     begin_flag=true;

	}
}
int upedge[4],downedge[4];
float distance[4];
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if(htim==&htim1&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4){
		upedge[0]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		downedge[0]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		distance[0]=((downedge[0]-upedge[0])*speed/2000000);


	}
	else if(htim==&htim2&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){
		upedge[1]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		downedge[1]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		distance[1]=((downedge[1]-upedge[1])*speed/2000000);

	}
	else if(htim==&htim3&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){
			upedge[2]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			downedge[2]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			distance[2]=((downedge[2]-upedge[2])*speed/2000000);

		}
	else if(htim==&htim8&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){
			upedge[3]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			downedge[3]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			distance[3]=((downedge[3]-upedge[3])*speed/2000000);//单位：米

		}


}
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
//
//	if(huart==&huart1)
//	{
//
//temperature=stringToFloat(temp);
//	if(temperature<=22||temperature>40)temperature=28.0;
//
//	}
//}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(20);
    OLED_Init();
    HAL_Delay(20);
//  OLED_NewFrame();
//  	OLED_PrintString(13, 0 ,"wait for data" , &font16x16, OLED_COLOR_NORMAL);
//  	 OLED_ShowFrame();
    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
//    HAL_UART_Receive_DMA(&huart1, temp, sizeof(temp));
//
//    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

HAL_TIM_Base_Start(&htim1);
HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_3);
HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
HAL_TIM_Base_Start(&htim2);
HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
HAL_TIM_Base_Start(&htim3);
HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
HAL_TIM_Base_Start(&htim8);
HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_1);
HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_2);
HAL_TIM_Base_Start_IT(&htim6);
	speed=331.4+0.6*temperature;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {if(!begin_game_flag&&HAL_GPIO_ReadPin(begin_flag_GPIO_Port, begin_flag_Pin)==GPIO_PIN_SET){
	  begin_game_flag=true;
	  HAL_TIM_Base_Start_IT(&htim6);
  }
  if(!begin_transmit&&HAL_GPIO_ReadPin(transmit_GPIO_Port, transmit_Pin)==GPIO_PIN_SET){
	  begin_transmit=true;
  }
  if(begin_transmit&&HAL_GPIO_ReadPin(transmit_GPIO_Port, transmit_Pin)==GPIO_PIN_RESET){
	  begin_transmit=false;
  }

	  if(begin_flag){
		if(begin_transmit) Send_Floats( distance[0],distance[1], distance[2],distance[3]);
		 count++;
		   static char buffer22[16];
			   static char buffer23[16];
			   static char buffer24[16];
			   static char buffer25[16];
					 snprintf(buffer22, sizeof(buffer22), "%.2fm", distance[0]);
					 snprintf(buffer23, sizeof(buffer23), "%.2fm", distance[1]);
					 snprintf(buffer24, sizeof(buffer24), "%.2fm", distance[2]);
					 snprintf(buffer25, sizeof(buffer25), "%.2fm", distance[3]);
				 OLED_NewFrame();
					OLED_PrintString(13,0 , buffer22, &font16x16, OLED_COLOR_NORMAL);
					OLED_PrintString(13, 15 , buffer23, &font16x16, OLED_COLOR_NORMAL);
					OLED_PrintString(13, 30 , buffer24, &font16x16, OLED_COLOR_NORMAL);
				OLED_PrintString(13, 45 , buffer25, &font16x16, OLED_COLOR_NORMAL);
				 OLED_ShowFrame();
		HAL_GPIO_WritePin(trig1_GPIO_Port,trig1_Pin, GPIO_PIN_SET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(trig1_GPIO_Port,trig1_Pin, GPIO_PIN_RESET);
	     __HAL_TIM_SET_COUNTER(&htim1,0);

	 	HAL_GPIO_WritePin(trig2_GPIO_Port,trig2_Pin, GPIO_PIN_SET);
	 	HAL_Delay(2);
	 	HAL_GPIO_WritePin(trig2_GPIO_Port,trig2_Pin, GPIO_PIN_RESET);
	      __HAL_TIM_SET_COUNTER(&htim2,0);

	  	HAL_GPIO_WritePin(trig3_GPIO_Port,trig3_Pin, GPIO_PIN_SET);
	  	HAL_Delay(2);
	  	HAL_GPIO_WritePin(trig3_GPIO_Port,trig3_Pin, GPIO_PIN_RESET);
	       __HAL_TIM_SET_COUNTER(&htim3,0);

	   	HAL_GPIO_WritePin(trig4_GPIO_Port,trig4_Pin, GPIO_PIN_SET);
	   	HAL_Delay(2);
	   	HAL_GPIO_WritePin(trig4_GPIO_Port,trig4_Pin, GPIO_PIN_RESET);
	        __HAL_TIM_SET_COUNTER(&htim8,0);
	  begin_flag=false;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
