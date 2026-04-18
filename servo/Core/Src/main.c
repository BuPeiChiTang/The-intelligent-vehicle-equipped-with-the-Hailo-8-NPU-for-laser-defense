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
#include "pid.h"
#include"usart recieve.h"
#include <stdbool.h>
#include "oled.h"
extern DMA_HandleTypeDef hdma_usart3_rx;
extern float Kp,Ki,Kd;
extern float out1,out2;
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
char data[30];
char check[30];
int position[4];
bool flag_key1=true;
bool flag_key2=true;
int count_key2;
//float dt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if(huart==&huart3)
	{ //  HAL_UART_Transmit(&huart3, (uint8_t*)data, Size,10);

	 Parse_Int_Packet(data, position);
//	  static char buffer[28];
//
//		     /* 安全格式化（限制缓冲区访问） */
//		   int len= snprintf(buffer, sizeof(buffer), "%d,%d,%.1f,%.1f\r\n", position[0]-320, position[1]-240,out1,out2);
//





		  //      HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
		      //  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len,49);


	 pid_position_control( position[0]-304, position[1]-282,position[2],position[3]);//先发的是x后发的是y
            //  sendoutput();
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data, sizeof(data));
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);


	}

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_Delay(20);
//  OLED_Init();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//  HAL_UART_Receive_IT(&huart2, data, 3);
//  	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1530);
//  	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1530);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data, sizeof(data));
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);

	//  HAL_UART_Transmit(&huart3, (uint8_t*)"aaa", 3,10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //死区调试代码起始
//	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1600);
//	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1600);
//HAL_Delay(50);
//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1700);
//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1700);
//HAL_Delay(50);
//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1400);
//	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1400);
//	  HAL_Delay(50);
//	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1300);
//	  	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1300);
//	  	  HAL_Delay(50);
	  //死区调试代码结束
	  //HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin);
//  if(HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin)==GPIO_PIN_RESET&&flag_key1){
//	  flag_key1=false;
//	  if(count_key2==0){
//		  Kp+=0.2;
//	  }
//	  else if(count_key2==1){
//		  Ki+=0.1;
//	  }
//	  else if(count_key2==2){
//		  Kd+=0.02;
//	  }
//
//
//  }
//  if(HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin)==GPIO_PIN_RESET&&flag_key2){
//	  flag_key2=false;
//	  count_key2++;
//	  if(count_key2>2)count_key2=0;
//  }
//  static char buffer22[16];
//  static char buffer23[16];
//  static char buffer24[16];
//  static char buffer25[16];
// snprintf(buffer22, sizeof(buffer22), "kp:%.2f", Kp);
// snprintf(buffer23, sizeof(buffer23), "kd:%.2f", Ki);
// snprintf(buffer24, sizeof(buffer24), "ki:%.2f", Kd);
// snprintf(buffer25, sizeof(buffer25), "count:%d", count_key2);
//OLED_NewFrame();
//OLED_PrintString(13, 0 , buffer22, &font16x16, OLED_COLOR_NORMAL);
//OLED_PrintString(13, 15 , buffer23, &font16x16, OLED_COLOR_NORMAL);
//OLED_PrintString(13, 30 , buffer24, &font16x16, OLED_COLOR_NORMAL);
//OLED_PrintString(13, 45 , buffer25, &font16x16, OLED_COLOR_NORMAL);
//
//OLED_ShowFrame();
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
