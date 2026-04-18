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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include"pid.h"
#include"icm20948.h"
#include"icm20948_ahrs.h"
#include"position.h"
#include"oled.h"
#include"MPU6050.h"
#include"recieve data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//      char message[20]="a";
//		char message2[20]="a";
//		char message3[20]="a";
//		float speed=0.0;
//		float out1;
//		uint16_t count=0;
//		int sum=0;
//	    float target,actual,errsum;
//   axises gyro, accel, mag;
//axises my_fiterdata;
//Attitude my_attitude;
//AttitudeSolver_t attitude;
//Attitude_t angles;
extern float last_gyro_zero_z;
extern DMA_HandleTypeDef hdma_usart2_rx;
bool calibrationComplete = false;
bool flag=1;
bool game_begin=false;
//bool game_begin=true;
bool stop_flag=false;
bool half_flag=false;
bool position_state=false;
bool rotate_state=false;
char position_data[64];
float positiondata_float[4];
int count=1;
int rotate_count=0;
int check;

extern int count_now1,count_now2,count_now3,count_now4;
MPU6050 MM;

   //   uint8_t flag,flag_test_PID=0;
      float count1,count2,count3,count4;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim6){
static int count5=0;
 count5++;
MPU6050_Get_Angle_Plus(& MM) ;
static uint8_t buffer[30];
			 sprintf(buffer,"%f",MM.yaw);
			  OLED_NewFrame();
			 OLED_PrintString(13, 30 , buffer, &font16x16, OLED_COLOR_NORMAL);
			  OLED_ShowFrame();
      if(count5==6){

		 if(MM.yaw*3<=65) {pid_speed_control(-40, 40, -40, 40);}
		 else if(MM.yaw*3<=87){pid_speed_control(-20,20, -20, 20);}
//		 else if(MM.yaw*3>93){
//			 pid_speed_control(-10,10, -10, 10);
//		 }
		 else {
			 motor_PWM_control4(0,0,0,0);

				 rotate_state=true;
				  reinit();
				 HAL_TIM_Base_Stop_IT(&htim6);
				 __HAL_TIM_CLEAR_FLAG(&htim6,TIM_IT_UPDATE);
				 __HAL_TIM_DISABLE_IT(&htim6,TIM_IT_UPDATE);
		 }
count5=0;
	}
	}


}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if(huart==&huart2)
	{check++;
		//HAL_UART_Transmit(&huart2, (uint8_t*)position_data, sizeof(position_data),Size);
		Unpack_Floats(position_data,positiondata_float);
 //int aaa=Unpack_Floats(position_data,positiondata_float);

//if(aaa==4){


if(positiondata_float[2]>=2){
	half_flag=true;

}
if(half_flag){
	if(positiondata_float[0]<=0.5){
		 motor_PWM_control4(0,0,0,0);
position_state=true;

	}
}
			  PositionPID_Control(30,positiondata_float[1]);




//			if(positiondata_float[0]<=1.20){
//				 PositionPID_Control(20,positiondata_float[1]);
//			}
//			else if (positiondata_float[0]<=1.05) {
//				//Second_Level_Control(positiondata_float, 0,1);
//				 motor_PWM_control4(0,0,0,0);
//				 position_state=true;
//			}
//			else{
//				 PositionPID_Control(30,positiondata_float[1]);
//			}


//		if(count==?){
//			//降速
//		}
	  memset(position_data, 0, sizeof(position_data));
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)position_data, sizeof( position_data));
		 __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);


	}

}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_SPI2_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(20);
  OLED_Init();


  OLED_NewFrame();
 OLED_PrintString(13, 30 , "wait", &font16x16, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
  MPU6050_init(&hi2c1);
  OLED_NewFrame();
  OLED_PrintString(13, 30 , "ok", &font16x16, OLED_COLOR_NORMAL);
   OLED_ShowFrame();
//  system_init();
//
//		    // 初始化姿态解算器
//		      Attitude_Init(&attitude);
//
//		      // 校准陀螺仪Z轴
//  Attitude_Calibrate_Gyro_Z(&attitude, 500);
//     Attitude_Calibrate_Accel(&attitude, 500);     // 加速度计校准
// icm20948_gyro_read_dps(&gyro);
//  icm20948_accel_read_g(&accel);
 // 更新姿态（选择一种算法）
 //   Attitude_Update_Complementary(&attitude, &gyro, &accel, 0.01f);
		     	        // 或者使用四元数算法
 // Attitude_Update_Quaternion(&attitude, &gyro, &accel, 0.01f);

		     	        // 获取姿态角
//  Attitude_Get_Angles(&attitude, &angles);
//   static char buffer22[16];
//	 snprintf(buffer22, sizeof(buffer22), "yaw:%.2f", angles.yaw);
// OLED_NewFrame();
//OLED_PrintString(13, 30 , buffer22, &font16x16, OLED_COLOR_NORMAL);
// OLED_ShowFrame();

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);

 //HAL_TIM_Base_Start_IT(&htim6);
 // HAL_TIM_Base_Stop_IT(&htim6);

//  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)position_data, sizeof( position_data));
// __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
//HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  if(rotate_state){/
//		  HAL_TIM_Base_Stop_IT(&htim6);
//	  }
//	     static char buffer22[16];
//	  	 snprintf(buffer22, sizeof(buffer22), "yaw:%.2f", MM.yaw);
//	   OLED_NewFrame();
//	  OLED_PrintString(13, 30 , buffer22, &font16x16, OLED_COLOR_NORMAL);
//	   OLED_ShowFrame();
	  //********************************************
	  if(HAL_GPIO_ReadPin(begin_flag_GPIO_Port, begin_flag_Pin)==GPIO_PIN_SET&&!game_begin){
		  game_begin=true;

	  }
//	  if(HAL_GPIO_ReadPin(stop_flag_GPIO_Port, stop_flag_Pin)==GPIO_PIN_SET&&!stop_flag){
//		  stop_flag=true;
//		  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_ALL);
//		  HAL_TIM_Base_Stop_IT(&htim6);
//		  HAL_UART_DMAStop(&huart2);
//	  }
	  if(game_begin&&!stop_flag){

//
//	  if(HAL_GPIO_ReadPin(stop_flag_GPIO_Port, stop_flag_Pin)==GPIO_PIN_SET&&!stop_flag){
//		  HAL_TIM_Base_Stop_IT(&htim6);
//		  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_ALL);
//		  game_begin=false;  }


	  if(count==1&&!position_state&&!rotate_state){
		  HAL_TIM_Base_Stop_IT(&htim6);
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)position_data, sizeof( position_data));
		 __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		 HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_SET);

	  }
	  else if(count==1&&!rotate_state&&position_state){
		  HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_RESET);
		  HAL_UART_DMAStop(&huart2);
		  memset(position_data, 0, sizeof(position_data));
		  MPU6050_FastResetYaw_Complementary(&MM);
		  __HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);
				  HAL_TIM_Base_Start_IT(&htim6);
	  }
	  else if(count==1&&position_state&&rotate_state){
		  count++;
		  position_state=false;
			  rotate_state=false;
			  HAL_TIM_Base_Stop_IT(&htim6);
			  __HAL_TIM_DISABLE_IT(&htim6,TIM_IT_UPDATE);
	  }
//***********************************************
	  if(count==2&&!position_state&&!rotate_state){
		  HAL_TIM_Base_Stop_IT(&htim6);
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)position_data, sizeof( position_data));
		 __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);

		 HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_SET);
	  }
	  else if(count==2&&!rotate_state&&position_state){
		  __HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);

		  HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_RESET);
		  HAL_UART_DMAStop(&huart2);
			  memset(position_data, 0, sizeof(position_data));
			  MPU6050_FastResetYaw_Complementary(&MM);


			  HAL_TIM_Base_Start_IT(&htim6);
	  }
	  else if(count==2&&position_state&&rotate_state){
		  count++;
		  position_state=false;
			  rotate_state=false;
			  HAL_TIM_Base_Stop_IT(&htim6);
			  __HAL_TIM_DISABLE_IT(&htim6,TIM_IT_UPDATE);
	  }
	  //**********************************************
	  if(count==3&&!position_state&&!rotate_state){
		  HAL_TIM_Base_Stop_IT(&htim6);
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)position_data, sizeof( position_data));
		 __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);

		 HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_SET);
	  }
	  else if(count==3&&!rotate_state&&position_state){
		  HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_RESET);
		  HAL_UART_DMAStop(&huart2);
			  memset(position_data, 0, sizeof(position_data));
			  MPU6050_FastResetYaw_Complementary(&MM);
			  __HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);
					  HAL_TIM_Base_Start_IT(&htim6);
	  }
	  else if(count==3&&position_state&&rotate_state){
		  count++;
		  position_state=false;
			  rotate_state=false;
			  HAL_TIM_Base_Stop_IT(&htim6);
	  }
	  //*******************************************
	  if(count==4&&!position_state&&!rotate_state){
		  HAL_TIM_Base_Stop_IT(&htim6);
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)position_data, sizeof( position_data));
		 __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
		 HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_SET);
	  }
	  else if(count==4&&!rotate_state&&position_state){
		  __HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);
		  HAL_GPIO_WritePin(transmit_GPIO_Port, transmit_Pin, GPIO_PIN_RESET);
		  HAL_UART_DMAStop(&huart2);
			  memset(position_data, 0, sizeof(position_data));
			  MPU6050_FastResetYaw_Complementary(&MM);

					  HAL_TIM_Base_Start_IT(&htim6);
	  }
	  else if(count==4&&position_state&&rotate_state){
		  count=1;
		  position_state=false;
		  rotate_state=false;
		  HAL_TIM_Base_Stop_IT(&htim6);

	  }
	  }
//**********************************************************************
//
//
//	  }
//  OLED_NewFrame();
////  sprintf(message,"speed: %.3f",speed);
//  sprintf(message,"out: %.3f",out1);
//    sprintf(message2,"count: %d",count);
//    sprintf(message3,"sum: %d",sum);
//OLED_PrintString(13, 0, message, &font16x16, OLED_COLOR_NORMAL);
// OLED_PrintString(13, 15 , message2, &font16x16, OLED_COLOR_NORMAL);
//  // OLED_PrintString(13, 30 , "target: 49.5", &font16x16, OLED_COLOR_NORMAL);
////
// OLED_PrintString(13, 45 , message3, &font16x16, OLED_COLOR_NORMAL);
//  OLED_ShowFrame();
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
