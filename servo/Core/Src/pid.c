/*
 * pid.c
 *
 *  Created on: May 11, 2025
 *      Author: liu
 */
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pid.h"
#include<math.h>
#include<stdio.h>
	//大误差区强kpkd小ki小误区反过来
float   Kd1 =0.005;

float  Kp1  =0.4;
float  Ki1=  0.05;
float kp_pro1=0.2;


float   Kd2 =0.005;

float  Kp2  =0.3;
float  Ki2=  0.05;
float kp_pro2=0.35;
float err_now1,err_last1,err_sum1,out1,D_out1;
	float position_lastx;

	float err_now2,err_last2,err_sum2,out2,D_out2;
	float position_lasty;
	//***********************************************************************************************************************************************//
	void servo_PWM_control(int output1,int output2){

	if(output1>=-5&&output1<=5){
		output1=0;
	}
                       output1+=1500;
                         if(output2>=-5&&output2<=5){
                         		output2=0;
                         	}
   output2+=1500;

				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, output1);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, output2);

		}
	//*******************************************************************************************************************************************//
	//期望始终为中心点0,0，舵机逆时针转动output为正，即镜头需要向左/下转动，物体中心在x负半轴，坐标为负



	//ch3 水平云台 ch4 纵向云台

	void pid_position_control(int positionx,int positiony,int length,int height){


		err_last1=err_now1;
		err_now1=0-positionx;
		err_sum1+=err_now1;

		if(err_sum1>800)err_sum1=300;
		if(err_sum1<-800)err_sum1=-300;//积分限幅.


		D_out1=0.3*(positionx-position_lastx)*Kd1+0.7*D_out1;//低通滤波+微分现行
 float err_now1_abs=fabsf(err_now1);
		if(err_now1_abs>length){out1=kp_pro1*err_now1+Ki1*err_sum1+D_out1;


			if(out1>400)out1=400;//输出限幅
						if(out1<-400)out1=-400;



		}
		else {
			out1=Kp1*err_now1+Ki1*err_sum1+D_out1;
			if(out1>5&&out1<30)out1+=28;
						if(out1<-5&&out1>-30)out1-=-28;

					if(out1>50)out1=50;//输出限幅
						if(out1<-50)out1=-50;

		}

		 position_lastx=positionx;

//*****************************************************************************//
		err_last2=err_now2;
		err_now2=0-positiony;
		err_sum2+=err_now2;

		if(err_sum2>800)err_sum2=300;
		if(err_sum2<-800)err_sum2=-300;//积分限幅.


		D_out2=0.3*(positiony-position_lasty)*Kd2+0.7*D_out2;//低通滤波+微分现行
		float err_now2_abs=fabsf(err_now2);
		if(err_now2_abs>height){ out2=kp_pro2*err_now2+Ki2*err_sum2+D_out2;

			if(out2>200)out2=400;//输出限幅
								if(out2<-200)out2=-400;

		}
		else {
			out2=Kp2*err_now2+Ki2*err_sum2+D_out2;
			if(out1>20&&out1<30)out1=30;
					if(out1<-20&&out1>-30)out1=-30;

					if(out2>80)out2=80;//输出限幅
							if(out2<-80)out2=-80;

		}

		 position_lasty=positiony;

//		if(-2<positionx<2){
//			if(-2<positiony<2){
//				servo_PWM_control( 0, 0);
//			}
//			else servo_PWM_control( 0, out2);
//		}else if(-3<positiony<3){
//			servo_PWM_control( out1, 0);
//		}
//		else
		if(positionx==0&&positiony==0){
			servo_PWM_control( 0, 0);
		}
		else servo_PWM_control( out1, out2*4/3);
	//	servo_PWM_control( out1, 0);
//		 static char buffer[128];
//
//			     /* 安全格式化（限制缓冲区访问） */
//			     int len = snprintf(buffer, sizeof(buffer), "%d,%d,%d,%d\n", (int)out1, (int)out2,positionx,positiony);
//
//
//
//
//
//
//			       // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
//			        HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len,49);
	}
void sendoutput(){




	 static char buffer[128];

		     /* 安全格式化（限制缓冲区访问） */
		     int len = snprintf(buffer, sizeof(buffer), "%d,%d,%d,%d\n", (int)out1, (int)out2,(int)err_now1,(int)err_now2);






		       // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
		        HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len,49);



}
