/*
 * pid.c
 *
 *  Created on: Apr 27, 2025
 *      Author: liu
 */
//#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pid.h"
#include"math.h"
#include <stdio.h>
#define   Kd 0.36f//0.3->50ms
#define   Kp  0.5f
#define   Ki  0.42f//0.5->50ms*4 后基本稳定
#define   Kp2  50.0f // 0.12f
#define   Ki2  20.0f
#define   Kd2 1.0f

#define   rated_rotational_speed -49.5 //3圈/s 编码器计数990/s 49.5/50ms 59.4/60ms 实测50ms内编码器在空转最大计数在80左右
#define   low_forward_speed 25
	float err_now1,err_last1,err_sum1,out1,D_out1;
	int count_now1,count_last1;

	float err_now2,err_last2,err_sum2,out2,D_out2;
	int count_now2,count_last2;

	float err_now3,err_last3,err_sum3,out3,D_out3;
	int count_now3,count_last3;

	float err_now4,err_last4,err_sum4,out4,D_out4;
	int count_now4,count_last4;
    extern float count1,count2,count3,count4;


	void motor_PWM_control4(int output1,int output2,int output3 ,int output4){
		//in1 low in2 high forward
//***********************************************************************************************************************************************//
	if(output1>=0){
		HAL_GPIO_WritePin(m1_in1_GPIO_Port,m1_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m1_in2_GPIO_Port, m1_in2_Pin, GPIO_PIN_SET);//转向控制
		if(output1>0)output1+=3;//输出偏移，占空比小于3%，电机无法转动


	}

	else{

		        HAL_GPIO_WritePin(m1_in1_GPIO_Port,m1_in1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(m1_in2_GPIO_Port, m1_in2_Pin, GPIO_PIN_RESET);

						 output1-=3;
			output1= -output1;
	}
//*******************************************************************************************************************************************//
	if(output2>=0){
		HAL_GPIO_WritePin(m2_in1_GPIO_Port,m2_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m2_in2_GPIO_Port, m2_in2_Pin, GPIO_PIN_SET);//转向控制
		if(output2>0)output2+=3;//输出偏移，占空比小于3%，电机无法转动


	}

	else{

		        HAL_GPIO_WritePin(m2_in1_GPIO_Port,m2_in1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(m2_in2_GPIO_Port, m2_in2_Pin, GPIO_PIN_RESET);

						 output2-=3;
			output2= -output2;
	}
	//*******************************************************************************************************************************************//
	if(output3>=0){
		HAL_GPIO_WritePin(m3_in1_GPIO_Port,m3_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m3_in2_GPIO_Port, m3_in2_Pin, GPIO_PIN_SET);//转向控制
		if(output3>0)output3+=3;//输出偏移，占空比小于3%，电机无法转动


	}

	else{

		        HAL_GPIO_WritePin(m3_in1_GPIO_Port,m3_in1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(m3_in2_GPIO_Port, m3_in2_Pin, GPIO_PIN_RESET);

						 output3-=3;
			output3= -output3;
	}
	//*******************************************************************************************************************************************//
	if(output4>=0){
		HAL_GPIO_WritePin(m4_in1_GPIO_Port,m4_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(m4_in2_GPIO_Port, m4_in2_Pin, GPIO_PIN_SET);//转向控制
		if(output4>0)output4+=3;//输出偏移，占空比小于3%，电机无法转动


	}

	else{

		        HAL_GPIO_WritePin(m4_in1_GPIO_Port,m4_in1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(m4_in2_GPIO_Port, m4_in2_Pin, GPIO_PIN_RESET);

						 output4-=3;
			output4= -output4;
	}
//*******************************************************************************************************************************************//
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, output1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, output2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, output3);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, output4);

	}


	//***********************************************************************************************************************************************//
void Send_Floats( float f1, float f2, float f3,float f4)
	 {
	     /* 使用静态缓冲区避免栈溢出风险 */
	     static char buffer[128];

	     /* 安全格式化（限制缓冲区访问） */
	     int len = snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f\r\n", (float)f1, (float)f2, (float)f3,(float)f4);






	       // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
	        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len,49);



	         // 清除传输完成标志（可选，部分HAL库会自动清除）


	 }

//***********************************************************************************************************************************************//

	void pid_speed_control(float v1,float v2,float v3,float v4){
//*******************************************************************************************************************************************//
		count_now1=__HAL_TIM_GET_COUNTER(&htim1);
		count_now2=__HAL_TIM_GET_COUNTER(&htim2);
		count_now3=__HAL_TIM_GET_COUNTER(&htim3);
		count_now4=__HAL_TIM_GET_COUNTER(&htim4);

//*******************************************************************************************************************************************//
		if(count_now1>=1000){
			count_now1-=65535;
		}//编码器反向计数会溢出至65535
		err_last1=err_now1;
		err_now1=v1-count_now1;
		err_sum1+=err_now1;

		if(err_sum1>200)err_sum1=200;
		if(err_sum1<-200)err_sum1=-200;//积分限幅.


		D_out1=0.3*(count_now1-count_last1)*Kd+0.7*D_out1;//低通滤波+微分现行
		out1=Kp*err_now1+Ki*err_sum1+D_out1;

	    count_last1=count_now1;
		if(out1>108)out1=108;//输出限幅
		if(out1<-108)out1=-108;


//*******************************************************************************************************************************************//
		if(count_now2>=1000){
			count_now2-=65535;
		}//编码器反向计数会溢出至65535
		err_last2=err_now2;
		err_now2=v2-count_now2;
		err_sum2+=err_now2;

		if(err_sum2>200)err_sum2=200;
		if(err_sum2<-200)err_sum2=-200;//积分限幅.


		D_out2=0.3*(count_now2-count_last2)*Kd+0.7*D_out2;//低通滤波+微分现行
		out2=Kp*err_now2+Ki*err_sum2+D_out2;

	    count_last2=count_now2;
		if(out2>108)out2=108;//输出限幅
			if(out2<-108)out2=-108;


//*******************************************************************************************************************************************//
		if(count_now3>=1000){
			count_now3-=65535;
		}//编码器反向计数会溢出至65535
		err_last3=err_now3;
		err_now3=v3-count_now3;
		err_sum3+=err_now3;

		if(err_sum3>200)err_sum3=200;
		if(err_sum3<-200)err_sum3=-200;//积分限幅.


		D_out3=0.3*(count_now3-count_last3)*Kd+0.7*D_out3;//低通滤波+微分现行
		out3=Kp*err_now3+Ki*err_sum3+D_out3;

	    count_last3=count_now3;
		if(out3>108)out3=108;//输出限幅
			if(out3<-108)out3=-108;


//*******************************************************************************************************************************************//
		if(count_now4>=1000){
			count_now4-=65535;
		}//编码器反向计数会溢出至65535
		err_last4=err_now4;
		err_now4=v4-count_now4;
		err_sum4+=err_now4;

		if(err_sum4>200)err_sum4=200;
		if(err_sum4<-200)err_sum4=-200;//积分限幅.


		D_out4=0.3*(count_now4-count_last4)*Kd+0.7*D_out4;//低通滤波+微分现行
		out4=Kp*err_now4+Ki*err_sum4+D_out4;

	    count_last4=count_now4;
		if(out4>108)out4=108;//输出限幅
			if(out4<-108)out4=-108;


//*******************************************************************************************************************************************//

		motor_PWM_control4((int)out1*5/6,(int)out2*5/6,(int)out3*5/6,(int)out4*5/6);
		    count1=count_now1;
			count2=count_now2;
			count3=count_now3;
			count4=count_now4;
		__HAL_TIM_SET_COUNTER(&htim1,0);
		__HAL_TIM_SET_COUNTER(&htim2,0);
		__HAL_TIM_SET_COUNTER(&htim3,0);
		__HAL_TIM_SET_COUNTER(&htim4,0);
	}

	float wheel_speed1, wheel_speed2, wheel_speed3, wheel_speed4;
	float err_2now, err_2last, err_2sum, out_2pid, D_2out;
	void PositionPID_Control(float TARGET_SPEED,float right_distance) {

				    const float DT = 0.06f;  // 60ms控制周期
        static float SPEED_LIMIT=80.0f;


				    // 计算距离误差
				    err_2now = 0.5f - right_distance;

				    // PID计算
				    err_2sum += err_2now * DT;                  // 积分项
					if(err_2sum>20)err_2sum=20;
						if(err_2sum<-20)err_2sum=-20;
				    D_2out = (err_2now - err_2last) / DT;        // 微分项
				    out_2pid = Kp2 * err_2now + Ki2 * err_2sum + Kd2 * D_2out;  // PID输出

				    // 速度分配（仅调整右侧两轮）
				    wheel_speed1 = TARGET_SPEED;  // 左前
				    wheel_speed2 = TARGET_SPEED + out_2pid;  // 右前
				    wheel_speed3 = TARGET_SPEED+ out_2pid;  // 左后
				    wheel_speed4 = TARGET_SPEED ;  // 右后

				    // 速度限幅
				    wheel_speed2 = fminf(fmaxf(wheel_speed2, -SPEED_LIMIT), SPEED_LIMIT);
				    wheel_speed4 = fminf(fmaxf(wheel_speed4, -SPEED_LIMIT), SPEED_LIMIT);

				    // 调用底层速度PID控制
				    pid_speed_control(wheel_speed1, wheel_speed2, wheel_speed3, wheel_speed4);

				    // 保存当前误差用于下次计算
				    err_2last = err_2now;
				}
	void reinit(){
		 err_2last =0;
		 err_2sum=0;
	}
//******************************************************************************************************************************************************//
	//***********************************************************************************************************************************************//
	//***********************************************************************************************************************************************//
	//调试用函数，不可直接调用
//	float err_now,err_last,err_sum,out,D_out;
//	int count_now,count_last;
//	char usart_pid[10];
//	extern float target,actual,errsum;

	//***********************************************************************************************************************************************//
//	void motor_PWM_control(int output){//in1 low in2 high forward
//	if(output>=0){
//		HAL_GPIO_WritePin(m3_in1_GPIO_Port,m3_in1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(m3_in2_GPIO_Port, m3_in2_Pin, GPIO_PIN_SET);//转向控制
//		if(output>0)output+=3;
//		else if(output <0)output-=3;
//		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, output);//占空比控制
//
//	}
//
//	else{
//
//		        HAL_GPIO_WritePin(m3_in1_GPIO_Port,m3_in1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(m3_in2_GPIO_Port, m3_in2_Pin, GPIO_PIN_RESET);
//				if(output>0)output+=3;
//						else if(output <0)output-=3;
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, -output);
//	}
//
//	}
//float pid_speed_control_forward(){
////HAL_GPIO_TogglePin(in1_GPIO_Port, in1_Pin);
////		HAL_GPIO_TogglePin(in2_GPIO_Port, in2_Pin);
//	count_now=__HAL_TIM_GET_COUNTER(&htim3);
//	if(count_now>=1000){
//		count_now-=65535;
//	}//编码器反向计数会溢出至65535
//	err_last=err_now;
//	err_now=rated_rotational_speed-count_now;
//	err_sum+=err_now;
//	if(err_sum>200)err_sum=200;
//			if(err_sum<-200)err_sum=-200;
////if(count_now-count_last>=20||count_now-count_last<=-20){
////	err_sum=0;
////}
//			errsum=err_sum;//chuanzhi tiaoshiyong
//
//			D_out=0.3*(count_now-count_last)*Kd+0.7*D_out;//低通滤波+微分现行
//			out=Kp*err_now+Ki*err_sum+D_out;
////	out=Kp*err_now+Ki*err_sum+Kd*(count_now-count_last);
//count_last=count_now;
//	if(out>90)out=90;//输出限幅
//	if(out<-90)out=-90;
//	motor_PWM_control((int)out);
//
//target =rated_rotational_speed;
//actual=count_now;
//return out;
//}
//
//float pid_speed_control_forward_low(){
////HAL_GPIO_TogglePin(in1_GPIO_Port, in1_Pin);
////		HAL_GPIO_TogglePin(in2_GPIO_Port, in2_Pin);
//	count_now=__HAL_TIM_GET_COUNTER(&htim3);
//	if(count_now>=1000){
//		count_now-=65535;
//	}
//	err_last=err_now;
//	err_now=low_forward_speed-count_now;
//	err_sum+=err_now;
//	if(err_sum>400)err_sum=400;
//			if(err_sum<-400)err_sum=-400;
////			if(count_now-count_last>=20||count_now-count_last<=-20){
////				err_sum=0;
////			}
//			D_out=0.3*(count_now-count_last)*Kd+0.7*D_out;//低通滤波+微分现行
//					out=Kp*err_now+Ki*err_sum+D_out;
//		//	out=Kp*err_now+Ki*err_sum+Kd*(count_now-count_last);
//		count_last=count_now;
//	if(out>90)out=90;//输出限幅
//	if(out<-90)out=-90;
//	motor_PWM_control((int)out);
//	 //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//target =low_forward_speed;
//actual=count_now;
//return out;
//}
