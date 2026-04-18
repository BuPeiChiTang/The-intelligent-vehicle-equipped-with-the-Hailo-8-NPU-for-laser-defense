/*
 * pid.h
 *
 *  Created on: Apr 27, 2025
 *      Author: liu
 */

#ifndef INC_PID_H_
#define INC_PID_H_
//float pid_speed_control_forward();
//void Send_Floats(float f1, float f2, float f3);
void Send_Floats( float f1, float f2, float f3,float f4);
void pid_speed_control(float v1,float v2,float v3,float v4);
void PositionPID_Control(float TARGET_SPEED,float right_distance);
void motor_PWM_control4(int output1,int output2,int output3 ,int output4);
void reinit();
//float pid_speed_control_forward_low();
#endif /* INC_PID_H_ */
