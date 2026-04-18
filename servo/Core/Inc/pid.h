/*
 * pid.h
 *
 *  Created on: May 11, 2025
 *      Author: liu
 */
void pid_position_control(int positionx,int positiony,int length,int height);
void sendoutput();
void servo_PWM_control(int output1,int output2);
