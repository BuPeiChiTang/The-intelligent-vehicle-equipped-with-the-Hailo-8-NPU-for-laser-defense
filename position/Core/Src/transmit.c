/*
 * transmit.c
 *
 *  Created on: Jun 25, 2025
 *      Author: liu
 */


#include"transmit.h"
#include"usart.h"
#include <stdio.h>
void Send_Floats( float f1, float f2, float f3,float f4)
	 {
	     /* 使用静态缓冲区避免栈溢出风险 */
	     static char buffer[128];

	     /* 安全格式化（限制缓冲区访问） */
	     int len = snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f\r\n", (float)f1, (float)f2, (float)f3,(float)f4);

	       // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
	        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len,10);



	 }


float stringToFloat(const char *str) {
    float result = 0.0;
    int sign = 1;
    int decimal = 0;
    float factor = 0.1;

    // 处理符号
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // 处理整数部分
    while (*str >= '0' && *str <= '9') {
        result = result * 10.0 + (*str - '0');
        str++;
    }

    // 处理小数点
    if (*str == '.') {
        str++;
        // 处理小数部分
        while (*str >= '0' && *str <= '9') {
            result = result + (*str - '0') * factor;
            factor *= 0.1;
            str++;
        }
    }

    return result * sign;
}
