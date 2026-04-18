/*
 * usart recieve.c
 *
 *  Created on: Jun 24, 2025
 *      Author: liu
 */

#include"usart recieve.h"
#include"usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/**
 * 简化版数据包解包函数
 * 功能：解析以'A'开头、'\n'结尾、逗号分隔的4个整数
 * 输入：input - 输入字符串，int_out - 整数输出数组
 * 返回：true - 解析成功，false - 解析失败
 */
bool Parse_Int_Packet(const char* input, int* int_out) {
    // 基础格式验证
    if (input == NULL || input[0] != 'A') return false;

    // 查找换行符位置并验证
    int len = 0;
    while (input[len] != '\0' && len < 64) {
        len++;
    }
    if (len < 3 || input[len-1] != '\n') return false;

    // 手动分割和解析4个整数
    int token_idx = 0;
    int num_idx = 0;
    int_out[0] = int_out[1] = int_out[2] = int_out[3] = 0;

    // 从帧头之后开始解析
    for (int i = 1; i < len-1; i++) {  // 跳过'A'和'\n'
        if (input[i] >= '0' && input[i] <= '9') {
            // 数字字符：累加到当前整数
            int_out[num_idx] = int_out[num_idx] * 10 + (input[i] - '0');
        } else if (input[i] == ',') {
            // 逗号：切换到下一个整数
            num_idx++;
            if (num_idx >= 4) return false;  // 超过4个数字
        } else if (input[i] != ' ' && input[i] != '\r') {
            // 非法字符
            return false;
        }
    }

    // 验证是否成功解析4个整数
    return num_idx == 3;  // 3个逗号分隔4个数字
}
