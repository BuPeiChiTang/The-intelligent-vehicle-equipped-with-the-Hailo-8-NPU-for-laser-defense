/*
 * recieve data.c
 *
 *  Created on: Jun 26, 2025
 *      Author: liu
 */


#include <string.h>
#include <stdlib.h>


int Unpack_Floats(const char* raw_str, float* output_array) {
    // 创建可修改的副本（避免修改原始字符串）
    char buffer[128];
    strncpy(buffer, raw_str, sizeof(buffer)-1);
    buffer[sizeof(buffer)-1] = '\0'; // 确保终止符

    // 验证帧结尾
    if (strlen(buffer) < 2 ||
        buffer[strlen(buffer)-2] != '\r' ||
        buffer[strlen(buffer)-1] != '\n') {
        return 0;
    }

    // 替换帧结束符
    buffer[strlen(buffer)-2] = '\0';

    // 解析浮点数
    char* token = strtok(buffer, ",");
    int count = 0;

    while (token != NULL && count < 4) {
        // 简单安全转换
        char* endptr;
        float value = strtof(token, &endptr);

        // 验证转换有效性
        if (*endptr != '\0') {
            return 0; // 包含非数字字符
        }

        output_array[count] = value;
        token = strtok(NULL, ",");
        count++;
    }

    return count; // 返回实际解析到的数值数量
}

