/*
 * position.c
 *
 *  Created on: Jun 20, 2025
 *      Author: liu
 */

#include "icm20948.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include<oled.h>
#include"position.h"
#include "usart.h"
// 定义全局变量
CalibSystem calib_sys = {
    .mag.Q = {{1e-4, 0, 0, 0, 0, 0},
              {0, 1e-4, 0, 0, 0, 0},
              {0, 0, 1e-4, 0, 0, 0},
              {0, 0, 0, 1e-6, 0, 0},
              {0, 0, 0, 0, 1e-6, 0},
              {0, 0, 0, 0, 0, 1e-6}},
    .mag.R = {0.1, 0.1, 0.1}
};

AccelFilter accel_filter = {
    .filters = {
        {.q=0.01, .r=0.5, .x=0, .p=1, .k=0},
        {.q=0.01, .r=0.5, .x=0, .p=1, .k=0},
        {.q=0.01, .r=0.5, .x=0, .p=1, .k=0}
    }
};

Attitude current_attitude = {0};
Displacement current_displacement = {0};
SensorData prev_velocity = {0};
VehiclePosition current_position = {0};
// 零速检测相关
static bool is_stationary = false;                 // 静止状态标志
static uint32_t stationary_counter = 0;            // 静止持续时间计数器
static const uint32_t stationary_time = 20;         // 确认静止所需周期数(50ms@10Hz)
static const float static_accel_thresh = 0.1f;    // 静止加速度阈值(m/s²)
static const float static_speed_thresh = 0.01f;    // 静止速度阈值(m/s)

// 位移计算相关
static SensorData accel_bias = {0};                // 加速度计偏置估计
//static const float max_accel = 2.0f;               // 最大允许加速度
//static const float max_speed = 1.5f;               // 最大允许速度
//static const float max_displacement = 50.0f;
// 其他函数实现...
static float constrain_acceleration(float current_vel, float accel, float max_jerk, float dt) {
    float delta_v = accel * dt;
    if(delta_v > max_jerk * dt) delta_v = max_jerk * dt;
    if(delta_v < -max_jerk * dt) delta_v = -max_jerk * dt;
    return current_vel + delta_v;
}
// 卡尔曼滤波更新
float kalman_update(KalmanFilter* kf, float measurement) {
	 // 动态调整测量噪声（根据当前状态）
	    static const float r_base = 0.1;
	    kf->r = r_base * (1 + 0.05f * fabsf(kf->x));  // 状态相关噪声

	    kf->p += kf->q;
	    kf->k = kf->p / (kf->p + kf->r);

	    // 零漂检测与补偿逻辑
	    const float drift_thresh = 0.01f;
	    if(fabsf(measurement - kf->x) < drift_thresh) {
	        kf->drift_estimate = 0.98f * kf->drift_estimate + 0.02f * (measurement - kf->x);
	        kf->x += kf->k * (measurement - kf->x - kf->drift_estimate);
	    } else {
	        kf->x += kf->k * (measurement - kf->x);
	    }

	    kf->p = (1 - kf->k) * kf->p;
	    return kf->x;
}

// 加速度计静态校准
void calibrate_accelerometer() {
    const int samples = 200;
    SensorData raw = {0}, sum = {0};

	  OLED_NewFrame();

   OLED_PrintString(13, 0 ,"wait for init", &font16x16, OLED_COLOR_NORMAL);
	  OLED_ShowFrame();
    for(int i = 0; i < samples; i++) {
        read_raw_accel(&raw);
        sum.x += raw.x;
        sum.y += raw.y;
        sum.z += raw.z;
        delay_ms(10);
    }

    calib_sys.accel.offset_x = -sum.x / samples;
    calib_sys.accel.offset_y = -sum.y / samples;
    calib_sys.accel.offset_z = GRAVITY - sum.z / samples;
    calib_sys.accel.is_calibrated = true;
	  OLED_NewFrame();

 OLED_PrintString(13, 15 ,"success", &font16x16, OLED_COLOR_NORMAL);
	  OLED_ShowFrame();
}

// 磁力计动态校准初始化
void mag_dynamic_calib_init() {
    memset(calib_sys.mag.x, 0, sizeof(calib_sys.mag.x));
    memset(calib_sys.mag.P, 0, sizeof(calib_sys.mag.P));

    for(int i = 0; i < 6; i++) {
        calib_sys.mag.P[i][i] = 1.0f;
        calib_sys.mag.P[i][i] *= (i < 3) ? 1e-4 : 1e-6;
    }
    calib_sys.mag.is_initialized = true;
}

// 磁力计动态校准更新
void mag_dynamic_calib_update(SensorData *mag) {
    static const float process_noise = 1e-5;
    static const float measure_noise = 0.1;

    for(int i = 0; i < 6; i++) {
        calib_sys.mag.P[i][i] += process_noise;
    }

    float residual_x = mag->x - (calib_sys.mag.x[0] + mag->x * calib_sys.mag.x[3]);
    float residual_y = mag->y - (calib_sys.mag.x[1] + mag->y * calib_sys.mag.x[4]);

    float S = calib_sys.mag.P[0][0] + measure_noise;
    float K = calib_sys.mag.P[0][0] / S;

    calib_sys.mag.x[0] += K * residual_x;
    calib_sys.mag.x[1] += K * residual_y;
    calib_sys.mag.P[0][0] = (1 - K) * calib_sys.mag.P[0][0];
}
// 新增2：零速检测函数
void zero_velocity_update(SensorData *accel, SensorData *velocity) {
    // 计算加速度模长和速度模长
    float accel_mag = sqrtf(accel->x*accel->x + accel->y*accel->y );
    float speed_mag = sqrtf(velocity->x*velocity->x + velocity->y*velocity->y);

    // 双条件静止检测
    if(accel_mag < static_accel_thresh && speed_mag < static_speed_thresh) {
        if(++stationary_counter > stationary_time) {
            is_stationary = true;
        }
    } else {
        is_stationary = false;
        stationary_counter = 0;
    }

    // 执行零速修正
    if(is_stationary) {
        current_displacement.x = 0;
        current_displacement.y = 0;
        prev_velocity.x = 0;
        prev_velocity.y = 0;
        accel_bias = (SensorData){0};  // 重置偏置估计
    }
}
// 传感器滤波
void filter_accelerometer(SensorData *raw, SensorData *filtered) {
    filtered->x = kalman_update(&accel_filter.filters[0], raw->x);
    filtered->y = kalman_update(&accel_filter.filters[1], raw->y);
    filtered->z = kalman_update(&accel_filter.filters[2], raw->z);
}

// 姿态解算
void calculate_attitude(SensorData *accel, SensorData *mag) {
//    current_attitude.pitch = atan2f(accel->y,
//                                   sqrtf(accel->x*accel->x + accel->z*accel->z));
//    current_attitude.roll = atan2f(accel->x,
//                                  sqrtf(accel->y*accel->y + accel->z*accel->z));
	// ...existing code...9.7944
	   float ax = accel->x /9.7944f;
	    float ay = accel->y / 9.7944f;
	    float az = accel->z / 9.7944f;
	    current_attitude.pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
	    // roll: 左右横滚，绕Y轴
	    current_attitude.roll  = atan2f(ay, az);


    float sin_pitch = sinf(current_attitude.pitch);
    float cos_pitch = cosf(current_attitude.pitch);
    float sin_roll = sinf(current_attitude.roll);
    float cos_roll = cosf(current_attitude.roll);

    SensorData compensated_mag = {
        mag->x * cos_pitch + mag->z * sin_pitch,
        mag->x * sin_roll * sin_pitch + mag->y * cos_roll -
        mag->z * sin_roll * cos_pitch
    };

    compensated_mag.x -= calib_sys.mag.x[0];
    compensated_mag.y -= calib_sys.mag.x[1];

    current_attitude.yaw = atan2f(compensated_mag.y, compensated_mag.x);
}

// 位移计算
void calculate_displacement(SensorData *accel, float dt) {
	// 1. 单位转换（g → m/s²）
	    const float G_TO_MS2 = 9.81f;
	    accel->x *= G_TO_MS2;
	    accel->y *= G_TO_MS2;
	    accel->z *= G_TO_MS2;

	    // 2. 重力补偿（关键修改：添加完整重力分量计算）
	    const float sp = sinf(current_attitude.pitch);
	    const float cp = cosf(current_attitude.pitch);
	    const float sr = sinf(current_attitude.roll);
	    const float cr = cosf(current_attitude.roll);

	    // 计算重力在载体坐标系中的分量
	    SensorData gravity = {
	        .x = GRAVITY * sr,          // 横向重力分量（X轴）
	        .y = -GRAVITY * sp,         // 纵向重力分量（Y轴）
	        .z = GRAVITY * cp * cr      // 垂直重力分量（Z轴）
	    };

	    // 从总加速度中去除重力分量
	    SensorData linear_accel = {
	        accel->x - gravity.x,       // 线性加速度X分量
	        accel->y - gravity.y,       // 线性加速度Y分量
	        accel->z - gravity.z        // 线性加速度Z分量
	    };

	    // 3. 高通滤波（去除传感器零偏）
	    static const float alpha = 0.95f;
	    accel_bias.x = alpha * accel_bias.x + (1 - alpha) * linear_accel.x;
	    accel_bias.y = alpha * accel_bias.y + (1 - alpha) * linear_accel.y;
	    linear_accel.x -= accel_bias.x;
	    linear_accel.y -= accel_bias.y;

	    // 4. 运动约束（防止积分发散）
	    linear_accel.x = fmaxf(fminf(linear_accel.x, MAX_ACCEL), -MAX_ACCEL);
	    linear_accel.y = fmaxf(fminf(linear_accel.y, MAX_ACCEL), -MAX_ACCEL);

	    // 5. 二阶积分计算（速度+位移）
	    static const float jerk_limit = 50.0f;  // 急动度限制
	    prev_velocity.x = constrain_acceleration(prev_velocity.x, linear_accel.x, jerk_limit, dt);
	    prev_velocity.y = constrain_acceleration(prev_velocity.y, linear_accel.y, jerk_limit, dt);

	    // 更新全局坐标（核心修改）
	    current_position.x += prev_velocity.x * dt + 0.5f * linear_accel.x * dt * dt;
	    current_position.y += prev_velocity.y * dt + 0.5f * linear_accel.y * dt * dt;

	    // 速度更新
	    prev_velocity.x += linear_accel.x * dt;
	    prev_velocity.y += linear_accel.y * dt;

	    // 6. 约束保护
	    prev_velocity.x = fmaxf(fminf(prev_velocity.x, MAX_SPEED), -MAX_SPEED);
	    prev_velocity.y = fmaxf(fminf(prev_velocity.y, MAX_SPEED), -MAX_SPEED);

	    current_position.x = fmaxf(fminf(current_position.x, MAX_DISPLACEMENT), -MAX_DISPLACEMENT);
	    current_position.y = fmaxf(fminf(current_position.y, MAX_DISPLACEMENT), -MAX_DISPLACEMENT);
	}
//c
// 系统初始化
void system_init() {
    icm20948_init();
    HAL_Delay(200);
    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
    ak09916_init();
    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    calibrate_accelerometer();
    mag_dynamic_calib_init();
    current_position = (VehiclePosition){0};
}

void Send_data( float f1, float f2, float f3)
	 {
	     /* 使用静态缓冲区避免栈溢出风险 */
	     static char buffer[128];

	     /* 安全格式化（限制缓冲区访问） */
	     int len = snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f\r\n", (float)f1, (float)f2, (float)f3);





	       // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, len);
	        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len,9000);





	 }


void core_computation(SensorData *raw_gyro,SensorData *raw_accel,SensorData *raw_mag,SensorData *filtered_accel) {
static float dt = LOOP_TIME_MS / 1000.0f;


    // 1. 应用加速度计校准
    if(calib_sys.accel.is_calibrated) {
        raw_accel->x += calib_sys.accel.offset_x;
        raw_accel->y += calib_sys.accel.offset_y;
        raw_accel->z += calib_sys.accel.offset_z;
    }

    // 2. 滤波处理
    filter_accelerometer(raw_accel, filtered_accel);

    // 3. 磁力计动态校准更新
    if(calib_sys.mag.is_initialized) {
        mag_dynamic_calib_update(raw_mag);
    }

    // 4. 姿态解算
  //  calculate_attitude(filtered_accel, raw_mag);
    calculate_attitude( raw_accel, raw_mag);
    // 5. 零速检测（传入速度数据）
    zero_velocity_update(filtered_accel, &prev_velocity);
    // 6. 位移计算
    calculate_displacement(filtered_accel, dt);
  // Send_data(current_displacement.x,current_displacement.y, filtered_accel->z);
   static char buffer22[32];
   static char buffer23[32];
   static char buffer24[32];
  	     /* 安全格式化（限制缓冲区访问） */
  	     snprintf(buffer22, sizeof(buffer22), "%.2f,%.2f",  current_attitude.pitch*180/3.1416f, current_attitude.yaw*180/3.1416f);
  	   snprintf(buffer23, sizeof(buffer23), "%.2f,%.2f", filtered_accel->x,filtered_accel->y);
  	 snprintf(buffer24, sizeof(buffer24), "%.2f,%.2f", prev_velocity.x,prev_velocity.y);
    OLED_NewFrame();
   	//  sprintf(message,"speed: %.3f",speed);
     //  sprintf(message,"dps:%.2f,%.2f,%.2f",my_gyro.x,my_gyro.y,my_gyro.z);
   //	    sprintf(message2,"g: %.2f,%.2f,%.2f",my_accel.x,my_accel.y,my_accel.z);
   //	    sprintf(message3,"ut:%.2f,%.2f,%.2f",my_mag.x,my_mag.y,my_mag.z);
   //OLED_PrintString(13, 0, message, &font16x16, OLED_COLOR_NORMAL);
   	 OLED_PrintString(13, 15 , buffer24, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(13, 30 , buffer22, &font16x16, OLED_COLOR_NORMAL);
   //
       OLED_PrintString(13, 0 ,buffer23, &font16x16, OLED_COLOR_NORMAL);
   	  OLED_ShowFrame();
}

// 主控制循环
//void main_loop() {
//    const float dt = LOOP_TIME_MS / 1000.0f;
//    SensorData raw_gyro, raw_accel, raw_mag;
//    SensorData filtered_accel;
//
//    while(1) {
//        read_raw_gyro(&raw_gyro);
//        read_raw_accel(&raw_accel);
//        read_raw_mag(&raw_mag);
//
//        if(calib_sys.accel.is_calibrated) {
//            raw_accel.x += calib_sys.accel.offset_x;
//            raw_accel.y += calib_sys.accel.offset_y;
//            raw_accel.z += calib_sys.accel.offset_z;
//        }
//
//        filter_accelerometer(&raw_accel, &filtered_accel);
//
//        if(calib_sys.mag.is_initialized) {
//            mag_dynamic_calib_update(&raw_mag);
//        }
//
//        calculate_attitude(&filtered_accel, &raw_mag);
//        calculate_displacement(&filtered_accel, dt);
//
//        Send_Floats(current_displacement.x, current_displacement.y, 0,
//                   current_attitude.roll, current_attitude.pitch, current_attitude.yaw,
//                   raw_accel.x, raw_accel.y, raw_accel.z);
//
//        delay_ms(LOOP_TIME_MS);
//    }
//}
