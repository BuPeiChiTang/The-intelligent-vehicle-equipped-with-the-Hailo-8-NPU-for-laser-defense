#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "icm20948.h"

/* 姿态结构体定义 */
typedef struct {
    float roll;     // 横滚角 (-180°~180°)
    float pitch;    // 俯仰角 (-90°~90°)
    float yaw;      // 偏航角 (0°~360°)
} Attitude_t;

/* 姿态解算器结构体（扩展版本） */
typedef struct {
    Attitude_t angles;           // 欧拉角输出
    float q_w, q_x, q_y, q_z;    // 四元数
    float gyro_bias_z;           // Z轴陀螺仪偏置
    float accel_bias_x;          // X轴加速度计偏置
    float accel_bias_y;          // Y轴加速度计偏置
    float accel_bias_z;          // Z轴加速度计偏置

    // 新增零速检测变量
    float last_gyro_z;           // 上一时刻Z轴角速度
    float angular_accel_z;       // Z轴角加速度
    bool z_axis_stationary;      // Z轴静止标志
    uint32_t stationary_count;   // 静止计数
    uint32_t start_count;   // 起步计数

    float kalman_p[3];           // 卡尔曼滤波器参数
    float kalman_r[3];
    bool initialized;            // 初始化标志
} AttitudeSolver_t;

/* 初始化姿态解算器 */
void Attitude_Init(AttitudeSolver_t *solver);

/* 使用互补滤波更新姿态 */
void Attitude_Update_Complementary(AttitudeSolver_t *solver,
                                 const axises *gyro,
                                 const axises *accel,
                                 float dt);

/* 使用四元数和互补滤波更新姿态 */
void Attitude_Update_Quaternion(AttitudeSolver_t *solver,
                               const axises *gyro,
                               const axises *accel,
                               float dt);

/* 获取当前姿态 */
void Attitude_Get_Angles(AttitudeSolver_t *solver, Attitude_t *angles);

/* 执行陀螺仪Z轴校准 */
void Attitude_Calibrate_Gyro_Z(AttitudeSolver_t *solver, uint16_t samples);

/* 执行加速度计静态校准 */
void Attitude_Calibrate_Accel(AttitudeSolver_t *solver, uint16_t samples);

#endif /* __ATTITUDE_H__ */
