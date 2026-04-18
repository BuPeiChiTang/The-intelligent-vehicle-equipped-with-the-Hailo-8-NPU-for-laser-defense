/*
 * icm20948_ahrs.c
 *
 *  Created on: Jun 22, 2025
 *      Author: liu
 */
#include "icm20948_ahrs.h"
#include "math.h"

/* 卡尔曼滤波器更新函数 */
static float KalmanFilter_Update(float *x, float *p, float q, float r, float measurement) {
    // 预测步骤
    *p = *p + q;
    // 计算卡尔曼增益
    float k = *p / (*p + r);
    // 更新估计值
    *x = *x + k * (measurement - *x);
    // 更新估计误差协方差
    *p = (1 - k) * *p;
    return *x;
}

/* 初始化姿态解算器 */
void Attitude_Init(AttitudeSolver_t *solver) {
    // 初始化欧拉角
    solver->angles.roll = 0.0f;
    solver->angles.pitch = 0.0f;
    solver->angles.yaw = 0.0f;

    // 初始化四元数
    solver->q_w = 1.0f;
    solver->q_x = 0.0f;
    solver->q_y = 0.0f;
    solver->q_z = 0.0f;

    // 初始化传感器偏置
    solver->gyro_bias_z = 0.0f;
    solver->accel_bias_x = 0.0f;
    solver->accel_bias_y = 0.0f;
    solver->accel_bias_z = 0.0f;

    // 初始化零速检测变量
    solver->last_gyro_z = 0.0f;
    solver->angular_accel_z = 0.0f;
    solver->z_axis_stationary = false;
    solver->stationary_count = 0;

    // 初始化卡尔曼滤波器参数
    solver->kalman_p[0] = 1.0f;  // Roll
    solver->kalman_p[1] = 1.0f;  // Pitch
    solver->kalman_p[2] = 1.0f;  // Yaw
    solver->kalman_r[0] = 0.1f;  // Roll
    solver->kalman_r[1] = 0.1f;  // Pitch
    solver->kalman_r[2] = 0.2f;  // Yaw

    solver->initialized = false;
}

/* 执行陀螺仪Z轴校准 */
void Attitude_Calibrate_Gyro_Z(AttitudeSolver_t *solver, uint16_t samples) {
    float sum = 0.0f;
    axises gyro;

    for (uint16_t i = 0; i < samples; i++) {
        icm20948_gyro_read_dps(&gyro);
        sum += gyro.z;
        HAL_Delay(10);  // 假设采样率为100Hz
    }

    solver->gyro_bias_z = sum / samples;
}

/* 执行加速度计静态校准 */
void Attitude_Calibrate_Accel(AttitudeSolver_t *solver, uint16_t samples) {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    axises accel;

    for (uint16_t i = 0; i < samples; i++) {
        icm20948_accel_read_g(&accel);
        sum_x += accel.x;
        sum_y += accel.y;
        sum_z += accel.z;
        HAL_Delay(10);
    }

    // 计算加速度计偏置（假设静止时Z轴应为1g）
    solver->accel_bias_x = sum_x / samples;
    solver->accel_bias_y = sum_y / samples;
    solver->accel_bias_z = sum_z / samples - 1.0f;  // Z轴减去重力加速度
}

/* 使用互补滤波更新姿态（含零速检测） */
void Attitude_Update_Complementary(AttitudeSolver_t *solver,
                                 const axises *gyro,
                                 const axises *accel,
                                 float dt) {
    if (!solver->initialized) {
        solver->initialized = true;
        return;
    }

    // 应用加速度计偏置校准
    float ax = accel->x - solver->accel_bias_x;
    float ay = accel->y - solver->accel_bias_y;
    float az = accel->z - solver->accel_bias_z;

    float gx = gyro->x;
    float gy = gyro->y;
    float gz = gyro->z - solver->gyro_bias_z;  // 应用陀螺仪偏置校准

    // 转换为弧度
    gx *= 0.0174533f * dt;  // 度/秒转弧度
    gy *= 0.0174533f * dt;
    gz *= 0.0174533f * dt;

    // 计算加速度的绝对值
    float absAcc = sqrtf(ax * ax + ay * ay + az * az);
    float weight = (absAcc > 1.2f) ? 0.8f : 1.0f;  // 动态调整权重

    // 计算原始角度（加速度计）
    float raw_roll = weight * atan2f(ay, az) * 57.2958f;
    float raw_pitch = -weight * atan2f(ax, az) * 57.2958f;

    // 积分陀螺仪角度
    static float gyro_roll = 0.0f;
    static float gyro_pitch = 0.0f;
    gyro_roll += gy;
    gyro_pitch += gx;

    // 互补滤波融合
    float filtered_roll = KalmanFilter_Update(&solver->kalman_p[0], &solver->kalman_r[0],
                                           0.001f, 0.1f, raw_roll);
    float filtered_pitch = KalmanFilter_Update(&solver->kalman_p[1], &solver->kalman_r[1],
                                            0.001f, 0.1f, raw_pitch);

    // 更新姿态角
    solver->angles.roll = filtered_roll;
    solver->angles.pitch = filtered_pitch;

    // 计算Z轴角加速度（零速检测）
    solver->angular_accel_z = (gz - solver->last_gyro_z) / dt;
    solver->last_gyro_z = gz;  // 保存当前角速度用于下次计算

    // Z轴静止检测（角速度和角加速度阈值判断）
    const float GYRO_THRESH = 0.1f;         // 角速度阈值(rad)
    const float ACCEL_THRESH = 0.13f;       // 角加速度阈值(rad/s²)
    const uint32_t DEBOUNCE_COUNT = 5;      // 防抖计数

    if (fabsf(gz) < GYRO_THRESH && fabsf(solver->angular_accel_z) < ACCEL_THRESH) {
        solver->stationary_count++;
        if (solver->stationary_count >= DEBOUNCE_COUNT) {
            solver->z_axis_stationary = true;
        }
    } else {
        solver->stationary_count = 0;
        solver->z_axis_stationary = false;
    }

    // 自动抑制Yaw更新（零速时）
    if (!solver->z_axis_stationary) {
        solver->angles.yaw += gz * 57.2958f;  // 正常更新Yaw
    }
    // 零速时Yaw不更新，自动抑制零漂

    // 确保Yaw在0-360度范围内
    while (solver->angles.yaw < 0) solver->angles.yaw += 360.0f;
    while (solver->angles.yaw >= 360.0f) solver->angles.yaw -= 360.0f;
}

/* 使用四元数和互补滤波更新姿态（含零速检测） */
void Attitude_Update_Quaternion(AttitudeSolver_t *solver,
                               const axises *gyro,
                               const axises *accel,
                               float dt) {
    if (!solver->initialized) {
        solver->initialized = true;
        return;
    }

    // 应用加速度计偏置校准
    float ax = accel->x - solver->accel_bias_x;
    float ay = accel->y - solver->accel_bias_y;
    float az = accel->z - solver->accel_bias_z;

    float gx = gyro->x - solver->gyro_bias_z;  // 应用陀螺仪偏置校准
    float gy = gyro->y;
    float gz = gyro->z;

    // 转换为弧度/秒
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 四元数微分方程
    float qDot1 = 0.5f * (-solver->q_x * gx - solver->q_y * gy - solver->q_z * gz);
    float qDot2 = 0.5f * (solver->q_w * gx + solver->q_y * gz - solver->q_z * gy);
    float qDot3 = 0.5f * (solver->q_w * gy - solver->q_x * gz + solver->q_z * gx);
    float qDot4 = 0.5f * (solver->q_w * gz + solver->q_x * gy - solver->q_y * gx);

    // 归一化加速度计测量值
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    // 辅助变量计算四元数乘积
    float q0q0 = solver->q_w * solver->q_w;
    float q0q1 = solver->q_w * solver->q_x;
    float q0q2 = solver->q_w * solver->q_y;
    float q1q3 = solver->q_x * solver->q_z;
    float q2q3 = solver->q_y * solver->q_z;
    float q3q3 = solver->q_z * solver->q_z;

    // 参考方向的重力
    float halfvx = solver->q_x * solver->q_z - solver->q_w * solver->q_y;
    float halfvy = solver->q_w * solver->q_x + solver->q_y * solver->q_z;
    float halfvz = q0q0 - 0.5f + q3q3;

    // 误差是测量方向和参考方向之间的叉积
    float halfex = (ay * halfvz - az * halfvy);
    float halfey = (az * halfvx - ax * halfvz);
    float halfez = (ax * halfvy - ay * halfvx);

    // 应用比例增益
    gx += 20.0f * halfex;
    gy += 20.0f * halfey;
    gz += 20.0f * halfez;

    // 更新四元数导数
    qDot1 -= solver->q_x * halfex + solver->q_y * halfey + solver->q_z * halfez;
    qDot2 += solver->q_w * halfex - solver->q_z * halfey + solver->q_y * halfez;
    qDot3 += solver->q_z * halfex + solver->q_w * halfey - solver->q_x * halfez;
    qDot4 += -solver->q_y * halfex + solver->q_x * halfey + solver->q_w * halfez;

    // 积分四元数导数
    solver->q_w += qDot1 * dt;
    solver->q_x += qDot2 * dt;
    solver->q_y += qDot3 * dt;
    solver->q_z += qDot4 * dt;

    // 归一化四元数
    norm = sqrtf(solver->q_w * solver->q_w + solver->q_x * solver->q_x +
                solver->q_y * solver->q_y + solver->q_z * solver->q_z);
    if (norm > 0) {
        solver->q_w /= norm;
        solver->q_x /= norm;
        solver->q_y /= norm;
        solver->q_z /= norm;
    }

//    // 计算Z轴角加速度（零速检测）
//    solver->angular_accel_z = (gz - solver->last_gyro_z) / dt;
//    solver->last_gyro_z = gz;  // 保存当前角速度用于下次计算
//
// //    Z轴静止检测（角速度和角加速度阈值判断）
//    const float GYRO_THRESH = 0.3f;         // 角速度阈值(rad/s)
//    const float ACCEL_THRESH = 0.035f;       // 角加速度阈值(rad/s²)
//    const uint32_t DEBOUNCE_COUNT = 5;      // 防抖计数
//
//    if (fabsf(gz) < GYRO_THRESH && fabsf(solver->angular_accel_z) < ACCEL_THRESH) {
//        solver->stationary_count++;
//        if (solver->stationary_count >= DEBOUNCE_COUNT) {
//            solver->z_axis_stationary = true;
//        }
//    } else {
//        solver->stationary_count = 0;
//        solver->z_axis_stationary = false;
//    }
    // 状态检测参数
//    const float GYRO_THRESH = 0.3f;         // 角速度阈值(rad/s)
//    const float ACCEL_THRESH = 0.035f;      // 角加速度阈值(rad/s²)
//    const uint32_t DEBOUNCE_COUNT = 5;      // 防抖计数阈值
//
//    // 当前状态判断
//    bool is_currently_stationary = (fabsf(gz) < GYRO_THRESH) &&
//                                  (fabsf(solver->angular_accel_z) < ACCEL_THRESH);
//
//    // 保存上一次状态
//    static bool last_stationary_state = true;//初始化为静止
//
//    // 状态转换逻辑
//    if (is_currently_stationary) {
//        // 静止状态持续计数
//        solver->stationary_count++;
//
//        // 运动状态计数清零
//        solver->start_count = 0;
//        gz = 0.0f;
//        // 从运动到静止的转换判断
//        if (solver->stationary_count >= DEBOUNCE_COUNT && last_stationary_state == false) {
//            solver->z_axis_stationary = true;
//            gz = 0.0f;
//         //   printf("状态: 运动 → 静止\n");
//        }
//    } else {
//        // 运动状态持续计数
//        solver->start_count++;
//
//        // 静止状态计数清零
//        solver->stationary_count = 0;
//
//        // 从静止到运动的转换判断
//        if (solver->start_count >= DEBOUNCE_COUNT && last_stationary_state == true) {
//            solver->z_axis_stationary = false;
//           // printf("状态: 静止 → 运动\n");
//        }
//    }
//
//    // 更新上一次状态
//    last_stationary_state = solver->z_axis_stationary;
    // 转换四元数到欧拉角
    solver->angles.roll = atan2f(2.0f * (solver->q_w * solver->q_x + solver->q_y * solver->q_z),
                              1.0f - 2.0f * (solver->q_x * solver->q_x + solver->q_y * solver->q_y)) * 57.2958f;
    solver->angles.pitch = asinf(2.0f * (solver->q_w * solver->q_y - solver->q_z * solver->q_x)) * 57.2958f;
    solver->angles.yaw = atan2f(2.0f * (solver->q_w * solver->q_z + solver->q_x * solver->q_y),
                             1.0f - 2.0f * (solver->q_y * solver->q_y + solver->q_z * solver->q_z)) * 57.2958f;

//    // 自动抑制Yaw更新（零速时）
//    if (solver->z_axis_stationary) {
//        static float locked_yaw = 0.0f;
//        if (!solver->z_axis_stationary) {
//            locked_yaw = solver->angles.yaw;  // 开始静止时记录Yaw
//        }
//        solver->angles.yaw = locked_yaw;  // 零速时锁定Yaw
//    }

    // 确保Yaw在0-360度范围内
    while (solver->angles.yaw < 0) solver->angles.yaw += 360.0f;
    while (solver->angles.yaw >= 360.0f) solver->angles.yaw -= 360.0f;
//    if(solver->angles.yaw>=180.0f)solver->angles.yaw=360.0f-(360.0f-solver->angles.yaw)*90.0f/23.0f;
//    else{
//    	solver->angles.yaw*=(90.0f/23.0f);
//    }
        if(solver->angles.yaw>=180.0f)solver->angles.yaw=360.0f-(360.0f-solver->angles.yaw)*3.0f;
        else{
        	solver->angles.yaw*=3.0f;
        }
}

/* 获取当前姿态 */
void Attitude_Get_Angles(AttitudeSolver_t *solver, Attitude_t *angles) {
    *angles = solver->angles;
}
