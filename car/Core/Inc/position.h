/*
 * position.h
 *
 *  Created on: Jun 20, 2025
 *      Author: liu
 */

#ifndef INC_POSITION_H_
#define INC_POSITION_H_

// ======================
// === 1. 硬件抽象层（已适配ICM20948驱动）===
// ======================
// 使用与驱动兼容的数据结构
typedef axises SensorData;  // 类型别名保持兼容性

// 直接使用驱动中的传感器读取接口
#define read_raw_gyro(data)   icm20948_gyro_read_dps(data)
#define read_raw_accel(data)  icm20948_accel_read_g(data)
#define read_raw_mag(data)    ak09916_mag_read_uT(data)
#define delay_ms(ms)          HAL_Delay(ms)

// ======================
// === 2. 校准模块 ===
// ======================
typedef struct {
    float offset_x, offset_y, offset_z;
    bool is_calibrated;
} AccelCalib;

typedef struct {
    float x[6];
    float P[6][6];
    const float Q[6][6];
    const float R[3];
    bool is_initialized;
} MagDynCalib;

typedef struct {
    AccelCalib accel;
    MagDynCalib mag;
} CalibSystem;

extern CalibSystem calib_sys;  // 声明为 extern
// 零速检测参数
#define STATIC_ACCEL_THRESH 0.05f   // 加速度阈值(m/s²)
#define STATIC_SPEED_THRESH 0.015f   // 速度阈值(m/s)
#define STATIC_CONFIRM_TIME 5       // 确认静止所需周期数

// 运动约束参数
#define MAX_ACCEL 10.0f              // 最大允许加速度(m/s²)
#define MAX_SPEED 1.5f              // 最大允许速度(m/s)
#define MAX_DISPLACEMENT 10.0f      // 最大允许位移(m)
// ======================
// === 3. 传感器滤波模块 ===
// ======================
typedef struct {
    float q, r, x, p, k;
    float drift_estimate;  // 新增：零漂估计值
    float last_measurement;
} KalmanFilter;
typedef struct {
    KalmanFilter filters[3];
} AccelFilter;

extern AccelFilter accel_filter;  // 声明为 extern

// ======================
// === 4. 姿态解算模块 ===
// ======================
typedef struct {
    float pitch, roll, yaw;
} Attitude;

extern Attitude current_attitude;  // 声明为 extern

// ======================
// === 5. 位移计算模块 ===
// ======================
typedef struct {
    float x, y;
} Displacement;

extern Displacement current_displacement;  // 声明为 extern
extern SensorData prev_velocity;  // 声明为 extern

typedef struct {
    float x;
    float y;
} VehiclePosition;

extern VehiclePosition current_position;
// ======================
// === 6. 主控制模块 ===
// ======================
#define LOOP_TIME_MS 10
#define GRAVITY 9.7944f//杭州地区9.7936f

float kalman_update(KalmanFilter* kf, float measurement);
void calibrate_accelerometer();
void mag_dynamic_calib_init();
void mag_dynamic_calib_update(SensorData *mag);
void filter_accelerometer(SensorData *raw, SensorData *filtered);
void calculate_attitude(SensorData *accel, SensorData *mag);
void calculate_displacement(SensorData *accel, float dt);
void system_init();
void core_computation(SensorData *raw_gyro,SensorData *raw_accel,SensorData *raw_mag,SensorData *filtered_accel) ;
void Send_data( float f1, float f2, float f3);
void zero_velocity_update(SensorData *accel, SensorData *velocity);
#endif /* INC_POSITION_H_ */
