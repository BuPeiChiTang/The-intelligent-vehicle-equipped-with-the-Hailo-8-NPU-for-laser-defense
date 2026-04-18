#ifndef MPU6050_H
#define MPU6050_H
#include "main.h"
/*
bilibili 小努班 UID:437280309
@time时间: 2025.5.5
@version版本:V1_4
@Encoding :UTF-8
@attention : 注意杜邦线别用劣质的，还有如果零飘严重可以进入函数MPU6050_Get_Angle()里进行修改，里面会告诉你怎么做
1.增加了使用四元素得到欧拉角的函数
2.引入了软件矫正零点漂移函数，用于修正z轴零点漂移(MPU6050_Get_Angle)
3.积分法

if you can't display Chinese correctly,please check your encoding mode(please set encoding mode to UTF-8)
*/

//陀螺仪原生数据结构体
typedef struct MPU6050_raw
{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    uint16_t Temp;
}MPU6050_raw;
//陀螺仪角度结构体
typedef struct MPU6050
{
    float yaw;
    float roll;
    float pitch;
}MPU6050;
typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 状态估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} MPU6050_KalmanFilter;
extern MPU6050_KalmanFilter kf_roll, kf_pitch, kf_xy;
extern float Gyroscope_roll, Gyroscope_pitch;
extern float q_w, q_x, q_y, q_z;
extern float _integralFBx, _integralFBy, _integralFBz;
/////////////////////////常用函数//////////////////////////
void MPU6050_init(I2C_HandleTypeDef *hi2c);   												//初始化姿态传感器,使用硬件SPI
void MPU6050_Get_Angle_Plus(MPU6050* this) ;                        //得到角度(四元素法+互补滤波)
void MPU6050_Get_Angle(MPU6050* this);                              //得到角度(正常方法+卡尔曼滤波+互补滤波)
float MPU6050_GetTemp(void);                                        //得到温度
void MPU6050_FastResetYaw_Complementary(MPU6050* sensor);
void MPU6050_FastResetYaw_Quaternion(MPU6050* sensor) ;
/////////////////////////扩展函数//////////////////////////
uint8_t MPU6050_ID(void);                                           //读取ID
void MPU6050_Get_Raw(MPU6050_raw* this);                            //得到原生数据

#endif
