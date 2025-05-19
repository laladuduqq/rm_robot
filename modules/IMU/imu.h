#ifndef __IMU_H
#define __IMU_H

#include "arm_math.h"
#include <stdint.h>


typedef struct
{
    float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    float gyroWorld[3]; //世界坐标系下角速度

    uint8_t init;

    float dt;
    float t;
    uint32_t dwt_cnt;
} INS_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;


typedef struct {
    const float *Yaw;
    const float *Pitch; 
    const float *Roll;
    const float *YawTotalAngle;
    const float (*gyro)[3];    // 指向float[3]数组的指针
} IMU_DATA_T;

extern INS_t INS;

void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void bodyToWord(float32_t gyroBody[3], float32_t roll, float32_t pitch, float32_t yaw, float32_t gyroWorld[3]);

void INS_TASK_init(void);   
IMU_DATA_T INS_GetData(void);

#endif
