#ifndef __BMI088_H
#define __BMI088_H

#include <stdint.h>
#include "controller.h"

#define GxOFFSET -0.00601393497f
#define GyOFFSET -0.00196841615f
#define GzOFFSET 0.00114696583f
#define gNORM 9.67463112f

/* BMI088数据*/
typedef struct
{
    float gyro[3];     // 陀螺仪数据,xyz
    float acc[3];      // 加速度计数据,xyz
    float temperature; // 温度
    float TempWhenCali; //标定时温度
    uint32_t BMI088_ERORR_CODE;
    PIDInstance imu_temp_pid;
    // 标定数据
    float AccelScale;
    float GyroOffset[3];
    float gNorm;          // 重力加速度模长,从标定获取
    uint8_t cali_mode;  //标定
} BMI088_Data_t;

typedef struct
{
    const float (*gyro)[3];    // 陀螺仪数据,xyz
    const float (*acc)[3];     // 加速度计数据,xyz
} BMI088_GET_Data_t;

void bmi088_temp_ctrl(void);
void BMI088_init(void);
BMI088_GET_Data_t BMI088_GET_DATA(void);

#endif
