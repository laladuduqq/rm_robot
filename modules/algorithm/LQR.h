#ifndef __LQR_H
#define __LQR_H
#include "stdint.h"

typedef enum {
    COMPENSATION_NONE,
    COMPENSATION_GRAVITY,
    COMPENSATION_FRICTION
} CompensationType;

typedef struct {
    float K[2]; // LQR增益矩阵 K
    uint8_t state_dim; // 状态维度
    float output_max;           // 输出最大值
    float output_min;           // 输出最小值
    CompensationType compensation_type; // 补偿类型
    float gravity_force; // 重力补偿力 (N)
    float arm_length; // 杆长 (m)
    float friction; // 摩擦力 (N)
    float DEADband;
    float integral_max;
    float ki;
} LQR_Init_Config_s;

typedef struct {
    float K[2]; // LQR增益矩阵 K
    uint8_t state_dim; // 状态维度
    float err;
    float output;
    float state[2];
    float feedbackreverseflag;
    float output_max;           // 输出最大值
    float output_min;           // 输出最小值
    CompensationType compensation_type; // 补偿类型
    float gravity_force; // 重力补偿力 (N)
    float arm_length; // 杆长 (m)
    float friction; // 摩擦力 (N)
    float DEADband;
    float ki;           // 积分增益
    float integral;           
    float integral_max; // 积分限幅
    uint32_t DWT_CNT;
    float dt;
} LQRInstance;

/**
 * @brief 初始化LQR实例
 * @param lqr    LQR实例指针
 * @param config LQR初始化配置
 */
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);

/**
 * @brief 计算LQR输出
 * @param lqr     LQR实例指针
 * @param state   当前状态向量
 * @return float  LQR计算输出
 */
float LQRCalculate(LQRInstance *lqr, float state0 ,float state1,float ref);

#endif
