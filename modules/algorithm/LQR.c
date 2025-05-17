
#include "LQR.h"
#include "dwt.h"
#include "user_lib.h"

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#include <arm_math.h>

#define LOG_TAG              "lqr"
#include <elog.h>

float degreesToRadians(float degrees) {
    return degrees * (PI / 180.0f);
}

/**
 * @brief 初始化LQR实例
 * @param lqr    LQR实例指针
 * @param config LQR初始化配置
 */
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config) {
    lqr->state_dim = config->state_dim;
    for (uint8_t i = 0; i < lqr->state_dim; i++) {
        lqr->K[i] = config->K[i];
    }
    lqr->output = 0.0f; // 初始化输出
    lqr->output_max = config->output_max;           // 输出最大值
    lqr->output_min = config->output_min;           // 输出最小值
    lqr->feedbackreverseflag =0;
    lqr->compensation_type = config->compensation_type; // 初始化补偿类型
    lqr->gravity_force = config->gravity_force; // 初始化重力补偿力
    lqr->arm_length = config->arm_length; // 初始化杆长
    lqr->friction = config->friction; // 初始化摩擦力
    lqr->DEADband = config->DEADband * (PI / 180.0f);
    lqr->integral_max = config->integral_max;
    lqr->ki = config->ki;
}

float calculateOutput(LQRInstance *lqr, float state0 ,float state1,float ref) {
    if (lqr == NULL) {
        return 0.0f; // Handle null pointer
    }

    lqr->state[0]=state0;
    lqr->state[1]=state1;

    float rad_ref = degreesToRadians(ref);
    lqr->dt = DWT_GetDeltaT(&lqr->DWT_CNT);

    if (lqr->state_dim ==1)
    {
        float rad_state_1 = degreesToRadians(state1);
        if (lqr->feedbackreverseflag==1) {rad_state_1 = -rad_state_1;}

        lqr->err = (rad_state_1 - rad_ref);


        lqr->output = -(lqr->K[0] * lqr->err);
    }
    else if (lqr->state_dim==2) 
    {
        float rad_state_0 = degreesToRadians(state0);
        float rad_state_1 = state1;
        if (lqr->feedbackreverseflag==1) {
            rad_state_0 = -rad_state_0;
            rad_state_1 = -rad_state_1;
        }

        lqr->err = (rad_state_0 - rad_ref);

        lqr->output = -(lqr->K[0] * lqr->err)-lqr->K[1] * rad_state_1;
    }

    // 计算补偿项
    float compensation = 0.0f;
    switch (lqr->compensation_type) {
        case COMPENSATION_GRAVITY:
            if (rad_ref>=0) {
                compensation = lqr->gravity_force * lqr->arm_length* cosf(rad_ref);//第一个角度为杆长夹角，第二个角度为连杆与竖直方向的夹角
            }else {
                compensation = lqr->gravity_force * (lqr->arm_length + lqr->arm_length *sinf(-rad_ref)) * cosf(-rad_ref);
            }
            break;
        case COMPENSATION_FRICTION:
            compensation = lqr->friction;
            break;
        case COMPENSATION_NONE:
        default:
            compensation = 0.0f;
            break;
    }

    // 加入补偿项
    lqr->output += compensation;

    // 积分累积（带抗饱和处理）
    lqr->integral += lqr->ki * lqr->err * lqr->dt; 
    VAL_LIMIT(lqr->integral, -lqr->integral_max, lqr->integral_max);
    lqr->output -= lqr->integral;

    // 限幅输出
    if (lqr->output > lqr->output_max) {
        lqr->output = lqr->output_max;
    } else if (lqr->output < lqr->output_min) {
        lqr->output = lqr->output_min;
    }

    return lqr->output;
}

/**
 * @brief 计算LQR输出
 * @param lqr     LQR实例指针
 * @param state   当前状态向量
 * @return float  LQR计算输出
 */
float LQRCalculate(LQRInstance *lqr, float state0 ,float state1,float ref) {
    if (lqr == NULL || lqr->state_dim < 1 || lqr->state_dim > 2) {
        return 0.0f; // Handle invalid state_dim or null pointer
    }

    return calculateOutput(lqr,state0,state1,ref);
}
