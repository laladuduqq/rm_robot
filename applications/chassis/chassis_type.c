#include "robotdef.h"
#include <math.h>

#define MAX_WHEEL_RPM 8000

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

//内部函数声明
static void MecanumCalculate(float chassis_vx, float chassis_vy, float chassis_wz,float_t *wheel_ops);
static void OmniCalculate(float chassis_vx, float chassis_vy, float chassis_wz,float_t *wheel_ops);
static void HelmCalculate(float chassis_vx, float chassis_vy, float chassis_wz,float_t *wheel_ops);


void ChassisCalculate(float chassis_vx, float chassis_vy, float chassis_wz,float *wheel_ops)
{
#if CHASSIS_TYPE == 1
    MecanumCalculate(chassis_vx,chassis_vy,chassis_wz,wheel_ops);
#elif CHASSIS_TYPE == 2
    OmniCalculate(chassis_vx,chassis_vy, chassis_wz,wheel_ops);
#elif CHASSIS_TYPE == 3
    HelmCalculate(chassis_vx,chassis_vy,chassis_wz,wheel_ops);
#else
    #error "chassis type error"
#endif 

}
static void OmniCalculate(float chassis_vx, float chassis_vy, float chassis_wz, float_t *wheel_ops)
{
    float max = 0.0f;

    wheel_ops[0] = ((-0.707f * chassis_vx + 0.707f *chassis_vy + chassis_wz * WHEEL_R) / RADIUS_WHEEL);  //lf
    wheel_ops[1] = (( 0.707f * chassis_vx + 0.707f *chassis_vy + chassis_wz * WHEEL_R) / RADIUS_WHEEL); //rf
    wheel_ops[2] = ((-0.707f * chassis_vx - 0.707f *chassis_vy + chassis_wz * WHEEL_R) / RADIUS_WHEEL);   //lb
    wheel_ops[3] = (( 0.707f * chassis_vx - 0.707f *chassis_vy + chassis_wz * WHEEL_R) / RADIUS_WHEEL);   //rb

    // 找到 wheel_ops 中的最大绝对值
    for (uint8_t i = 0; i < 4; i++)
    {
        if (abs(wheel_ops[i]) > max)
        {
            max = abs(wheel_ops[i]);
        }
    }

    // 如果最大值超过 MAX_WHEEL_RPM，则按比例缩放
    if (max > MAX_WHEEL_RPM)
    {
        float rate = (MAX_WHEEL_RPM) / max;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_ops[i] *= rate;
        }
    }
}
