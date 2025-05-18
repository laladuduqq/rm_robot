#ifndef __POWERCONTROLLER_H
#define __POWERCONTROLLER_H

#include "dji.h"

typedef struct 
{
    float K1[4];  //
    float K2[4];  //dji motor 铜损 系数
    float constant[4]; //静态功耗
    float power_control_out[4]; //功控输出
    float initial_give_power[4]; //上述pid计算下每个电机估计消耗的功率
    float chassis_max_power;     //设定的功率上限
    float initial_total_power;   //无功控计算下总预计消耗功率
    float scaled_give_power[4];  //缩放后每个电机分配的功率
    float real_power[4];         //实际电机消耗的功率，用于观察
    float real_total_power;      //实际底盘消耗的总功率
    float inside[4];
    float buffer_energy;
    PIDInstance buffer_energy_pid; // 定义PID控制器实例
} PowerControl_t;


void powercontrol_init(void);
void SetPowerLimit(float power_limit);
int16_t GetPowerControlOutput(uint8_t motor_num);
void PowerControlDjiFinalize(DJIMotor_t **motor_list, uint8_t motor_count);
void PowerControlDji(DJIMotor_t *motor, float control_output);

#endif
