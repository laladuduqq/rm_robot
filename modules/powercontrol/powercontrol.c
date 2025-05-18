#include "powercontroller.h"
#include "dji.h"
#include "motor_def.h"
#include <math.h>
#include "controller.h"
#include "referee.h"
#include "referee_protocol.h"
#include "robotdef.h"


#define LOG_TAG              "powercontrol"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

#define TOQUE_COEFFICIENT 1.99688994e-6f

static PowerControl_t powercontrol;

void powercontrol_init(void)
{
    for (size_t i = 0; i < 4; i++)
    {
        powercontrol.K1[i]=1.23e-07;
        powercontrol.K2[i]=1.453e-07;
        powercontrol.constant[i] = 4.081f; //电机输出轴实际转矩，单位N·m
        powercontrol.power_control_out[i] = 0;
        powercontrol.initial_give_power[i] = 0;
    }

    PID_Init_Config_s config = {.MaxOut = 30,
        .IntegralLimit = 20,
        .DeadBand = 5,
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
        .Improve = 0x01}; // enable integratiaon limit
    PIDInit(&powercontrol.buffer_energy_pid, &config);
}

/**
 * @brief 设置底盘的最大功率限制
 *
 * @param power_limit 限制的功率值
 */
void SetPowerLimit(float power_limit)
{
    powercontrol.chassis_max_power = power_limit-5; //保留5w余量
}


void PowerControlDji(DJIMotor_t *motor, float control_output)
{
    // 检查电机的功率控制状态是否开启
    if (motor->motor_settings.PowerControlState != PowerControlState_ON)
    {
        return;
    }

    // 保存控制输出用于功率计算
    powercontrol.power_control_out[motor->message_num] = control_output;

    // 计算预期功率
    float A = powercontrol.K1[motor->message_num] * control_output * control_output;
    float B = powercontrol.K2[motor->message_num] * motor->measure.speed_rpm * motor->measure.speed_rpm;
    float C = TOQUE_COEFFICIENT * motor->measure.speed_rpm * control_output;
    powercontrol.initial_give_power[motor->message_num] = A + B + C + powercontrol.constant[motor->message_num];

    // 计算实际功率
    powercontrol.real_power[motor->message_num] = TOQUE_COEFFICIENT * motor->measure.real_current * motor->measure.speed_rpm
                                               + powercontrol.K1[motor->message_num] * motor->measure.speed_rpm * motor->measure.speed_rpm 
                                               + powercontrol.K2[motor->message_num] * motor->measure.real_current * motor->measure.real_current
                                               + powercontrol.constant[motor->message_num];
}



// 计算总功率并进行功率分配
void PowerControlDjiFinalize(DJIMotor_t **motor_list, uint8_t motor_count)
{
    powercontrol.initial_total_power = 0;
    powercontrol.real_total_power = 0;
    DJIMotor_t *motor = NULL;

    #if USING_REFREE_SYSTEM == 1 
        uint16_t data_len = sizeof(ext_power_heat_data_t);
        const ext_power_heat_data_t* power_data = GetRefereeDataByCmd(ID_power_heat_data, &data_len);
        if(power_data != NULL) {
            powercontrol.buffer_energy = power_data->buffer_energy;
        }
        data_len = sizeof(ext_game_robot_state_t);
        const ext_game_robot_state_t* robot_data = GetRefereeDataByCmd(ID_game_robot_state, &data_len);
        if(robot_data != NULL) {
            powercontrol.chassis_max_power = robot_data->chassis_power_limit;
        }
        float pid_output = PIDCalculate(&powercontrol.buffer_energy_pid, powercontrol.buffer_energy, 40.0f);
        powercontrol.chassis_max_power -= pid_output;
    #else
        powercontrol.chassis_max_power = 80;
        powercontrol.buffer_energy = 0;
    #endif
    
    // 使用传入的电机列表进行遍历
    for(uint8_t i = 0; i < motor_count; i++) 
    {
        motor = motor_list[i];
        if(motor && motor->motor_settings.PowerControlState == PowerControlState_ON)
        {
            if(powercontrol.initial_give_power[motor->message_num] < 0) {
                continue;
            }
            powercontrol.initial_total_power += powercontrol.initial_give_power[motor->message_num];
            powercontrol.real_total_power += powercontrol.real_power[motor->message_num];
        }
    }

    if(powercontrol.initial_total_power > powercontrol.chassis_max_power)
    {        
        float power_ratio = 1.0f;
            
        // 检查预期总功率限制
        if(powercontrol.initial_total_power > powercontrol.chassis_max_power) {
            power_ratio = powercontrol.chassis_max_power / powercontrol.initial_total_power;
        }
            
        // 检查实际总功率限制，如果实际功率比例更小则使用实际功率比例
        if(powercontrol.real_total_power > powercontrol.chassis_max_power) {
            float real_power_ratio = powercontrol.chassis_max_power / powercontrol.real_total_power;
            if(real_power_ratio < power_ratio) {
                power_ratio = real_power_ratio;
            }
        }
        
        // 只有需要限制时才进行功率分配
        if(power_ratio < 1.0f)
        {
                for(uint8_t i = 0; i < motor_count; i++)
                {
                    motor = motor_list[i];
                    if(motor && motor->motor_settings.PowerControlState == PowerControlState_ON)
                    {
                        // 使用功率比例计算缩放后的期望功率
                        powercontrol.scaled_give_power[motor->message_num] = 
                            powercontrol.initial_give_power[motor->message_num] * power_ratio;
                        
                        if(powercontrol.scaled_give_power[motor->message_num] < 0) {
                            continue;
                        }
        
                        float a = powercontrol.K1[motor->message_num];
                        float b = TOQUE_COEFFICIENT * motor->measure.speed_rpm;
                        float c = powercontrol.K1[motor->message_num] * motor->measure.speed_rpm * motor->measure.speed_rpm 
                                 - powercontrol.scaled_give_power[motor->message_num] + powercontrol.constant[motor->message_num];
                        
                        powercontrol.inside[motor->message_num] = b * b - 4 * a * c;
                        if(powercontrol.inside[motor->message_num] < 0) {
                            continue;
                        }
        
                        // 计算最终输出并限幅
                        if(powercontrol.power_control_out[motor->message_num] > 0)
                        {
                            float temp = (-b + sqrt(powercontrol.inside[motor->message_num])) / (2 * a);
                            powercontrol.power_control_out[motor->message_num] = 
                                temp > 15000 ? 15000 : temp;
                        }
                        if(powercontrol.power_control_out[motor->message_num] < 0)
                        {
                            float temp = (-b - sqrt(powercontrol.inside[motor->message_num])) / (2 * a);
                            powercontrol.power_control_out[motor->message_num] = 
                                temp < -15000 ? -15000 : temp;
                        }
                        
                        // 再次验证实际功率
                        float new_power = TOQUE_COEFFICIENT * powercontrol.power_control_out[motor->message_num] * motor->measure.speed_rpm
                                       + powercontrol.K1[motor->message_num] * motor->measure.speed_rpm * motor->measure.speed_rpm 
                                       + powercontrol.K2[motor->message_num] * powercontrol.power_control_out[motor->message_num] * powercontrol.power_control_out[motor->message_num]
                                       + powercontrol.constant[motor->message_num];
                                       
                        // 如果单个电机功率仍然过大，进行二次限制
                        if(new_power > powercontrol.chassis_max_power / motor_count) {
                            powercontrol.power_control_out[motor->message_num] *= 
                                sqrt(powercontrol.chassis_max_power / (motor_count * new_power));
                        }
                    }
                }
        }
    }
}

// 获取电机的功率控制输出
int16_t GetPowerControlOutput(uint8_t motor_num)
{

    return (int16_t)powercontrol.power_control_out[motor_num];
}

