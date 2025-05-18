#ifndef __DJI_H
#define __DJI_H

#include "bsp_can.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C"{
#endif

#include "motor_def.h"

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.9f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_rpm;          // 转速
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
    float total_output_round; //输出轴总圈数
} DJI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    DJI_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;
    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志
    uint8_t offline_index;
    Can_Device *can_device;
} DJIMotor_t;


DJIMotor_t *DJIMotorInit(Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotor_t *motor, float ref);
void DJIMotorChangeFeed(DJIMotor_t *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void DJIMotorControl(void);
void DJIMotorStop(DJIMotor_t *motor);
void DJIMotorEnable(DJIMotor_t *motor);
void DJIMotorOuterLoop(DJIMotor_t *motor, Closeloop_Type_e outer_loop);
void DecodeDJIMotor(const CAN_HandleTypeDef* hcan,const  uint32_t rx_id);



#ifdef __cplusplus
}
#endif

#endif // DJI_H
