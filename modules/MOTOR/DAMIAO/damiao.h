#ifndef __DAMIAO_H
#define __DAMIAO_H

#include "motor_def.h"

#define DM_P_MIN -12.5f
#define DM_P_MAX 12.5f
#define DM_V_MIN -30.0f
#define DM_V_MAX 30.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f
#define DM_T_MIN -10.0f
#define DM_T_MAX 10.0f

#define MIT_MODE 		0x000
#define POS_MODE		0x100
#define SPEED_MODE		0x200

#define DM_MOTOR_CNT	12

typedef enum 
{
    DM_NO_ERROR = 0X00,
    OVERVOLTAGE_ERROR = 0X08,
    UNDERVOLTAGE_ERROR = 0X09,
    OVERCURRENT_ERROR = 0X0A,
    MOS_OVERTEMP_ERROR = 0X0B,
    MOTOR_COIL_OVERTEMP_ERROR = 0X0C,
    COMMUNICATION_LOST_ERROR = 0X0D,
    OVERLOAD_ERROR = 0X0E,
} DMMotorError_t;

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
    DMMotorError_t Error_Code; 
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;

typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;    // 电机控制器
    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag;
    uint8_t offline_index;
    Can_Device *can_device;
    uint32_t DMMotor_Mode_type;
}DMMOTOR_t;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;


DMMOTOR_t *DMMotorInit(Motor_Init_Config_s *config,uint32_t DM_Mode_type);

void DMMotorSetRef(DMMOTOR_t *motor, float ref);
void DMMotorOuterLoop(DMMOTOR_t *motor,Closeloop_Type_e closeloop_type,LQR_Init_Config_s *lqr_config);
void DMMotorEnable(DMMOTOR_t *motor);
void DMMotorStop(DMMOTOR_t *motor);
void DMMotorCaliEncoder(DMMOTOR_t *motor);
void DMMotorDecode(const CAN_HandleTypeDef* hcan,const  uint32_t rx_id);
void DMMotorcontrol(void);
void DMMotorSetMode(DMMotor_Mode_e cmd, DMMOTOR_t *motor);


#endif // DAMIAO_H
