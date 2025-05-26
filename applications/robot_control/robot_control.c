#include "board_com.h"
#include "can.h"
#include "dji.h"
#include "imu.h"
#include "message_center.h"
#include "referee.h"
#include "sbus.h"
#include "usbd_cdc_if.h"
#include "user_lib.h"
#include "offline.h"
#include "robotdef.h"
#include "vcom.h"
#include <stdint.h>
#include <string.h>


#define LOG_TAG              "robotcontrol"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>


//部分仅限内部使用函数声明
static float CalcOffsetAngle(float getyawangle);
static void RemoteControlSet(Chassis_Ctrl_Cmd_s *Chassis_Ctrl,Shoot_Ctrl_Cmd_s *Shoot_Ctrl,Gimbal_Ctrl_Cmd_s *Gimbal_Ctrl);

#ifdef ONE_BOARD

#endif

#ifndef ONE_BOARD
    #if defined (GIMBAL_BOARD)
            static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
            static Gimbal_Ctrl_Cmd_s gimbal_cmd_send={0};      // 传递给云台的控制信息

            static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
            static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

            static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
            static Shoot_Ctrl_Cmd_s shoot_cmd_send={0};      // 传递给发射的控制信息

            static Chassis_Ctrl_Cmd_s chassis_cmd_send = {0};      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
            //static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

            static board_com_t *board_com = NULL; // 板间通讯实例
            static struct Sentry_Send_s sentry_send;
            void robot_control_init(void)
            {
                //订阅 发布注册
                gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
                gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
                shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

                //板间通讯初始化
                board_com_init_t board_com_config = {
                    .offline_manage_init = {
                      .name = "board_com",
                      .timeout_ms = 100,
                      .level = OFFLINE_LEVEL_HIGH,
                      .beep_times = 8,
                      .enable = OFFLINE_ENABLE,
                    },
                    .Can_Device_Init_Config = {
                        .can_handle = &hcan2,
                        .tx_id = GIMBAL_ID,
                        .rx_id = CHASSIS_ID,
                        .tx_mode = CAN_MODE_BLOCKING,
                        .rx_mode = CAN_MODE_IT,
                        .can_callback = board_recv
                    }
                };
                board_com = board_com_init(&board_com_config);
            }

            void robot_control(void)
            {
                SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
                chassis_cmd_send.offset_angle = CalcOffsetAngle(gimbal_fetch_data.yaw_motor_single_round_angle);
                RemoteControlSet(&chassis_cmd_send,&shoot_cmd_send,&gimbal_cmd_send);
                
                PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

                if (gimbal_cmd_send.gimbal_mode == GIMBAL_AUTO_MODE)
                {
                    offline_device_enable(vcom_receive.offline_index);
                    if (get_device_status(vcom_receive.offline_index)==0)
                    {
                        chassis_cmd_send.vx = vcom_receive.recv.vx * -300.0f;
                        chassis_cmd_send.vy = vcom_receive.recv.vy * -300.0f;
                        if (gimbal_cmd_send.gimbal_mode == GIMBAL_AUTO_MODE)
                        {
                            if (vcom_receive.recv.fire_advice ==1)
                            {
                                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                            }
                            else
                            {
                                shoot_cmd_send.load_mode = LOAD_STOP;
                            }
                        }
                    }
                }
                else
                {
                    offline_device_disable(vcom_receive.offline_index);
                }
                PubPushMessage(shoot_cmd_pub,(void *)&shoot_cmd_send);

                board_send(&chassis_cmd_send);    

                sentry_send.header = 0xaa;
                sentry_send.current_hp_percent = board_com->Chassis_Upload_Data.current_hp_percent;
                sentry_send.projectile_allowance_17mm = board_com->Chassis_Upload_Data.projectile_allowance_17mm;  //剩余发弹量
                sentry_send.power_management_shooter_output = board_com->Chassis_Upload_Data.power_management_shooter_output; // 功率管理 shooter 输出
                sentry_send.current_hp_percent = board_com->Chassis_Upload_Data.current_hp_percent; // 机器人当前血量百分比
                sentry_send.outpost_HP = board_com->Chassis_Upload_Data.outpost_HP;     //前哨站血量
                sentry_send.base_HP = board_com->Chassis_Upload_Data.base_HP;        //基地血量
                sentry_send.game_progess = board_com->Chassis_Upload_Data.game_progess;
                sentry_send.game_time = board_com->Chassis_Upload_Data.game_time;
                sentry_send.mode = 0;
                sentry_send.roll = INS.Roll;
                sentry_send.pitch = INS.Pitch * (-1);
                sentry_send.yaw = INS.Yaw;
                sentry_send.end = 0x0D;
                uint8_t data[sizeof(struct Sentry_Send_s)]={0};
                memcpy(data,&sentry_send,sizeof(struct Sentry_Send_s));
                CDC_Transmit_FS(data,sizeof(struct Sentry_Send_s));           
            } 
    #else
            static Chassis_referee_Upload_Data_s Chassis_referee_Upload_Data;
            static board_com_t *board_com = NULL; // 板间通讯实例
            static Publisher_t *chassis_cmd_pub;                  // 底盘控制消息发布者
            static Chassis_Ctrl_Cmd_s chassis_cmd_send={0};      // 传递给底盘的控制信息
            void robot_control_init(void)
            {
                //订阅注册
                chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
                //板间通讯初始化
                board_com_init_t board_com_config = {
                    .offline_manage_init = {
                      .name = "board_com",
                      .timeout_ms = 100,
                      .level = OFFLINE_LEVEL_HIGH,
                      .beep_times = 7,
                      .enable = OFFLINE_ENABLE,
                    },
                    .Can_Device_Init_Config = {
                        .can_handle = &hcan2,
                        .tx_id = CHASSIS_ID,
                        .rx_id = GIMBAL_ID,
                        .tx_mode = CAN_MODE_BLOCKING,
                        .rx_mode = CAN_MODE_IT,
                        .can_callback = board_recv
                    }
                };
                board_com = board_com_init(&board_com_config);
            }

            void robot_control(void)
            {
                if (get_device_status(board_com->offlinemanage_index)==1)
                {
                    chassis_cmd_send.vx =0;
                    chassis_cmd_send.vy =0;
                    chassis_cmd_send.wz =0;
                    chassis_cmd_send.offset_angle = 0;
                    chassis_cmd_send.chassis_mode =CHASSIS_ZERO_FORCE;
                }
                else
                {
                    chassis_cmd_send = *(Chassis_Ctrl_Cmd_s *)BoardRead();
                }
                PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
                referee_to_gimbal(&Chassis_referee_Upload_Data);
                board_send(&Chassis_referee_Upload_Data);     
            } 
    #endif 

    #if defined (DRONE_MODE)
    #endif

#endif 


/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
#if defined (GIMBAL_BOARD) || defined (ONE_BOARD) 
static float CalcOffsetAngle(float getyawangle)
{
    static float offsetangle;
    // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (getyawangle > YAW_ALIGN_ANGLE && getyawangle <= 180.0f + YAW_ALIGN_ANGLE)
    {    
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
    else if (getyawangle > 180.0f + YAW_ALIGN_ANGLE)
    {    
        offsetangle = getyawangle - YAW_ALIGN_ANGLE - 360.0f;
        return offsetangle;
    }
    else
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
#else // 小于180度
    if (getyawangle > YAW_ALIGN_ANGLE)
    {    
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
    else if (getyawangle <= YAW_ALIGN_ANGLE && getyawangle >= YAW_ALIGN_ANGLE - 180.0f)
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE;
        return offsetangle;
    }
    else
    {
        offsetangle = getyawangle - YAW_ALIGN_ANGLE + 360.0f;
        return offsetangle;
    }
#endif
}


#define DEAD_ZONE(value, min, max, center) ((value) >= (min) && (value) <= (max) ? (center) : (value))

#if CONTROL_SOURCE == 1
static void RemoteControlSet(Chassis_Ctrl_Cmd_s *Chassis_Ctrl,Shoot_Ctrl_Cmd_s *Shoot_Ctrl,Gimbal_Ctrl_Cmd_s *Gimbal_Ctrl)
{
    static SBUS_CH_Struct* sbus = NULL;
    if (sbus == NULL) {
        sbus = Get_SBUS_Data();
    }
    if (get_device_status(sbus->offline_index) == STATE_ONLINE)
    {
        
        Chassis_Ctrl->vx = -790.0f * (float)(DEAD_ZONE(sbus->CH2, 1020, 1030, 1024) - SBUS_CHX_BIAS) / ((SBUS_CHX_UP - SBUS_CHX_DOWN) / 2.0f);
        Chassis_Ctrl->vy =  790.0f * (float)(DEAD_ZONE(sbus->CH1, 1020, 1030, 1024) - SBUS_CHX_BIAS) / ((SBUS_CHX_UP - SBUS_CHX_DOWN) / 2.0f); //根据 2 * PI * wheel_radius * rpm / 60.0f可得最大线速度

        switch (sbus->CH6)
        {
            case SBUS_CHX_DOWN:
                    Gimbal_Ctrl->gimbal_mode = GIMBAL_KEEPING_SMALL_YAW;

                    Gimbal_Ctrl->yaw -= 0.001f * (float)(DEAD_ZONE(sbus->CH4, 1020, 1030, 1024)-SBUS_CHX_BIAS);

                    Gimbal_Ctrl->small_yaw = SMALL_YAW_ALIGN_ANGLE;
                    Gimbal_Ctrl->pitch = SMALL_YAW_PITCH_HORIZON_ANGLE;
                    break;

            case SBUS_CHX_BIAS:
                    Gimbal_Ctrl->gimbal_mode = GIMBAL_KEEPING_BIG_YAW;
                    
                    Gimbal_Ctrl->small_yaw -= 0.001f * (float)(DEAD_ZONE(sbus->CH4, 1020, 1030, 1024)-SBUS_CHX_BIAS);
                    VAL_LIMIT(Gimbal_Ctrl->small_yaw, SMALL_YAW_MIN_ANGLE, SMALL_YAW_MAX_ANGLE);
                    Gimbal_Ctrl->pitch += 0.001f * (float)(DEAD_ZONE(sbus->CH3, 1020, 1030, 1024)- SBUS_CHX_BIAS);
                    VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);
                    break;

            case SBUS_CHX_UP:
                    Gimbal_Ctrl->gimbal_mode = GIMBAL_AUTO_MODE;
                    Chassis_Ctrl->chassis_mode = CHASSIS_AUTO_MODE;
                    break;

            default:
                break;
        }

        // 处理CH9和CH10的逻辑
        if (sbus->CH9 > SBUS_CHX_BIAS)
        {
            
        }
        if (sbus->CH10 > SBUS_CHX_BIAS)
        {
            
        }

        // 处理射击控制逻辑
        switch (sbus->CH5)
        {
            case SBUS_CHX_DOWN:
            Shoot_Ctrl->shoot_mode =SHOOT_OFF;
                Shoot_Ctrl->friction_mode = FRICTION_OFF;
                Shoot_Ctrl->load_mode = LOAD_STOP;
                break;

            case SBUS_CHX_BIAS:
                Shoot_Ctrl->shoot_mode =SHOOT_ON;
                Shoot_Ctrl->friction_mode = FRICTION_OFF;
                Shoot_Ctrl->load_mode = LOAD_STOP;
                break;

            case SBUS_CHX_UP:
                Shoot_Ctrl->shoot_mode =SHOOT_ON;
                Shoot_Ctrl->friction_mode = FRICTION_ON;
                if (sbus->CH7 == SBUS_CHX_DOWN)
                {
                    Shoot_Ctrl->load_mode = LOAD_1_BULLET;
                }
                else if (sbus->CH7 == SBUS_CHX_BIAS)
                {
                    Shoot_Ctrl->load_mode = LOAD_STOP;
                }
                else if (sbus->CH7 == SBUS_CHX_UP)
                {
                    Shoot_Ctrl->load_mode = LOAD_BURSTFIRE;
                }
                else
                {
                    Shoot_Ctrl->load_mode = LOAD_STOP;
                }
                break;

            default:
                break;
        }
        // 处理底盘控制逻辑
        switch (sbus->CH8)
        {
            case SBUS_CHX_DOWN:
                if(Chassis_Ctrl->chassis_mode != CHASSIS_AUTO_MODE){
                    Chassis_Ctrl->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                }
                break;

            case SBUS_CHX_BIAS:
                Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE;
                break;

            case SBUS_CHX_UP:
                Chassis_Ctrl->chassis_mode = CHASSIS_ROTATE_REVERSE;
                break;

            default:
                break;
        }
    }
    else
    {
        // 处理离线状态
        Gimbal_Ctrl->gimbal_mode = GIMBAL_ZERO_FORCE;
        Chassis_Ctrl->chassis_mode = CHASSIS_ZERO_FORCE;
        Shoot_Ctrl->shoot_mode = SHOOT_OFF;
        Shoot_Ctrl->friction_mode = FRICTION_OFF;
        Shoot_Ctrl->load_mode = LOAD_STOP;
        memset(Chassis_Ctrl, 0, sizeof(Chassis_Ctrl_Cmd_s));
    }
}
#endif

#endif
