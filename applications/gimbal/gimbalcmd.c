#include "can.h"
#include "cmsis_os.h"
#include "damiao.h"
#include "dji.h"
#include "dwt.h"
#include "message_center.h"
#include "motor_def.h"
#include "offline.h"
#include "robotdef.h"
#include "imu.h"
#include "dm_imu.h"
#include "systemwatch.h"
#include "gimbalcmd.h"

#define LOG_TAG              "gimcmd"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

static osThreadId gimbalTaskHandle;


#if defined (SENTRY_MODE)
    static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
    static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
    static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
    static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息


    DJIMotor_t *big_yaw = NULL;
    DJIMotor_t *small_yaw = NULL; 
    DMMOTOR_t *pitch_motor = NULL; 
    void gimbal_init(void)
    {
        Motor_Init_Config_s yaw_config = {
            .offline_device_motor ={
              .name = "6020_big",                        // 设备名称
              .timeout_ms = 100,                              // 超时时间
              .level = OFFLINE_LEVEL_HIGH,                     // 离线等级
              .beep_times = 2,                                // 蜂鸣次数
              .enable = OFFLINE_ENABLE,                       // 是否启用离线管理
            },
            .can_init_config = {
                .can_handle = &hcan2,
                .tx_id = 1,
            },
            .controller_param_init_config = {
                .other_angle_feedback_ptr = &dm_imu.YawTotalAngle,
                .other_speed_feedback_ptr = &dm_imu.gyro[2],
                .lqr_config ={
                    .K ={22.36f,4.05f},
                    .output_max = 2.223,
                    .output_min =-2.223,
                    .state_dim = 2,
                    .compensation_type =COMPENSATION_NONE,
                }
            },
            .controller_setting_init_config = {
                .control_algorithm = CONTROL_LQR,
                .feedback_reverse_flag =FEEDBACK_DIRECTION_NORMAL,
                .angle_feedback_source =OTHER_FEED,
                .speed_feedback_source =OTHER_FEED,
                .outer_loop_type = ANGLE_LOOP,
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            },
            .motor_type = GM6020_CURRENT,
        };
        big_yaw = DJIMotorInit(&yaw_config);

        Motor_Init_Config_s small_yaw_config = {
            .offline_device_motor ={
              .name = "6020_small",                        // 设备名称
              .timeout_ms = 100,                              // 超时时间
              .level = OFFLINE_LEVEL_HIGH,                     // 离线等级
              .beep_times = 3,                                // 蜂鸣次数
              .enable = OFFLINE_ENABLE,                       // 是否启用离线管理
            },
            .can_init_config = {
                .can_handle = &hcan1,
                .tx_id = 1,
            },
            .controller_param_init_config = {
                .other_angle_feedback_ptr = &INS.YawTotalAngle,
                .other_speed_feedback_ptr = &INS.Gyro[2],
                .lqr_config ={
                    .K ={17.32f,1.0f},
                    .output_max = 2.223,
                    .output_min =-2.223,
                    .state_dim = 2,
                    .compensation_type =COMPENSATION_NONE,
                }
            },
            .controller_setting_init_config = {
                .control_algorithm = CONTROL_LQR,
                .feedback_reverse_flag =FEEDBACK_DIRECTION_NORMAL,
                .angle_feedback_source =OTHER_FEED,
                .speed_feedback_source =OTHER_FEED,
                .outer_loop_type = ANGLE_LOOP,
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            },
            .motor_type = GM6020_CURRENT,
        };
        small_yaw =DJIMotorInit(&small_yaw_config);

        // PITCH
        Motor_Init_Config_s pitch_config = {
            .offline_device_motor ={
              .name = "dm4310",                        // 设备名称
              .timeout_ms = 100,                              // 超时时间
              .level = OFFLINE_LEVEL_HIGH,                     // 离线等级
              .beep_times = 4,                                // 蜂鸣次数
              .enable = OFFLINE_ENABLE,                       // 是否启用离线管理
            },
            .can_init_config = {
                .can_handle = &hcan1,
                .tx_id = 0X23,
                .rx_id = 0X206,
            },
            .controller_param_init_config = {
                .other_angle_feedback_ptr = &INS.Pitch,
                .other_speed_feedback_ptr = &INS.Gyro[0],
                .lqr_config ={
                    .K ={44.7214f,3.3411f}, //28.7312f,2.5974f
                    .output_max = 7,
                    .output_min = -7,
                    .state_dim = 2,
                    .compensation_type =COMPENSATION_GRAVITY,
                    .arm_length = 0.09,
                    .gravity_force = 16,
                }
            },
            .controller_setting_init_config = {
                .control_algorithm = CONTROL_LQR,
                .feedback_reverse_flag =FEEDBACK_DIRECTION_REVERSE,
                .angle_feedback_source =OTHER_FEED,
                .speed_feedback_source =OTHER_FEED,
                .outer_loop_type = ANGLE_LOOP,
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            },
            .motor_type = DM4310,
        };
        pitch_motor = DMMotorInit(&pitch_config,MIT_MODE);

        gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
        gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    }
#endif

void gimbal_thread_entry(const void *parameter)
{   
    SystemWatch_RegisterTask(gimbalTaskHandle, "gimbalTask");
    for (;;)
    {
        SystemWatch_ReportTaskAlive(osThreadGetId());
        SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
        if (!get_device_status(big_yaw->offline_index) 
         && !get_device_status(small_yaw->offline_index) 
         && !get_device_status(pitch_motor->offline_index) ) 
        {
            switch (gimbal_cmd_recv.gimbal_mode) 
            {
                case GIMBAL_ZERO_FORCE:
                    DJIMotorStop(big_yaw);
                    DJIMotorStop(small_yaw);
                    DMMotorStop(pitch_motor); 
                    break;
                case GIMBAL_KEEPING_SMALL_YAW:
                {
                    DJIMotorEnable(big_yaw);
                    DJIMotorEnable(small_yaw);
                    DMMotorEnable(pitch_motor); 

                    DJIMotorSetRef(big_yaw,gimbal_cmd_recv.yaw);
                    DMMotorSetRef(pitch_motor, SMALL_YAW_PITCH_HORIZON_ANGLE); 

                    // 计算相对角度
                    float small_yaw_offset = INS.YawTotalAngle - small_yaw->measure.total_angle; 
                    DJIMotorSetRef(small_yaw, small_yaw_offset);
                    break;
                }
                case GIMBAL_KEEPING_BIG_YAW:
                {
                    DJIMotorEnable(big_yaw);
                    DJIMotorEnable(small_yaw);
                    DMMotorEnable(pitch_motor); 
                    float small_yaw_offset = INS.YawTotalAngle - small_yaw->measure.total_angle; 
                    DJIMotorSetRef(big_yaw,gimbal_cmd_recv.yaw);
                    DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
                    DJIMotorSetRef(small_yaw, gimbal_cmd_recv.small_yaw + small_yaw_offset);
                    break;
                }
                case GIMBAL_AUTO_MODE:
                    DJIMotorEnable(big_yaw);
                    DJIMotorEnable(small_yaw);
                    DMMotorEnable(pitch_motor); 

                    DJIMotorSetRef(big_yaw,gimbal_cmd_recv.yaw);
                    DMMotorSetRef(pitch_motor, SMALL_YAW_PITCH_HORIZON_ANGLE); 

                    // 计算相对角度
                    float small_yaw_offset = INS.YawTotalAngle - small_yaw->measure.total_angle; 
                    DJIMotorSetRef(small_yaw, small_yaw_offset);
                    break;
                case GIMBAL_GYRO_MODE:
                default:
                    break;
            }
        }
        else
        {
            DJIMotorStop(big_yaw);
            DJIMotorStop(small_yaw);
            DMMotorStop(pitch_motor);
        }

        // 设置反馈数据,主要是imu和yaw的ecd
        if (!get_device_status(big_yaw->offline_index))
        {
            gimbal_feedback_data.yaw_motor_single_round_angle = big_yaw->measure.angle_single_round;
            // 推送消息
            PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        }else
        {
            gimbal_feedback_data.yaw_motor_single_round_angle = YAW_ALIGN_ANGLE * ECD_ANGLE_COEF_DJI;
            // 推送消息
            PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        }
        osDelay(3);
    }
}

void gimbal_task_init(void)
{
    osThreadDef(gimbalTask, gimbal_thread_entry, osPriorityNormal, 0, 256);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    if (gimbalTaskHandle == NULL)
    {
        log_e("Failed to create gimbal task");
    }
    log_i("gimbal task created");

    gimbal_init();
}
