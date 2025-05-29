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
#include "user_lib.h"
#include "vcom.h"
#include <stdint.h>
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
    static float small_yaw_offset=0.0f;

    static float sine_time_yaw,sine_time_pitch = 0.0f;
    #define SINE_FREQ_YAW 0.2f           // 正弦运动频率(Hz)
    #define SINE_FREQ_Pitch 1.5f           // 正弦运动频率(Hz)
    #define YAW_SINE_AMP 80.0f      // yaw轴正弦运动幅度(度) - 基于中间位置的偏移量
    #define PITCH_SINE_AMP 10.0f     // pitch轴正弦运动幅度(度)
    static float auto_angle_record=0.0f;

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
            // 计算相对角度
            small_yaw_offset = INS.YawTotalAngle - small_yaw->measure.total_angle; 
            if (gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE)
            {
                DJIMotorEnable(big_yaw);
                DJIMotorEnable(small_yaw);
                DMMotorEnable(pitch_motor); 
            }
            else
            {
                DJIMotorStop(big_yaw);
                DJIMotorStop(small_yaw);
                DMMotorStop(pitch_motor); 
            }
            switch (gimbal_cmd_recv.gimbal_mode) 
            {
                case GIMBAL_KEEPING_SMALL_YAW:
                {
                    LQR_Init_Config_s lqr_config ={
                        .K ={22.36f,4.05f},
                        .output_max = 2.223,
                        .output_min =-2.223,
                        .state_dim = 2,
                        .compensation_type =COMPENSATION_NONE,
                    };
                    DJIMotorOuterLoop(big_yaw, ANGLE_LOOP, &lqr_config);
                    DJIMotorSetRef(big_yaw,auto_angle_record + gimbal_cmd_recv.yaw);
                    DMMotorSetRef(pitch_motor, SMALL_YAW_PITCH_HORIZON_ANGLE); 
                    DJIMotorSetRef(small_yaw, small_yaw_offset);
                    break;
                }
                case GIMBAL_KEEPING_BIG_YAW:
                {
                    LQR_Init_Config_s lqr_config ={
                        .K ={22.36f,4.05f},
                        .output_max = 2.223,
                        .output_min =-2.223,
                        .state_dim = 2,
                        .compensation_type =COMPENSATION_NONE,
                    };
                    DJIMotorOuterLoop(big_yaw, ANGLE_LOOP, &lqr_config);
                    DJIMotorSetRef(big_yaw,auto_angle_record + gimbal_cmd_recv.yaw);
                    DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
                    DJIMotorSetRef(small_yaw, gimbal_cmd_recv.small_yaw + small_yaw_offset);
                    break;
                }
                case GIMBAL_AUTO_MODE:
                {
                    if (get_device_status(vcom_receive.offline_index)==0)
                    {
                        auto_angle_record = dm_imu.YawTotalAngle;
                        if (vcom_receive.recv.distance == -1 && vcom_receive.recv.vx ==0 && vcom_receive.recv.vy == 0)
                        {
                            LQR_Init_Config_s lqr_config ={
                                .K ={50.0f},
                                .output_max = 2.223,
                                .output_min =-2.223,
                                .state_dim = 1,
                                .compensation_type =COMPENSATION_NONE,
                            };
                            DJIMotorOuterLoop(big_yaw, SPEED_LOOP, &lqr_config);
                            DJIMotorSetRef(big_yaw,0.2 * 6.0f);
                            DJIMotorSetRef(small_yaw,small_yaw_offset);
                        }
                        else
                        {
                            LQR_Init_Config_s lqr_config ={
                                .K ={22.36f,4.05f},
                                .output_max = 2.223,
                                .output_min =-2.223,
                                .state_dim = 2,
                                .compensation_type =COMPENSATION_NONE,
                            };
                            DJIMotorOuterLoop(big_yaw, ANGLE_LOOP, &lqr_config);
                            DJIMotorSetRef(big_yaw,dm_imu.YawTotalAngle);
                        }
                        if (vcom_receive.recv.distance ==-1)
                        {
                            float pitch_sine = PITCH_SINE_AMP * arm_sin_f32(2.0f * PI * SINE_FREQ_Pitch * sine_time_pitch);
                            // pitch轴在水平位置基础上进行正弦运动
                            float pitch_target = SMALL_YAW_PITCH_HORIZON_ANGLE + pitch_sine;
                            // 限制pitch角度范围
                            pitch_target = fminf(fmaxf(pitch_target, SMALL_YAW_PITCH_MIN_ANGLE), SMALL_YAW_PITCH_MAX_ANGLE);
                            DMMotorSetRef(pitch_motor, pitch_target);
                            
                            if (vcom_receive.recv.vx !=0 || vcom_receive.recv.vy != 0)
                            {
                                // 基于small_yaw_offset(中间位置)进行正弦运动
                                float yaw_sine = YAW_SINE_AMP * arm_sin_f32(2.0f * PI * SINE_FREQ_YAW * sine_time_yaw);
                                // small_yaw基于中间位置(small_yaw_offset)进行正弦运动
                                DJIMotorSetRef(small_yaw, small_yaw_offset + yaw_sine);
                            }
                            
                        }
                        else
                        {
    
                            DMMotorSetRef(pitch_motor, vcom_receive.recv.pitch);
                            DJIMotorSetRef(small_yaw, vcom_receive.recv.yaw + INS.YawRoundCount * 360.0f);
                        }
                        // 更新时间
                        sine_time_yaw += 0.003f;
                        if(sine_time_yaw >= (1.0f/SINE_FREQ_YAW)) {
                            sine_time_yaw = 0.0f;
                        }
                        sine_time_pitch += 0.003f;
                        if(sine_time_pitch >= (1.0f/SINE_FREQ_Pitch)) {
                            sine_time_pitch = 0.0f;
                        }
                    }
                    else
                    {
                        DMMotorSetRef(pitch_motor, SMALL_YAW_PITCH_HORIZON_ANGLE); 
                        DJIMotorSetRef(small_yaw, small_yaw_offset);
                        DJIMotorSetRef(big_yaw,dm_imu.YawTotalAngle);
                    }
                    break;
                }
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
