#include "board_com.h"
#include "can.h"
#include "cmsis_os.h"
#include "dji.h"
#include "dwt.h"
#include "message_center.h"
#include "motor_def.h"
#include "offline.h"
#include "referee.h"
#include "referee_protocol.h"
#include "robotdef.h"
#include "systemwatch.h"
#include "user_lib.h"
#include "chassiscmd.h"


#define LOG_TAG              "chassis"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

static osThreadId chassisTaskHandle;

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令 
static DJIMotor_t *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static PIDInstance chassis_follow_pid;
static Subscriber_t *chassis_sub;                  // cmd控制消息订阅者
static Hit_Check_t hit_check={0};

extern void ChassisCalculate(float chassis_vx, float chassis_vy, float chassis_wz,float *wheel_ops);
void ChassisInit(void)
{
    PID_Init_Config_s config = {.MaxOut = 0.5,
                                .IntegralLimit = 0.01,
                                .DeadBand = 1,
                                .Kp = 0.01,
                                .Ki = 0,
                                .Kd = 0.001,
                                .Improve = 0x01}; // enable integratiaon limit
    PIDInit(&chassis_follow_pid, &config);

    
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .offline_device_motor ={
          .timeout_ms = 100,                              // 超时时间
          .level = OFFLINE_LEVEL_HIGH,                     // 离线等级
          .enable = OFFLINE_ENABLE,                       // 是否启用离线管理
        },
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .lqr_config ={
                .K ={0.0043f}, 
                .output_max = 6.0f,
                .output_min =-6.0f,
                .state_dim = 1,
            }
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .feedback_reverse_flag =FEEDBACK_DIRECTION_NORMAL,
            .control_algorithm =CONTROL_LQR,
            .PowerControlState =PowerControlState_ON,
        },
        .motor_type = M3508,
    };
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.offline_device_motor.name = "m3508_1";
    chassis_motor_config.offline_device_motor.beep_times = 1;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.offline_device_motor.name = "m3508_2";
    chassis_motor_config.offline_device_motor.beep_times = 2;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.offline_device_motor.name = "m3508_3";
    chassis_motor_config.offline_device_motor.beep_times = 3;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.offline_device_motor.name = "m3508_4";
    chassis_motor_config.offline_device_motor.beep_times = 4;
    motor_lb = DJIMotorInit(&chassis_motor_config);


    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    Hit_Check_t hit_check = {
        .is_being_hit = 0,
        .last_hp = 0,
        .last_hit_time = 0,
        .is_first_check = 1
    };
}

void chassis_thread_entry(const void *parameter)
{
    SystemWatch_RegisterTask(chassisTaskHandle, "chassisTask");
    for (;;) {
        SystemWatch_ReportTaskAlive(osThreadGetId());
        SubGetMessage(chassis_sub, &chassis_cmd_recv);
        if (!get_device_status(motor_lf->offline_index)
            && !get_device_status(motor_lb->offline_index)
            && !get_device_status(motor_rf->offline_index)
            && !get_device_status(motor_rb->offline_index))
        {
            if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
            { 
                DJIMotorStop(motor_lf);
                DJIMotorStop(motor_rf);
                DJIMotorStop(motor_lb);
                DJIMotorStop(motor_rb);
            }
            else
            { 
                DJIMotorEnable(motor_lf);
                DJIMotorEnable(motor_rf);
                DJIMotorEnable(motor_lb);
                DJIMotorEnable(motor_rb);
            }

            // 根据控制模式设定旋转速度
            switch (chassis_cmd_recv.chassis_mode)
            {
            case CHASSIS_ROTATE_REVERSE: // 自旋反转,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
                chassis_cmd_recv.wz =-2;
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
                PIDCalculate(&chassis_follow_pid,chassis_cmd_recv.offset_angle,0);
                chassis_cmd_recv.wz = chassis_follow_pid.Output;
                break;
            case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
                chassis_cmd_recv.wz = 0.45;
                break;
            case CHASSIS_AUTO_MODE:  
                {
                    uint16_t data_len = sizeof(ext_game_state_t);
                    const ext_game_state_t* game_data = GetRefereeDataByCmd(ID_game_state, &data_len);
                    if (game_data->game_progress == 4)
                    {
                        if (CheckRobotBeingHit(&hit_check)==0)
                        {
                            PIDCalculate(&chassis_follow_pid,chassis_cmd_recv.offset_angle,0);
                            chassis_cmd_recv.wz = chassis_follow_pid.Output;
                        }
                        if (chassis_cmd_recv.vx == 0 && chassis_cmd_recv.vy == 0 && CheckRobotBeingHit(&hit_check)==1)
                        {
                            chassis_cmd_recv.wz = 2;
                        }
                        if (chassis_cmd_recv.vx !=0 && chassis_cmd_recv.vy != 0 && CheckRobotBeingHit(&hit_check)==1)
                        {
                            chassis_cmd_recv.wz = 0.5;
                        }
                    }
                    else
                    {
                        chassis_cmd_recv.vx =0;
                        chassis_cmd_recv.vy =0;
                        chassis_cmd_recv.wz =0;
                    }
                }
            break;
            
            default:
                break;
            }

            // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
            // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
            static float sin_theta, cos_theta,wheel_ops[4];
            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
            chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

            ChassisCalculate(chassis_vx, chassis_vy, chassis_cmd_recv.wz, wheel_ops);
            DJIMotorSetRef(motor_lf, wheel_ops[0] * 6.0f);
            DJIMotorSetRef(motor_rf, wheel_ops[1] * 6.0f);
            DJIMotorSetRef(motor_lb, wheel_ops[2] * 6.0f);
            DJIMotorSetRef(motor_rb, wheel_ops[3] * 6.0f);
        }
        else
        {
            DJIMotorStop(motor_lf);
            DJIMotorStop(motor_rf);
            DJIMotorStop(motor_lb);
            DJIMotorStop(motor_rb);
        }
        osDelay(3);
    } 
}

void chassis_task_init(void){
    osThreadDef(chassisTask, chassis_thread_entry, osPriorityNormal, 0, 256);
    chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

    if (chassisTaskHandle == NULL)
    {
        log_e("Failed to create gimbal task");
    }
    log_i("gimbal task created");
    ChassisInit();
}

/**
 * @brief 检查机器人是否正在被击打
 * @param hit_check 击打检测结构体
 * @return 1表示正在被击打，0表示没有被击打
 */
uint8_t CheckRobotBeingHit(Hit_Check_t *hit_check)
{
    uint16_t data_len = sizeof(ext_game_robot_state_t);
    const ext_game_robot_state_t* robot_state = GetRefereeDataByCmd(ID_game_robot_state, &data_len);
    
    if (robot_state == NULL) {
        return 0;
    }

    // 首次检查时初始化数据
    if (hit_check->is_first_check) {
        hit_check->last_hp = robot_state->current_HP;
        hit_check->is_first_check = 0;
        return 0;
    }
    
    uint32_t current_time = xTaskGetTickCount();

    // 检测血量是否减少
    if (robot_state->current_HP < hit_check->last_hp) {
        hit_check->is_being_hit = 1;
        hit_check->last_hit_time = current_time;
        //log_w("Hit detected! HP decreased from %d to %d", hit_check->last_hp, robot_state->current_HP);
    }
    // 检查是否超过5秒没有受到新的伤害
    else if (hit_check->is_being_hit && 
             (current_time - hit_check->last_hit_time > 5000)) {
        hit_check->is_being_hit = 0;
        //log_i("Robot recovered from hit");
    }
    
    // 更新上一次的血量值
    hit_check->last_hp = robot_state->current_HP;
    
    return hit_check->is_being_hit;
}

