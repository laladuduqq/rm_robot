#include "dji.h"
#include "bsp_can.h"
#include "can.h"
#include "dwt.h"
#include "motor_def.h"
#include "offline.h"
#include "powercontroller.h"
#include "user_lib.h"
#include <stdint.h>


#define LOG_TAG              "dji"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>


static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
static DJIMotor_t *dji_motor_list[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算
// 存储未开启功率控制的电机输出
static float motor_outputs[DJI_MOTOR_CNT] = {0};
// 获取未开启功率控制的电机输出
static int16_t GetRawMotorOutput(uint8_t motor_num) {
    if(motor_num >= DJI_MOTOR_CNT) return 0;
    return (int16_t)motor_outputs[motor_num];
}
/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff 0x1fe 0x2fe
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF [3]:0x1fe
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF 
 */
static CanMessage_t sender_assignment[10] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId =0x1ff, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [1] = {.can_handle = &hcan1, .txconf.StdId =0x200, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [2] = {.can_handle = &hcan1, .txconf.StdId =0x2ff, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [3] = {.can_handle = &hcan1, .txconf.StdId =0x1fe, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [4] = {.can_handle = &hcan1, .txconf.StdId =0x2fe, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},

    [5] = {.can_handle = &hcan2, .txconf.StdId =0x1ff, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [6] = {.can_handle = &hcan2, .txconf.StdId =0x200, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [7] = {.can_handle = &hcan2, .txconf.StdId =0x2ff, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [8] = {.can_handle = &hcan2, .txconf.StdId =0x1fe, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
    [9] = {.can_handle = &hcan2, .txconf.StdId =0x2fe, .txconf.IDE = CAN_ID_STD ,.txconf.RTR = CAN_RTR_DATA , .txconf.DLC =8 ,.tx_buff = {0},},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[10] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotor_t *motor, Can_Device_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 6;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan2 ? 0 : 5;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;


        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_list[i]->can_device->can_handle == config->can_handle && dji_motor_list[i]->can_device->rx_id == config->rx_id)
            {
                log_e("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                log_e("[dji_motor] id [%d], can_bus [%d]", device->rx_id, (device->can_handle == &hcan1 ? 0 : 1));
                assert (1);
            }
        }
        break;


    case GM6020_CURRENT:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 3 : 8;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 7;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_list[i]->can_device->can_handle == config->can_handle && dji_motor_list[i]->can_device->rx_id == config->rx_id)
            {
                log_e("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                log_e("[dji_motor] id [%d], can_bus [%d]", device->rx_id, (device->can_handle == &hcan1 ? 0 : 1));
                assert (1);
            }
        }
        break;

    case GM6020_VOLTAGE:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 5;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 4 : 9;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_list[i]->can_device->can_handle == config->can_handle && dji_motor_list[i]->can_device->rx_id == config->rx_id)
            {
                log_e("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                log_e("[dji_motor] id [%d], can_bus [%d]", device->rx_id, (device->can_handle == &hcan1 ? 0 : 1));
                assert (1);
            }
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 根据返回的can_instance对反馈报文进行解析
 */
void DecodeDJIMotor(const CAN_HandleTypeDef* hcan,const uint32_t rx_id)
{
    for (uint8_t i = 0; i < idx; i++) {
        if (dji_motor_list[i] 
            &&dji_motor_list[i]->can_device->can_handle == hcan 
            && dji_motor_list[i]->can_device->rx_id == rx_id)
        {
            uint8_t *rxbuff = dji_motor_list[i]->can_device->rx_buff;    
            offline_device_update(dji_motor_list[i]->offline_index);
            // 找到对应电机，更新数据
            dji_motor_list[i]->measure.last_ecd = dji_motor_list[i]->measure.ecd;
            dji_motor_list[i]->measure.ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
            dji_motor_list[i]->measure.angle_single_round = ECD_ANGLE_COEF_DJI * (float)dji_motor_list[i]->measure.ecd;
            dji_motor_list[i]->measure.speed_rpm = (1.0f - SPEED_SMOOTH_COEF) * dji_motor_list[i]->measure.speed_rpm + 
                                                    SPEED_SMOOTH_COEF*(float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
            dji_motor_list[i]->measure.speed_aps = (1.0f - SPEED_SMOOTH_COEF) * dji_motor_list[i]->measure.speed_aps +
                                                    RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
            dji_motor_list[i]->measure.real_current = (1.0f - CURRENT_SMOOTH_COEF) * dji_motor_list[i]->measure.real_current +
                                                       CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
            dji_motor_list[i]->measure.temperature = rxbuff[6];
            // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
            if (dji_motor_list[i]->measure.ecd - dji_motor_list[i]->measure.last_ecd > 4096)
                dji_motor_list[i]->measure.total_round--;
            else if (dji_motor_list[i]->measure.ecd - dji_motor_list[i]->measure.last_ecd < -4096)
                dji_motor_list[i]->measure.total_round++;
            dji_motor_list[i]->measure.total_angle = dji_motor_list[i]->measure.total_round * 360 + dji_motor_list[i]->measure.angle_single_round;                
        }
    }
}

// 电机初始化,返回一个电机实例
DJIMotor_t *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotor_t *DJIMotor = (DJIMotor_t *)malloc(sizeof(DJIMotor_t));
    if (DJIMotor == NULL) {
        log_e("Failed to allocate memory for DJIMotor\n");
        return NULL;
    }
    memset(DJIMotor, 0, sizeof(DJIMotor_t));

    // motor basic setting 电机基本设置
    DJIMotor->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    DJIMotor->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(DJIMotor, &config->can_init_config); //更新rx_id
    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = config->can_init_config.can_handle,
        .tx_id = config->can_init_config.tx_id,
        .rx_id = config->can_init_config.rx_id,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
        .can_callback = DecodeDJIMotor
    };
    // 注册 CAN 设备并获取引用
    Can_Device *can_dev = BSP_CAN_Device_Init(&can_config);
    if (can_dev == NULL) {
        log_e("Failed to initialize CAN device for DJI motor");
        vPortFree(DJIMotor);
        return NULL;
    }
    // 保存设备指针
    DJIMotor->can_device = can_dev;

    DJIMotor->motor_settings.control_algorithm = config->controller_setting_init_config.control_algorithm;
    switch (config->controller_setting_init_config.control_algorithm) {
        case CONTROL_PID:
            // motor controller init 电机控制器初始化
            PIDInit(&DJIMotor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
            PIDInit(&DJIMotor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
            PIDInit(&DJIMotor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
            DJIMotor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
            DJIMotor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
            DJIMotor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
            DJIMotor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
            break;
        case CONTROL_LQR:
            LQRInit(&DJIMotor->motor_controller.lqr, &config->controller_param_init_config.lqr_config);
            DJIMotor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
            DJIMotor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
            DJIMotor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
            DJIMotor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
            break;
        case CONTROL_OTHER:
            // 未来添加其他控制算法的初始化
            break;
    }

    //掉线检测
    DJIMotor->offline_index =offline_device_register(&config->offline_device_motor);
    // 记录电机实例
    dji_motor_list[idx++] =DJIMotor;

    return DJIMotor;
}


void DJIMotorChangeFeed(DJIMotor_t *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        log_e("[dji_motor] loop type error, check memory access and func param\n"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void DJIMotorStop(DJIMotor_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotor_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotor_t *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// 设置参考值
void DJIMotorSetRef(DJIMotor_t *motor, float ref)
{
    switch (motor->motor_settings.control_algorithm) 
    {
        case CONTROL_PID:
            motor->motor_controller.pid_ref = ref;
            break;
        case CONTROL_LQR:
            motor->motor_controller.lqr_ref = ref;
            break;
        case CONTROL_OTHER:
            break;
    }
}

/*
    转换函数，根据dji电机int16_t发送范围与电机对应电流，将目标电流转化为int16_t
*/
int16_t currentToInteger(float I_min,float I_max,int16_t V_min,int16_t V_max,float current) {
    int16_t V = (int)((current - I_min) / (I_max - I_min) * (V_max - V_min) + V_min);
    if (V >V_max)
    {
        V=V_max;
    }
    if (V<V_min)
    {
        V=V_min;
    }
    return V;
}

static float CalculatePIDOutput(DJIMotor_t *motor)
{
    float pid_measure, pid_ref;
    
    pid_ref = motor->motor_controller.pid_ref;
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        pid_ref *= -1;

    // 速度环计算
    if ((motor->motor_settings.close_loop_type & SPEED_LOOP) && 
        (motor->motor_settings.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if (motor->motor_settings.feedforward_flag & SPEED_FEEDFORWARD)
            pid_ref += *motor->motor_controller.speed_feedforward_ptr;

        pid_measure = (motor->motor_settings.speed_feedback_source == OTHER_FEED) ? 
                     *motor->motor_controller.other_speed_feedback_ptr : 
                     motor->measure.speed_rpm;
                     
        pid_ref = PIDCalculate(&motor->motor_controller.speed_PID, pid_measure, pid_ref);
    }

    // 电流环计算
    if (motor->motor_settings.feedforward_flag & CURRENT_FEEDFORWARD)
        pid_ref += *motor->motor_controller.current_feedforward_ptr;
        
    if (motor->motor_settings.close_loop_type & CURRENT_LOOP)
    {
        pid_ref = PIDCalculate(&motor->motor_controller.current_PID, 
                              motor->measure.real_current, 
                              pid_ref);
    }

    if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) 
        pid_ref *= -1;

    return pid_ref;
}

static float CalculateLQROutput(DJIMotor_t *motor)
{
    float state0 = 0, state1 = 0;
    
    if(motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        motor->motor_controller.lqr_ref *= -1;
    }

    // 位置状态计算
    if ((motor->motor_settings.close_loop_type & ANGLE_LOOP) && 
        motor->motor_settings.outer_loop_type == ANGLE_LOOP)
    {
        state0 = (motor->motor_settings.angle_feedback_source == OTHER_FEED) ?
                 *motor->motor_controller.other_angle_feedback_ptr :
                 motor->measure.total_angle;
    }

    // 速度状态计算
    if ((motor->motor_settings.close_loop_type & SPEED_LOOP) && 
        (motor->motor_settings.outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        state1 = (motor->motor_settings.speed_feedback_source == OTHER_FEED) ?
                 *motor->motor_controller.other_speed_feedback_ptr :
                 motor->measure.speed_aps;
    }

    float torque = LQRCalculate(&motor->motor_controller.lqr, 
                               state0, 
                               state1, 
                               motor->motor_controller.lqr_ref);

    if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) torque *= -1;

    switch (motor->motor_type) 
    {                                          
        case GM6020_CURRENT:
            {
                return currentToInteger(-3.0f, 3.0f, -16384, 16384, (torque / 0.741f)); // 力矩常数（Nm/A）
                break;
            }
        case M3508:
            {
                return currentToInteger(-20.0f, 20.0f, -16384, 16384, (torque / 0.3f)); // 力矩常数（Nm/A）
                break;                                
            }
        case M2006:
            {
                return currentToInteger(-10.0f, 10.0f, -10000, 10000, (torque / (0.18f))); // 力矩常数（Nm/A）                                
                break;                                   
            }
        default:
            return 0;
            break;
    }
}


void DJIMotorControl(void)
{
    uint8_t group, num;
    float control_output;
    DJIMotor_t *motor;
    uint8_t power_control_count = 0;
    
    // 第一次遍历：计算控制输出
    for (size_t i = 0; i < idx; ++i) {
        motor = dji_motor_list[i];

        if (get_device_status(motor->offline_index)==1 || motor->stop_flag == MOTOR_STOP) {
            control_output = 0;
        }
        else {
            // 根据控制算法计算输出
            switch (motor->motor_settings.control_algorithm) 
            {
                case CONTROL_PID:
                    control_output = CalculatePIDOutput(motor);
                    break;
                    
                case CONTROL_LQR:
                    control_output = CalculateLQROutput(motor);
                    break;
                    
                default:
                    control_output = 0;
                    break;
            }
        }
        // 根据功率控制状态分别处理
        if(motor->motor_settings.PowerControlState == PowerControlState_ON) {
            PowerControlDji(motor, control_output);
            power_control_count++;
        } else {
            // 未开启功率控制的直接存储到本地数组
            motor_outputs[i] = control_output;
        }
    }

    // 只有存在需要功率控制的电机时才执行功率分配
    if(power_control_count > 0) {
        PowerControlDjiFinalize(dji_motor_list, idx);
    }
    // 第二次遍历：填充发送数据
    for (size_t i = 0; i < idx; ++i) {
        motor = dji_motor_list[i];
        group = motor->sender_group;
        num = motor->message_num;
        
        int16_t output;
        // 根据功率控制状态选择输出来源
        if(motor->motor_settings.PowerControlState == PowerControlState_ON) {
            output = GetPowerControlOutput(num);  // 从功率控制模块获取
        } else {
            output = GetRawMotorOutput(i);      // 从本地数组获取
        }
        
        // 填充发送数据
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(output >> 8);
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(output & 0x00ff);
    }
    // 发送CAN消息
    for (size_t i = 0; i < 10; ++i) {
        if (sender_enable_flag[i]) {
            CAN_SendMessage_hcan(sender_assignment[i].can_handle,
                               &sender_assignment[i].txconf,
                               sender_assignment[i].tx_buff,
                               &sender_assignment[i].tx_mailbox,
                               sender_assignment[i].txconf.DLC);
        }
    }
}






