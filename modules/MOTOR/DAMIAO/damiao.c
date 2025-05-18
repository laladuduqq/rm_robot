#include "damiao.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "dwt.h"
#include "offline.h"
#include <stdint.h>
#include <string.h>

static uint8_t idx;
static DMMOTOR_t *dmm_motor_list[DM_MOTOR_CNT]= {NULL}; // 会在motor_task任务中遍历该指针数组进行pid计算

#define LOG_TAG              "dm"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 将弧度转换为角度
 * 
 * @param rad 弧度值
 * @return float 转换后的角度值
 */
float rad_to_deg(float rad)
{
    return rad * (180.0f / PI);
}

/**
 * @brief 将角度转换为弧度
 * 
 * @param deg 角度值
 * @return float 转换后的弧度值
 */
float deg_to_rad(float deg)
{
    return deg * (PI / 180.0f);
}

void DMMotorSetMode(DMMotor_Mode_e cmd, DMMOTOR_t *motor)
{

    motor->can_device->tx_buff[0]=0xff;
    motor->can_device->tx_buff[1]=0xff;
    motor->can_device->tx_buff[2]=0xff;
    motor->can_device->tx_buff[3]=0xff;
    motor->can_device->tx_buff[4]=0xff;
    motor->can_device->tx_buff[5]=0xff;
    motor->can_device->tx_buff[6]=0xff;
    motor->can_device->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    motor->can_device->txconf.StdId = motor->can_device->tx_id+motor->DMMotor_Mode_type;
    motor->can_device->txconf.DLC = 8;
    CAN_SendMessage(motor->can_device,motor->can_device->txconf.DLC);
}

void mit_ctrl(DMMOTOR_t *motor, float pos, float vel,float kp, float kd, float torq)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	
	pos_tmp = float_to_uint(pos,  DM_P_MIN,  DM_P_MAX,  16);
	vel_tmp = float_to_uint(vel,  DM_V_MIN,  DM_V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   DM_KP_MIN, DM_KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   DM_KD_MIN, DM_KD_MAX, 12);
	tor_tmp = float_to_uint(torq, DM_T_MIN,  DM_T_MAX,  12);

    motor->can_device->txconf.StdId = motor->can_device->tx_id + MIT_MODE;
    motor->can_device->txconf.DLC = 8;
	motor->can_device->tx_buff[0] = (pos_tmp >> 8);
	motor->can_device->tx_buff[1] = pos_tmp;
	motor->can_device->tx_buff[2] = (vel_tmp >> 4);
	motor->can_device->tx_buff[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	motor->can_device->tx_buff[4] = kp_tmp;
	motor->can_device->tx_buff[5] = (kd_tmp >> 4);
	motor->can_device->tx_buff[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	motor->can_device->tx_buff[7] = tor_tmp;
	
	CAN_SendMessage(motor->can_device,motor->can_device->txconf.DLC);
}


void pos_speed_ctrl(DMMOTOR_t *motor, float pos_degree, float vel)
{
	uint8_t *pbuf, *vbuf;

    float pos_rad = deg_to_rad(pos_degree);
    LIMIT_MIN_MAX(pos_rad, DM_P_MIN, DM_P_MAX);
	
	motor->can_device->txconf.StdId = motor->can_device->tx_id + POS_MODE;
    motor->can_device->txconf.DLC = 8;

	pbuf=(uint8_t*)&pos_rad;
	vbuf=(uint8_t*)&vel;
	
	motor->can_device->tx_buff[0] = *pbuf;
	motor->can_device->tx_buff[1] = *(pbuf+1);
	motor->can_device->tx_buff[2] = *(pbuf+2);
	motor->can_device->tx_buff[3] = *(pbuf+3);
	motor->can_device->tx_buff[4] = *vbuf;
	motor->can_device->tx_buff[5] = *(vbuf+1);
	motor->can_device->tx_buff[6] = *(vbuf+2);
	motor->can_device->tx_buff[7] = *(vbuf+3);
	
	CAN_SendMessage(motor->can_device,motor->can_device->txconf.DLC);
}


void speed_ctrl(DMMOTOR_t *motor, float vel)
{
	
	uint8_t *vbuf;
	motor->can_device->txconf.StdId = motor->can_device->tx_id + SPEED_MODE;
    motor->can_device->txconf.DLC = 4;
	
	vbuf=(uint8_t*)&vel;
	
	motor->can_device->tx_buff[0] = *vbuf;
	motor->can_device->tx_buff[1] = *(vbuf+1);
	motor->can_device->tx_buff[2] = *(vbuf+2);
	motor->can_device->tx_buff[3] = *(vbuf+3);
	
	CAN_SendMessage(motor->can_device,motor->can_device->txconf.DLC);
}

void DMMotorDecode(const CAN_HandleTypeDef* hcan,const  uint32_t rx_id)
{
    for (uint8_t i = 0; i < idx; i++) {
        if (dmm_motor_list[i] 
            &&dmm_motor_list[i]->can_device->can_handle == hcan 
            && dmm_motor_list[i]->can_device->rx_id == rx_id)
        {
            uint8_t *rxbuff = dmm_motor_list[i]->can_device->rx_buff;  
            offline_device_update(dmm_motor_list[i]->offline_index);
            dmm_motor_list[i]->dt = DWT_GetDeltaT(&dmm_motor_list[i]->feed_cnt);

            uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量

            dmm_motor_list[i]->measure.last_position = dmm_motor_list[i]->measure.position;
            tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
            dmm_motor_list[i]->measure.position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
            dmm_motor_list[i]->measure.position = rad_to_deg(dmm_motor_list[i]->measure.position);

            tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
            dmm_motor_list[i]->measure.velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
            dmm_motor_list[i]->measure.velocity = rad_to_deg(dmm_motor_list[i]->measure.velocity);

            tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
            dmm_motor_list[i]->measure.torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

            dmm_motor_list[i]->measure.T_Mos = (float)rxbuff[6];
            dmm_motor_list[i]->measure.T_Rotor = (float)rxbuff[7];

            // 解析错误码
            uint8_t error_code = (rxbuff[0] >> 4) & 0x0F;
            dmm_motor_list[i]->measure.id = rxbuff[0] & 0x0F;
            dmm_motor_list[i]->measure.Error_Code = (error_code >= 0x08 && error_code <= 0x0E) ? (DMMotorError_t)error_code : DM_NO_ERROR;
        }
    }
}

void DMMotorCaliEncoder(DMMOTOR_t *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    osDelay(10);
}


DMMOTOR_t *DMMotorInit(Motor_Init_Config_s *config,uint32_t DM_Mode_type){
    DMMOTOR_t *DMMotor = (DMMOTOR_t *)pvPortMalloc(sizeof(DMMOTOR_t));
    if (DMMotor == NULL) {
        log_e("Failed to allocate memory for DJIMotor\n");
        return NULL;
    }
    memset(DMMotor, 0, sizeof(DMMOTOR_t));

    // motor basic setting 电机基本设置
    DMMotor->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    DMMotor->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等
    DMMotor->DMMotor_Mode_type =DM_Mode_type;

    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = config->can_init_config.can_handle,
        .tx_id = config->can_init_config.tx_id,
        .rx_id = config->can_init_config.rx_id,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
        .can_callback = DMMotorDecode
    };
    // 注册 CAN 设备并获取引用
    Can_Device *can_dev = BSP_CAN_Device_Init(&can_config);
    if (can_dev == NULL) {
        log_e("Failed to initialize CAN device for DJI motor");
        vPortFree(DMMotor);
        return NULL;
    }
    // 保存设备指针
    DMMotor->can_device = can_dev;

    DMMotor->motor_settings.control_algorithm = config->controller_setting_init_config.control_algorithm;
    switch (config->controller_setting_init_config.control_algorithm) {
        case CONTROL_PID:
            // motor controller init 电机控制器初始化
            PIDInit(&DMMotor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
            PIDInit(&DMMotor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
            PIDInit(&DMMotor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
            DMMotor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
            DMMotor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
            DMMotor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
            DMMotor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
            break;
        case CONTROL_LQR:
            LQRInit(&DMMotor->motor_controller.lqr, &config->controller_param_init_config.lqr_config);
            DMMotor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
            DMMotor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
            DMMotor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
            DMMotor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
            break;
        case CONTROL_OTHER:
            // 未来添加其他控制算法的初始化
            break;
    }
    
    //掉线检测
    DMMotor->offline_index =offline_device_register(&config->offline_device_motor);
    // 记录电机实例
    dmm_motor_list[idx++] =DMMotor;

    DMMotor->measure.Error_Code = DM_NO_ERROR;
    DMMotorSetMode(DM_CMD_CLEAR_ERROR,DMMotor);

    return DMMotor;
}

void DMMotorSetRef(DMMOTOR_t *motor, float ref)
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

void DMMotorEnable(DMMOTOR_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
}

void DMMotorStop(DMMOTOR_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMOTOR_t *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void DMMotorcontrol(void){
    DMMOTOR_t *motor =NULL;
    float state0,state1;

    for (size_t i = 0; i < idx; ++i){
        motor = dmm_motor_list[i];
        if (!get_device_status(motor->offline_index) || motor->stop_flag == MOTOR_STOP) // 如果电机处于离线状态,发送0 若该电机处于停止状态,直接将buff置零
        {
            DMMotorSetMode(DM_CMD_RESET_MODE, motor);
        }
        else 
        {
            switch (motor->DMMotor_Mode_type)
            {
                case MIT_MODE:
                {
                    if(motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                    {
                        motor->motor_controller.lqr.feedbackreverseflag =1;
                    }
                    if (motor->motor_settings.angle_feedback_source == OTHER_FEED) 
                    {state0 = *motor->motor_controller.other_angle_feedback_ptr;}
                    else 
                    {state0 = motor->measure.position;} // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃

                    if (motor->motor_settings.speed_feedback_source == OTHER_FEED) 
                    {state1 = *motor->motor_controller.other_speed_feedback_ptr;}
                    else  
                    {state1 = motor->measure.velocity;}
                    
                    float torque = LQRCalculate(&motor->motor_controller.lqr, state0, state1,motor->motor_controller.lqr_ref);
                    mit_ctrl(motor, 0, 0, 0, 0, torque);
                    break;
                }
                case POS_MODE:
                {               
                    float pid_ref = 0.0f;             // 电机PID测量值和设定值
                    if (motor->stop_flag == MOTOR_STOP) { DMMotorSetMode(DM_CMD_RESET_MODE, motor); };
                    pos_speed_ctrl(motor, pid_ref, PI);
                    break;
                }
                case SPEED_MODE:
                {
                    float pid_ref = 0.0f;             // 电机PID测量值和设定值
                    LIMIT_MIN_MAX(pid_ref, DM_V_MIN, DM_V_MAX);
                    if (motor->stop_flag == MOTOR_STOP) { DMMotorSetMode(DM_CMD_RESET_MODE, motor); };
                    speed_ctrl(motor, pid_ref);
                    break;
                }
            }
        }
    }
}
