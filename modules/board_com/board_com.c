#include "board_com.h"
#include "offline.h"
#include "portable.h"
#include "robotdef.h"
#include "stm32f4xx_hal_def.h"
#include <string.h>


#define LOG_TAG              "board_com"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

uint8_t float_to_uint8(float f);
float uint8_to_float(uint8_t u);

static board_com_t *board_com_list[1]= {NULL}; //就一个实例
board_com_t *board_com_init(board_com_init_t* board_com_init)
{
#ifndef ONE_BOARD
    board_com_t *board_com = (board_com_t *)pvPortMalloc(sizeof(board_com_t));
    if (board_com == NULL) {
        log_e("Failed to allocate memory for board_com\n");
        return NULL;
    }
    memset(board_com, 0, sizeof(board_com_t));

    // 初始化板间通讯的掉线检测
    board_com->offlinemanage_index = offline_device_register(&board_com_init->offline_manage_init);

    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = board_com_init->Can_Device_Init_Config.can_handle,
        .tx_id = board_com_init->Can_Device_Init_Config.tx_id,
        .rx_id = board_com_init->Can_Device_Init_Config.rx_id,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
        .can_callback = board_recv
    };
    // 注册 CAN 设备并获取引用
    Can_Device *can_dev = BSP_CAN_Device_Init(&can_config);
    if (can_dev == NULL) {
        log_e("Failed to initialize CAN device for board_com");
        vPortFree(board_com);
        return NULL;
    }
    board_com->candevice = can_dev;
    if (board_com->candevice->tx_id == GIMBAL_ID)
    {
        log_w("register deivce is gimbal");
    }
    else
    {
        log_w("register deivce is chassis");
    }
    
    // 记录实例
    board_com_list[0] = board_com;
    return board_com;
#else
    return NULL;
#endif 
    
}

void board_recv(const CAN_HandleTypeDef* hcan,const  uint32_t rx_id)
{
    UNUSED(hcan);
    UNUSED(rx_id);
#ifndef ONE_BOARD
    #if defined(HERO_MODE) || defined(ENGINEER_MODE) || defined (INFANTRY_MODE) || defined (SENTRY_MODE)
        offline_device_update(board_com_list[0]->offlinemanage_index);
        uint8_t *rxbuff = board_com_list[0]->candevice->rx_buff;  
        #ifdef GIMBAL_BOARD
            board_com_list[0]->Chassis_Upload_Data.Robot_Color = (rxbuff[0] >> 7) & 0x01;
            // projectile_allowance_17mm (从0-127还原到0-1000范围)
            uint8_t compressed_projectile = rxbuff[0] & 0x7F;
            board_com_list[0]->Chassis_Upload_Data.projectile_allowance_17mm = (compressed_projectile * 1000) / 127;
            board_com_list[0]->Chassis_Upload_Data.power_management_shooter_output = (rxbuff[1] >> 7) & 0x01;
            board_com_list[0]->Chassis_Upload_Data.current_hp_percent = ((rxbuff[1] & 0x7F) * 400) / 100;
            // outpost_HP
            uint16_t outpost = (rxbuff[2] << 3) | ((rxbuff[3] >> 5) & 0x07);
            board_com_list[0]->Chassis_Upload_Data.outpost_HP = (outpost * 1500) / 2047;
            // base_HP
            uint16_t base = ((rxbuff[3] & 0x1F) << 8) | rxbuff[4];
            board_com_list[0]->Chassis_Upload_Data.base_HP = (base * 5000) / 8191;
            // game_progess
            board_com_list[0]->Chassis_Upload_Data.game_progess = ((rxbuff[5] >> 5) & 0x07) + 1;
            // game_time
            board_com_list[0]->Chassis_Upload_Data.game_time = ((rxbuff[5] & 0x1F) << 3) | ((rxbuff[6] >> 5) & 0x07);
        #else
            board_com_list[0]->Chassis_Ctrl_Cmd.vx = ((int16_t)(rxbuff[0] << 8) | rxbuff[1]) ;
            board_com_list[0]->Chassis_Ctrl_Cmd.vy = ((int16_t)(rxbuff[2] << 8) | rxbuff[3]) ;
            board_com_list[0]->Chassis_Ctrl_Cmd.offset_angle = ((int16_t)(rxbuff[4] << 8) | rxbuff[5]) / 100.0f;
            board_com_list[0]->Chassis_Ctrl_Cmd.wz = ((int8_t)(rxbuff[6]/10.0f));
            board_com_list[0]->Chassis_Ctrl_Cmd.chassis_mode = rxbuff[7];
        #endif 
    #endif  
#endif     
    
}

void *BoardRead(void)
{
    #if defined(GIMBAL_BOARD)
        return &board_com_list[0]->Chassis_Upload_Data;
    #else
        return &board_com_list[0]->Chassis_Ctrl_Cmd;
    #endif
    return NULL; // 默认返回 NULL
}

/*
   target_id 发送者的id
*/
void board_send(void *data)
{
#ifndef ONE_BOARD
    #if defined (GIMBAL_BOARD)
        Chassis_Ctrl_Cmd_s *cmd = NULL;
        cmd = (Chassis_Ctrl_Cmd_s *)data; 
        board_com_list[0]->candevice->tx_buff[0] = ((int16_t)(cmd->vx               ) >> 8) & 0xFF;      
        board_com_list[0]->candevice->tx_buff[1] = ((int16_t)(cmd->vx               ) & 0xFF);            
        board_com_list[0]->candevice->tx_buff[2] = ((int16_t)(cmd->vy               ) >> 8) & 0xFF;       
        board_com_list[0]->candevice->tx_buff[3] = ((int16_t)(cmd->vy               ) & 0xFF);             
        board_com_list[0]->candevice->tx_buff[4] = ((int16_t)(cmd->offset_angle *100) >> 8) & 0xFF;   
        board_com_list[0]->candevice->tx_buff[5] = ((int16_t)(cmd->offset_angle *100) & 0xFF);    
        board_com_list[0]->candevice->tx_buff[6] = ((int8_t )(cmd->wz           *10));
        board_com_list[0]->candevice->tx_buff[7] = cmd->chassis_mode;
    #else
        Chassis_referee_Upload_Data_s *cmd = NULL;
        cmd = (Chassis_referee_Upload_Data_s *)data;
        
        // 处理弹量: 负数置0，超过1000限制为1000，然后映射到0-127范围
        int16_t projectile = cmd->projectile_allowance_17mm;
        if(projectile < 0) projectile = 0;
        if(projectile > 1000) projectile = 1000;
        uint8_t compressed_projectile = (projectile * 127) / 1000;
    
        // 第1字节: Robot_Color(1位) + projectile_allowance_17mm前7位
        board_com_list[0]->candevice->tx_buff[0] = (cmd->Robot_Color & 0x01) << 7; 
        board_com_list[0]->candevice->tx_buff[0] |= compressed_projectile & 0x7F;    
        // 第2字节: power_management_shooter_output(1位) + current_hp_percent(7位)
        board_com_list[0]->candevice->tx_buff[1] = (cmd->power_management_shooter_output & 0x01) << 7;
        board_com_list[0]->candevice->tx_buff[1] |= (cmd->current_hp_percent * 100 / 400) & 0x7F;
        // 第3-4字节: outpost_HP(11位)
        uint16_t outpost = (cmd->outpost_HP * 2047 / 1500) & 0x07FF;
        board_com_list[0]->candevice->tx_buff[2] = outpost >> 3;
        board_com_list[0]->candevice->tx_buff[3] = (outpost & 0x07) << 5;
        // 第4-5字节: base_HP(13位)
        uint16_t base = (cmd->base_HP * 8191 / 5000) & 0x1FFF;
        board_com_list[0]->candevice->tx_buff[3] |= (base >> 8) & 0x1F;
        board_com_list[0]->candevice->tx_buff[4] = base & 0xFF;
        // 第6字节: game_progess(3位) + game_time高6位
        board_com_list[0]->candevice->tx_buff[5] = ((cmd->game_progess - 1) & 0x07) << 5;
        board_com_list[0]->candevice->tx_buff[5] |= (cmd->game_time >> 3) & 0x1F;  
        // 第7字节: game_time低3位 + 预留5位
        board_com_list[0]->candevice->tx_buff[6] = (cmd->game_time & 0x07) << 5;
    #endif
    if (board_com_list[0]->candevice->can_handle != NULL) 
    {
        CAN_SendMessage(board_com_list[0]->candevice,board_com_list[0]->candevice->txconf.DLC);
    }
#endif 
}


