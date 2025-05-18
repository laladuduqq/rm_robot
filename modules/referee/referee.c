#include "referee.h"
#include "bsp_uart.h"
#include "crc_rm.h"
#include "offline.h"
#include "referee_protocol.h"
#include "stm32f4xx_hal_def.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
 
#define LOG_TAG "referee"
#define LOG_LVL LOG_LVL_DBG
#include <elog.h>

static referee_info_t referee_info;			  // 裁判系统数据
uint8_t UI_Seq;
static osThreadId refereeTaskHandle;
static uint8_t referee_buf[2][128];

static void Sentry_Free_Revive(void);
void RefereeTask(const void *argument);
void JudgeReadData(uint8_t *buff);
void DeterminRobotID(void);
void RefereeInit(void)
{   
    OfflineDeviceInit_t offline_init = {
        .name = "referee",
        .timeout_ms = 1000,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 6,
        .enable = OFFLINE_ENABLE
    };
    
    referee_info.offline_index = offline_device_register(&offline_init);

    UART_Device_init_config uart6_cfg = {
        .huart = &huart6,
        .expected_len = 0,       // 不定长
        .rx_buf_size = 1024,
        .rx_buf = (uint8_t (*)[2])referee_buf,
        .rx_mode = UART_MODE_DMA,
        .tx_mode = UART_MODE_DMA,
        .timeout = 1000,
        .rx_complete_cb = NULL,
        .cb_type = UART_CALLBACK_EVENT,
        .event_flag = UART_RX_DONE_EVENT
    };
    UART_Device* uart6 = UART_Init(&uart6_cfg);
    referee_info.uart_device = uart6;

    osThreadDef(refereeTask, RefereeTask, osPriorityHigh, 0, 256);
    refereeTaskHandle = osThreadCreate(osThread(refereeTask), NULL);
}

void RefereeTask(const void *argument)
{ 
    UNUSED(argument);
    for (;;) 
    {
        while(1) {
            // 等待接收事件
            uint32_t flags = xEventGroupWaitBits(
                referee_info.uart_device->rx_event, 
                referee_info.uart_device->event_flag, 
                pdTRUE,  // 自动清除标志位
                pdFALSE,  // 等待所有位
                portMAX_DELAY
            );
            if(flags & UART_RX_DONE_EVENT) 
            {
                JudgeReadData(*referee_info.uart_device->rx_buf);
            }
        }
    }
}

 /**
  * @brief  读取裁判数据,中断中读取保证速度
  * @param  buff: 读取到的裁判系统原始数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
void JudgeReadData(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    if (buff == NULL)	   // 空数据包，则不作任何处理
        return;
 
    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
 
    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        // 帧头CRC8校验
         if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == RM_TRUE)
        {
            offline_device_update(referee_info.offline_index);
             // 统计一帧数据长度(byte),用于CR16校验
             judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
             // 帧尾CRC16校验
             if (Verify_CRC16_Check_Sum(buff, judge_length) == RM_TRUE)
             {
                 // 2个8位拼成16位int
                referee_info.CmdID = (buff[6] << 8 | buff[5]);
                 // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                 // 第8个字节开始才是数据 data=7
                switch (referee_info.CmdID)
                {
                 case ID_game_state: // 0x0001
                    memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
                    break;
                 case ID_game_result: // 0x0002
                    memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
                    break;
                 case ID_game_robot_survivors: // 0x0003
                    memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
                    break;
                 case ID_event_data: // 0x0101
                    memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
                    break;
                 case ID_supply_projectile_action: // 0x0102
                    memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
                    break;
                 case ID_game_robot_state: // 0x0201
                    memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
                    if (referee_info.init_flag == 0)
                    {
                        DeterminRobotID();
                        referee_info.init_flag=1;
                    }
                    break;
                 case ID_power_heat_data: // 0x0202
                    memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
                    break;
                 case ID_game_robot_pos: // 0x0203
                    memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
                    break;
                 case ID_buff_musk: // 0x0204
                    memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
                    break;
                 case ID_aerial_robot_energy: // 0x0205
                    memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
                    break;
                 case ID_robot_hurt: // 0x0206
                    memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
                    break;
                 case ID_shoot_data: // 0x0207
                    memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
                    break;
                 case ID_shoot_remaining:
                    memcpy(&referee_info.ext_shoot_remaing, (buff + DATA_Offset), LEN_shoot_remaing);
                    break;
                 case ID_rfid_status:
                    memcpy(&referee_info.rfid_status, (buff + DATA_Offset), LEN_rfid_status);
                    break;
                 case ID_ground_robot_position :
                    memcpy(&referee_info.ground_robot_position, (buff + DATA_Offset), LEN_ground_robot_position);
                    break;
                 case ID_sentry_info:
                    memcpy(&referee_info.sentry_info, (buff + DATA_Offset), LEN_sentry_info);
                    if (referee_info.sentry_info.sentry_can_free_revive == 1){Sentry_Free_Revive();}
                    break;
                }
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
        { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
             JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
        }
    }
}

void RefereeSend(uint8_t *send, uint16_t tx_len){
    UART_Send(referee_info.uart_device, send, tx_len);
}
 
void DeterminRobotID(void)
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_info.referee_id.Robot_ID = referee_info.GameRobotState.robot_id;
    referee_info.referee_id.Cilent_ID = 0x0100 + referee_info.referee_id.Robot_ID; // 计算客户端ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}
void Sentry_Free_Revive(void)
{
    Communicate_SendData_t Communicate_SendData;

	uint8_t temp_datalength = Interactive_Data_LEN_Head + Sentinel_Autonomous_Instructions_LEN; // 计算交互数据长度

	Communicate_SendData.FrameHeader.SOF = REFEREE_SOF;
	Communicate_SendData.FrameHeader.DataLength = temp_datalength;
	Communicate_SendData.FrameHeader.Seq = UI_Seq;
	Communicate_SendData.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&Communicate_SendData, LEN_CRC8, 0xFF);

	Communicate_SendData.CmdID = ID_student_interactive;

	Communicate_SendData.datahead.data_cmd_id = Sentinel_Autonomous_Instructions;
	Communicate_SendData.datahead.receiver_ID = 0x8080; // 发送给裁判系统
	Communicate_SendData.datahead.sender_ID = referee_info.referee_id.Robot_ID; // 发送者ID

    Communicate_SendData.Data.data[0] = 0x01; // 1:免费复活

	Communicate_SendData.frametail = Get_CRC16_Check_Sum((uint8_t *)&Communicate_SendData, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

	RefereeSend((uint8_t *)&Communicate_SendData, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送

	UI_Seq++; // 包序号+1
}
