#ifndef __REFEREE_H
#define __REFEREE_H

#include "bsp_uart.h"
#include "offline.h"
#include "referee_protocol.h"
#include <stdint.h>

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	ext_shoot_remaing_t ext_shoot_remaing;				   // 0x0208
	rfid_status_t rfid_status;							   // 0x0209
	ground_robot_position_t ground_robot_position;         // 0x020b
	sentry_info_t sentry_info;                             // 0x020d
	map_data_t map_data;                                   // 0x0307

	// 自定义交互数据的接收
	Communicate_ReceiveData_t ReceiveData;

	uint8_t init_flag;
	uint8_t offline_index; // 离线检测索引

	UART_Device *uart_device; // UART实例
} referee_info_t;

#pragma pack()

extern uint8_t UI_Seq;

void RefereeInit(void);
void RefereeSend(uint8_t *send, uint16_t tx_len);


#endif 
