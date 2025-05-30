#ifndef __VCOM_H__
#define __VCOM_H__

#include <stdint.h>
#pragma pack(1)
struct Sentry_Send_s
{
  uint8_t header;
  uint8_t Robot_Color;
  uint16_t projectile_allowance_17mm;  //剩余发弹量
  uint8_t power_management_shooter_output; // 功率管理 shooter 输出
  uint16_t current_hp_percent; // 机器人当前血量百分比
  uint16_t outpost_HP;     //前哨站血量
  uint16_t base_HP;        //基地血量
  uint8_t game_progess;
  uint16_t game_time;
  uint8_t mode;
  float roll;
  float pitch;
  float yaw;
  uint8_t end ;
};

struct Nav_Recv_s
{
  uint8_t header; 
  uint8_t nav_state;
  float vx;
  float vy;
  float wz;
  uint8_t tail;
};

struct Vision_Recv_s
{
  uint8_t header ;
  uint8_t fire_advice;
  float pitch;
  float yaw;
  float distance;
  uint8_t check_byte;
  uint8_t end ;
};

#pragma pack() 

void vcom_init(void);
void Get_data(struct Nav_Recv_s *nav_recv_user,struct Vision_Recv_s *vision_recv_user);

#endif /* __VCOM_H__ */
