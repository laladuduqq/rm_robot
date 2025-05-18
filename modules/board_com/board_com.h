#ifndef __BOARD_COM_H
#define __BOARD_COM_H

#include "bsp_can.h"
#include "offline.h"
#include "robotdef.h"

typedef struct
{
    Can_Device *candevice;
    uint8_t offlinemanage_index;
    #if defined(SENTRY_MODE)
        #if defined(GIMBAL_BOARD)
        Chassis_referee_Upload_Data_s Chassis_Upload_Data;
        #else 
        Chassis_Ctrl_Cmd_s Chassis_Ctrl_Cmd;
        #endif 
    #endif 
} board_com_t;


typedef struct
{
    Can_Device_Init_Config_s Can_Device_Init_Config;
    OfflineDeviceInit_t offline_manage_init;
} board_com_init_t;


board_com_t *board_com_init(board_com_init_t* board_com_init);
void board_send(void *data);
void *BoardRead(void);
void board_recv(const CAN_HandleTypeDef* hcan,const  uint32_t rx_id);

#endif // CAN_COMMON_H
