#ifndef __SBUS_H
#define __SBUS_H
#include "bsp_uart.h"
#include <stdint.h>


#define SBUS_CHX_BIAS 1024
#define SBUS_CHX_UP   1807
#define SBUS_CHX_DOWN 240

typedef struct
{
    uint16_t CH1;//1通道
    uint16_t CH2;//2通道
    uint16_t CH3;//3通道
    uint16_t CH4;//4通道
    uint16_t CH5;//5通道
    uint16_t CH6;//6通道
    uint16_t CH7;//7通道
    uint16_t CH8;//8通道
    uint16_t CH9;//9通道
    uint16_t CH10;//10通道
    uint16_t CH11;//11通道
    uint16_t CH12;//12通道
    uint16_t CH13;//13通道
    uint16_t CH14;//14通道
    uint16_t CH15;//15通道
    uint16_t CH16;//16通道
    uint8_t ConnectState;   //连接的标志

    uint8_t offline_index; // 离线索引
    UART_Device *uart_device; // UART实例
}SBUS_CH_Struct;

SBUS_CH_Struct* Get_SBUS_Data(void);
void Remote_init(void);

#endif // SBUS_H
