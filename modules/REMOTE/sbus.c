#include "sbus.h"
#include "bsp_uart.h"
#include "offline.h"
#include "usart.h"
#include <stdint.h>

#define LOG_TAG  "sbus"
#include "elog.h"

static SBUS_CH_Struct sbus;
static uint8_t sbus_buf[2][30];

void uart3_rx_callback(uint8_t *buf, uint16_t len)
{ 
    if(len == 25 && buf[0] == 0x0F && buf[24] == 0x00)
    {
        sbus.CH1      = ((int16_t)buf[1] >> 0 | ((int16_t)buf[2] << 8)) & 0x07FF;
        sbus.CH2      = ((int16_t)buf[2] >> 3 | ((int16_t)buf[3] << 5)) & 0x07FF;
        sbus.CH3      = ((int16_t)buf[3] >> 6 | ((int16_t)buf[4] << 2) | (int16_t)buf[5] << 10) & 0x07FF;
        sbus.CH4      = ((int16_t)buf[5] >> 1 | ((int16_t)buf[6] << 7)) & 0x07FF;
        sbus.CH5      = ((int16_t)buf[6] >> 4 | ((int16_t)buf[7] << 4)) & 0x07FF;
        sbus.CH6      = ((int16_t)buf[7] >> 7 | ((int16_t)buf[8] << 1) | (int16_t)buf[9] << 9) & 0x07FF;
        sbus.CH7      = ((int16_t)buf[9] >> 2 | ((int16_t)buf[10] << 6)) & 0x07FF;
        sbus.CH8      = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3)) & 0x07FF;
        sbus.CH9      = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8)) & 0x07FF;
        sbus.CH10     = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5)) & 0x07FF;
        sbus.CH11     = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2) | (int16_t)buf[16] << 10) & 0x07FF;
        sbus.CH12     = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7)) & 0x07FF;
        sbus.CH13     = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4)) & 0x07FF;
        sbus.CH14     = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1) | (int16_t)buf[20] << 9) & 0x07FF;
        sbus.CH15     = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6)) & 0x07FF;
        sbus.CH16     = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3)) & 0x07FF;
        sbus.ConnectState = buf[23];
        if (sbus.ConnectState == 0x00)
        {
            offline_device_update(sbus.offline_index);
        }
    }
}

SBUS_CH_Struct* Get_SBUS_Data(void)
{
    return &sbus;
}

void Remote_init(void){
    OfflineDeviceInit_t offline_init = {
        .name = "sbus",
        .timeout_ms = 100,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 0,
        .enable = OFFLINE_ENABLE,
    };
    sbus.offline_index = offline_device_register(&offline_init);

    UART_Device_init_config uart3_cfg = {
        .huart = &huart3,
        .expected_len = 25,      
        .rx_buf = (uint8_t (*)[2])sbus_buf,
        .rx_buf_size = 30,
        .rx_mode = UART_MODE_DMA,
        .tx_mode = UART_MODE_BLOCKING,
        .timeout = 1000,
        .rx_complete_cb = uart3_rx_callback, 
        .cb_type = UART_CALLBACK_DIRECT,
        .event_flag = 0x01,
    };
    UART_Device *remote = UART_Init(&uart3_cfg);
    if (remote != NULL)
    {
        sbus.uart_device = remote;
    }
    else
    {
        log_e("init uart failed");
    }
    
    log_i("sbus init success!");
}
