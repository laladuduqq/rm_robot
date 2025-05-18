#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include <stdbool.h>
#include "semphr.h"


#define UART_MAX_INSTANCE_NUM 3 
#define UART_RX_DONE_EVENT (0x01 << 0) //接收完成事件
#define UART_DEFAULT_BUF_SIZE 32  // 添加默认最小缓冲区大小定义

// 模式选择
typedef enum {
    UART_MODE_BLOCKING,
    UART_MODE_IT,
    UART_MODE_DMA
} UART_Mode;

// 回调触发方式
typedef enum {
    UART_CALLBACK_DIRECT,
    UART_CALLBACK_EVENT
} UART_CallbackType;

// UART设备结构体
typedef struct {
    UART_HandleTypeDef *huart;
    
    // 接收相关
    uint8_t (*rx_buf)[2];    // 指向外部定义的双缓冲区
    uint16_t rx_buf_size;    // 缓冲区大小
    volatile uint8_t rx_active_buf;  // 当前活动缓冲区
    uint16_t rx_len;         // 接收数据长度
    uint16_t expected_len;   // 预期长度（0为不定长）

    //发送相关
    SemaphoreHandle_t tx_mutex;
    
    // 回调相关
    void (*rx_complete_cb)(uint8_t *data, uint16_t len);
    EventGroupHandle_t rx_event;
    uint32_t event_flag;
    UART_CallbackType cb_type;
    
    // 配置参数
    UART_Mode rx_mode;
    UART_Mode tx_mode;
    uint32_t timeout;
} UART_Device;

// 初始化配置结构体
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t (*rx_buf)[2];    // 外部缓冲区指针
    uint16_t rx_buf_size;    // 缓冲区大小
    uint16_t expected_len;
    UART_Mode rx_mode;
    UART_Mode tx_mode;
    uint32_t timeout;
    void (*rx_complete_cb)(uint8_t *data, uint16_t len);
    UART_CallbackType cb_type;
    uint32_t event_flag;
} UART_Device_init_config;

// 接口函数
UART_Device* UART_Init(UART_Device_init_config *config);
HAL_StatusTypeDef UART_Send(UART_Device *inst, uint8_t *data, uint16_t len);
void UART_Deinit(UART_Device *inst);

#endif /* __BSP_UART_H__ */
