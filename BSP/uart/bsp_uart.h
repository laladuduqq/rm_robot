#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include <stdbool.h>
#include "projdefs.h"
#include "semphr.h"


#define UART_MAX_INSTANCE_NUM 3
#define UART_RX_DONE_EVENT (0x01 << 0)

// 接收模式选择
typedef enum {
    UART_RX_MODE_BLOCKING,
    UART_RX_MODE_IT,
    UART_RX_MODE_DMA
} UART_RxMode;

// 发送模式选择
typedef enum {
    UART_TX_MODE_BLOCKING,
    UART_TX_MODE_IT,
    UART_TX_MODE_DMA
} UART_TxMode;

// 回调触发方式
typedef enum {
    UART_CALLBACK_DIRECT,
    UART_CALLBACK_EVENT
} UART_CallbackType;

// UART实例结构体
typedef struct {
    UART_HandleTypeDef *huart;
    
    // 接收相关
    uint8_t rx_buf[2][256]; // 双缓冲
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
    UART_RxMode rx_mode;
    UART_TxMode tx_mode;
    uint32_t timeout;
} UART_Instance;

// 初始化配置结构体
typedef struct {
    UART_HandleTypeDef *huart;
    uint16_t expected_len;
    UART_RxMode rx_mode;
    UART_TxMode tx_mode;
    uint32_t timeout;
    void (*rx_complete_cb)(uint8_t*,uint16_t);
    UART_CallbackType cb_type;
    uint32_t event_flag;
} UART_Config;

// 接口函数
UART_Instance* UART_Init(UART_Config *config);
HAL_StatusTypeDef UART_Send(UART_Instance *inst, uint8_t *data, uint16_t len);
void UART_Deinit(UART_Instance *inst);

#endif /* __BSP_UART_H__ */
