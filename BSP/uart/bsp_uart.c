#include "bsp_uart.h"
#include "stm32f4xx_hal_def.h"
#include "task.h"
#include "portable.h"
#include "stm32f4xx_hal_uart.h"
#include <string.h>

#define LOG_TAG "bsp_uart"
#include "elog.h"

static UART_Instance *registered_instances[UART_MAX_INSTANCE_NUM] = {NULL};

// 私有函数声明
static void Start_Rx(UART_Instance *inst);
static void Process_Rx_Complete(UART_Instance *inst, uint16_t Size);

UART_Instance* UART_Init(UART_Config *config) {
    // 检查实例是否已存在
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(registered_instances[i] && registered_instances[i]->huart == config->huart)
        {   log_w("UART instance already exists for huart: 0x%p", (void*)config->huart);
            return NULL;
        }
    }

    UART_Instance *inst = pvPortMalloc(sizeof(UART_Instance)) ;
    memset(inst, 0, sizeof(UART_Instance)); 

    // 初始化参数
    inst->huart = config->huart;
    inst->expected_len = config->expected_len;
    inst->rx_mode = config->rx_mode;
    inst->tx_mode = config->tx_mode;
    inst->timeout = config->timeout;
    inst->rx_complete_cb = config->rx_complete_cb;
    inst->cb_type = config->cb_type;
    inst->event_flag = config->event_flag; //注意只能单一位（0x01,0x02）,不能复合位

    //初始化事件组
    inst->rx_event = xEventGroupCreate();

    //初始化互斥量
    inst->tx_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(inst->tx_mutex); // 初始化为可用状态
    
    // 注册实例
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(!registered_instances[i]){
            registered_instances[i] = inst;
            log_i("UART instance registered at index %d", i);
            break;
        }
    }

    // 启动接收
    Start_Rx(inst);
    log_i("UART initialized successfully. Mode: RX=%d, TX=%d", inst->rx_mode, inst->tx_mode);
    return inst;
}

HAL_StatusTypeDef UART_Send(UART_Instance *inst, uint8_t *data, uint16_t len) {
    // 检查互斥量
    if(xSemaphoreTake(inst->tx_mutex,inst->timeout) != pdTRUE) return HAL_BUSY;
    switch(inst->tx_mode){
        case UART_TX_MODE_BLOCKING:
            HAL_UART_Transmit(inst->huart, data, len, inst->timeout);
            xSemaphoreGive(inst->tx_mutex);
            break;
            
        case UART_TX_MODE_IT:
            HAL_UART_Transmit_IT(inst->huart, data, len);
            break;
            
        case UART_TX_MODE_DMA:
            HAL_UART_Transmit_DMA(inst->huart, data, len);
            break;
    }
    return HAL_OK;
}

// 接收完成回调（包括空闲中断）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        UART_Instance *inst = registered_instances[i];
        if(inst && inst->huart == huart){
            Process_Rx_Complete(inst, Size);
            Start_Rx(inst); // 重启接收
            break;
        }
    }
}

//发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        UART_Instance *inst = registered_instances[i];
        if(inst && inst->huart == huart){
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(inst->tx_mutex, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken){portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}// 必要时触发上下文切换
            break;
        }
    }
}


// 错误处理回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        UART_Instance *inst = registered_instances[i];
        if(inst && inst->huart == huart){      
            log_e("UART error occurred, restarting RX");      
            HAL_UART_AbortReceive(huart);
            Start_Rx(inst);
            break;
        }
    }
}

// 私有函数实现
static void Start_Rx(UART_Instance *inst) {
    uint8_t next_buf = !inst->rx_active_buf;
    uint16_t max_len = sizeof(inst->rx_buf[0]);

    switch(inst->rx_mode){
        case UART_RX_MODE_BLOCKING:
            HAL_UART_Receive(inst->huart, inst->rx_buf[next_buf], 
                inst->expected_len ? inst->expected_len : max_len, inst->timeout);
            break;
            
        case UART_RX_MODE_IT:
            HAL_UARTEx_ReceiveToIdle_IT(inst->huart, inst->rx_buf[next_buf], 
                inst->expected_len ? inst->expected_len : max_len);
            break;
            
        case UART_RX_MODE_DMA:
            HAL_UARTEx_ReceiveToIdle_DMA(inst->huart, inst->rx_buf[next_buf], 
                inst->expected_len ? inst->expected_len : max_len);
            break;
    }
    
    inst->rx_active_buf = next_buf;
}

static void Process_Rx_Complete(UART_Instance *inst, uint16_t Size) {
    // 计算实际接收长度
    if(inst->expected_len == 0){
        if(inst->rx_mode == UART_RX_MODE_DMA){
            Size = sizeof(inst->rx_buf[0]) - __HAL_DMA_GET_COUNTER(inst->huart->hdmarx);
        }
    }
    inst->rx_len = Size;

    // 触发回调
    uint8_t *data = inst->rx_buf[!inst->rx_active_buf]; // 使用非活动缓冲区
    
    if(inst->cb_type == UART_CALLBACK_DIRECT){
        if(inst->rx_complete_cb)
            inst->rx_complete_cb(data, Size);
    }
    else{
        if(inst->rx_event)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xEventGroupSetBitsFromISR(inst->rx_event, 
                                    inst->event_flag,
                                    &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken){portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}// 必要时触发上下文切换
        }
    }
}

void UART_Deinit(UART_Instance *inst) {
    HAL_UART_Abort(inst->huart);
    vPortFree(inst);
}
