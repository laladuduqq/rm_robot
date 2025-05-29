#include "bsp_uart.h"
#include "stm32f4xx_hal_def.h"
#include "task.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>
#include <string.h>
#include "event_groups.h"

#define LOG_TAG "bsp_uart"
#include "elog.h"

static UART_Device registered_uart[UART_MAX_INSTANCE_NUM] = {0}; // 结构体数组
static bool uart_used[UART_MAX_INSTANCE_NUM] = {false};         // 添加使用状态标记
static uint8_t default_buf[UART_MAX_INSTANCE_NUM][2][UART_DEFAULT_BUF_SIZE];

// 私有函数声明
static void Start_Rx(UART_Device *inst);
static void Process_Rx_Complete(UART_Device *inst, uint16_t Size);

UART_Device* UART_Init(UART_Device_init_config *config) {
    // 检查实例是否已存在
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(uart_used[i] && registered_uart[i].huart == config->huart)
        {   
            log_w("UART instance already exists for huart: 0x%p", (void*)config->huart);
            return NULL;
        }
    }

    // 查找空闲槽位
    int free_index = -1;
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(!uart_used[i]){
            free_index = i;
            break;
        }
    }

    if(free_index == -1) {
        log_e("No free UART instance slot available");
        return NULL;
    }

    // 初始化参数
    UART_Device *inst = &registered_uart[free_index];
    memset(inst, 0, sizeof(UART_Device));

    inst->huart = config->huart;
    inst->huart = config->huart;
    
    // 修改初始化函数中的缓冲区配置部分
    if (config->rx_buf != NULL) {
        inst->rx_buf = (uint8_t (*)[2])config->rx_buf;
        inst->rx_buf_size = config->rx_buf_size;
    } else {
        inst->rx_buf = (uint8_t (*)[2])default_buf[free_index];  // 类型转换
        inst->rx_buf_size = UART_DEFAULT_BUF_SIZE;
    }
    inst->expected_len = config->expected_len;
    if (inst->expected_len > inst->rx_buf_size) 
    {
        inst->expected_len = inst->rx_buf_size; // 确保不超过缓冲区大小
    }
    inst->rx_mode = config->rx_mode;
    inst->tx_mode = config->tx_mode;
    inst->timeout = config->timeout;
    inst->rx_complete_cb = config->rx_complete_cb;
    inst->cb_type = config->cb_type;
    inst->event_flag = config->event_flag;

    //初始化事件组
    inst->rx_event = xEventGroupCreate();

    //初始化互斥量
    inst->tx_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(inst->tx_mutex);
    
    uart_used[free_index] = true;
    log_i("UART instance registered at index %d", free_index);

    // 启动接收
    Start_Rx(inst);
    log_i("UART initialized successfully. Mode: RX=%d, TX=%d", inst->rx_mode, inst->tx_mode);
    return inst;
}

HAL_StatusTypeDef UART_Send(UART_Device *inst, uint8_t *data, uint16_t len) {
    // 检查互斥量
    if(xSemaphoreTake(inst->tx_mutex,inst->timeout) != pdTRUE) return HAL_BUSY;
    switch(inst->tx_mode){
        case UART_MODE_BLOCKING:
            HAL_UART_Transmit(inst->huart, data, len, inst->timeout);
            xSemaphoreGive(inst->tx_mutex);
            break;
            
        case UART_MODE_IT:
            HAL_UART_Transmit_IT(inst->huart, data, len);
            break;
            
        case UART_MODE_DMA:
            HAL_UART_Transmit_DMA(inst->huart, data, len);
            break;
    }
    return HAL_OK;
}

void UART_Deinit(UART_Device *inst) {
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(&registered_uart[i] == inst){
            HAL_UART_Abort(inst->huart);
            vEventGroupDelete(inst->rx_event);
            vSemaphoreDelete(inst->tx_mutex);
            memset(inst, 0, sizeof(UART_Device));
            uart_used[i] = false;
            break;
        }
    }
}

/*--------------------------中断函数处理------------------------------------*/

// 接收完成回调（包括空闲中断）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(uart_used[i] && registered_uart[i].huart == huart){
            Process_Rx_Complete(&registered_uart[i], Size);
            Start_Rx(&registered_uart[i]); // 重启接收
            break;
        }
    }
}

//发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(uart_used[i] && registered_uart[i].huart == huart){
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(registered_uart[i].tx_mutex, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken){portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}
            break;
        }
    }
}


// 错误处理回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for(int i=0; i<UART_MAX_INSTANCE_NUM; i++){
        if(uart_used[i] && registered_uart[i].huart == huart){      
            log_e("UART error occurred, restarting RX");      
            HAL_UART_AbortReceive(huart);
            Start_Rx(&registered_uart[i]);
            break;
        }
    }
}



/*----------------------------------私有函数实现----------------------------*/
static void Start_Rx(UART_Device *inst) {
    uint8_t next_buf = !inst->rx_active_buf;
    uint16_t max_len = inst->rx_buf_size;

    switch(inst->rx_mode){
        case UART_MODE_BLOCKING:
            HAL_UART_Receive(inst->huart, (uint8_t*)&inst->rx_buf[next_buf], 
                inst->expected_len ? inst->expected_len : max_len, inst->timeout);
            break;
            
        case UART_MODE_IT:
            HAL_UARTEx_ReceiveToIdle_IT(inst->huart, (uint8_t*)&inst->rx_buf[next_buf], 
                inst->expected_len ? inst->expected_len : max_len);
            break;
            
        case UART_MODE_DMA:
            HAL_UARTEx_ReceiveToIdle_DMA(inst->huart, (uint8_t*)&inst->rx_buf[next_buf], 
                inst->expected_len ? inst->expected_len : max_len);
            break;
    }
    
    inst->rx_active_buf = next_buf;
}

static void Process_Rx_Complete(UART_Device *inst, uint16_t Size) {
    // 计算实际接收长度
    if(inst->expected_len == 0){
        if(inst->rx_mode == UART_MODE_DMA){
            // 修正为获取单缓冲区大小
            Size = inst->rx_buf_size - __HAL_DMA_GET_COUNTER(inst->huart->hdmarx);
            // 增加长度校验
            if(Size > inst->rx_buf_size){
                Size = inst->rx_buf_size;
            }
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
