#include "bsp_can.h"
#include <string.h>
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_gcc.h"
#include "dwt.h"
#include "semphr.h"
#include "stm32f4xx_hal_def.h"

#define LOG_TAG "bsp_can"
#include "elog.h"

/* 总线管理器实例 */
static CANBusManager can_bus[] = {
    {.hcan = NULL, .tx_mutex = NULL },  // CAN1总线
    {.hcan = NULL, .tx_mutex = NULL },    // CAN2总线
};

void canbus_init(void) {
    can_bus[0].hcan = &hcan1;
    can_bus[0].tx_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(can_bus[0].tx_mutex);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    log_i("CAN1 bus initialized");

    can_bus[1].hcan = &hcan2;
    can_bus[1].tx_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(can_bus[1].tx_mutex);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    log_i("CAN2 bus initialized");
}

static void CANAddFilter(Can_Device *device)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = (device->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdLow = device->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank = device->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

    HAL_CAN_ConfigFilter(device->can_handle, &can_filter_conf);
}

static bool check_device_id_conflict(CANBusManager *bus, uint16_t tx_id, uint16_t rx_id) {
    for(uint8_t i = 0; i < MAX_DEVICES_PER_BUS; i++) {
        if(bus->devices[i].can_handle != NULL) {  // 使用can_handle判断设备是否存在
            if(bus->devices[i].rx_id == rx_id) {
                log_e("Device ID conflict: new(tx:0x%03X, rx:0x%03X) with existing(tx:0x%03X, rx:0x%03X)", 
                    (unsigned int)tx_id, (unsigned int)rx_id,
                    (unsigned int)bus->devices[i].tx_id, (unsigned int)bus->devices[i].rx_id);
                return true;
            }
        }
    }
    return false;
}
/****************** 设备初始化 ******************/
Can_Device* BSP_CAN_Device_Init(Can_Device_Init_Config_s *config) {
    /* 查找对应总线 */
    if (can_bus[0].hcan == NULL) {
        log_i("First time initializing CAN bus");
        canbus_init();} //先注册总线
    CANBusManager *bus = NULL;
    for(uint8_t i=0; i<2; i++) {
        if(can_bus[i].hcan == config->can_handle) {
            bus = &can_bus[i];
            break;
        }
    }
    if(!bus) {
        log_e("CAN bus not found for handle: 0x%08X", (unsigned int)config->can_handle);
        return NULL;
    }

    /* 检查设备数量是否超限 */
    uint8_t device_count = bus->device_count;
    if (device_count >= MAX_DEVICES_PER_BUS) {
        log_e("Failed to register CAN device: Maximum devices (%d) reached on bus", MAX_DEVICES_PER_BUS);
        return NULL;
    }

    /* 检查ID是否冲突 */
    if(check_device_id_conflict(bus, config->tx_id, config->rx_id)) {
        log_e("Failed to register CAN device: ID conflict");
        return NULL;
    }

    /* 注册设备到总线 */
    for(uint8_t i=0; i<MAX_DEVICES_PER_BUS; i++) {
        if(bus->devices[i].can_handle == NULL) {  // 使用can_handle为空判断槽位是否可用
            // 直接复制到结构体数组中
            bus->devices[i].can_handle = config->can_handle;
            bus->devices[i].tx_id = config->tx_id;
            bus->devices[i].rx_id = config->rx_id;
            bus->devices[i].tx_mode = config->tx_mode;
            bus->devices[i].rx_mode = config->rx_mode;
            bus->devices[i].can_callback = config->can_callback;
            
            /* 发送配置初始化 */
            bus->devices[i].txconf.StdId = config->tx_id;
            bus->devices[i].txconf.IDE = CAN_ID_STD;
            bus->devices[i].txconf.RTR = CAN_RTR_DATA;
            bus->devices[i].txconf.DLC = 8;
            bus->devices[i].txconf.TransmitGlobalTime = DISABLE;
            
            CANAddFilter(&bus->devices[i]); // 添加过滤器
            
            bus->device_count++;

            /* 启动接收 */
            if(bus->devices[i].tx_mode == CAN_MODE_IT) {
                HAL_CAN_ActivateNotification(bus->devices[i].can_handle,CAN_IT_ERROR);
            }
            HAL_CAN_Start(bus->devices[i].can_handle);

            log_i("CAN device initialized with TX_ID: 0x%03X, RX_ID: 0x%03X", 
                (unsigned int)bus->devices[i].tx_id, (unsigned int)bus->devices[i].rx_id);
            return &bus->devices[i];  // 返回设备结构体的地址
        }
    }

    log_e("No free device slot available");
    return NULL;
}

/****************** 发送函数 ******************/
uint8_t CAN_SendMessage(Can_Device *device, uint8_t len) {
    uint32_t start_time;
    uint32_t timeout_cycles = CAN_SEND_TIMEOUT_US * 168; // 168MHz下的周期数
    uint8_t retry_cnt = CAN_SEND_RETRY_CNT;

    while (retry_cnt--) {
        start_time = DWT->CYCCNT;
        // 快速检查邮箱
        if (HAL_CAN_GetTxMailboxesFreeLevel(device->can_handle) > 0) {
            device->txconf.DLC = len;
            HAL_CAN_AddTxMessage(device->can_handle, 
                                      &device->txconf,
                                      device->tx_buff,
                                      &device->tx_mailbox);
            return HAL_OK;
        }
        
        // 短时等待
        while ((DWT->CYCCNT - start_time) < timeout_cycles);
    }
    return HAL_TIMEOUT;
}

uint8_t CAN_SendMessage_hcan(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader,
    const uint8_t aData[], uint32_t *pTxMailbox, uint8_t len) {
    uint32_t start_time;
    uint32_t timeout_cycles = CAN_SEND_TIMEOUT_US * 168; // 168MHz下的周期数
    uint8_t retry_cnt = CAN_SEND_RETRY_CNT;

    while (retry_cnt--) {
        start_time = DWT->CYCCNT;
        // 快速检查邮箱
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
            pHeader->DLC = len;
            HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox);
            return HAL_OK;
        }
        
        // 短时等待
        while ((DWT->CYCCNT - start_time) < timeout_cycles);
    }
    
    return HAL_TIMEOUT;
}

void BSP_CAN_Device_DeInit(Can_Device *dev) {
    if(dev == NULL) {
        log_e("Trying to deinit NULL device");
        return;
    }
    for(uint8_t bus=0; bus<2; bus++) {
        for(uint8_t i=0; i<MAX_DEVICES_PER_BUS; i++) {
            // 通过比较内存地址来查找设备
            if(&(can_bus[bus].devices[i]) == dev) {
                // 清零设备结构体
                memset(&can_bus[bus].devices[i], 0, sizeof(Can_Device));
                can_bus[bus].device_count--;
                log_i("CAN device deinitialized: bus=%d, index=%d", bus, i);
                return;
            }
        }
    }
    log_w("Device not found in CAN bus managers");
}

/****************** 中断处理 ******************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    static CAN_RxHeaderTypeDef rx_header;
    static uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) == HAL_OK) {
            CANBusManager *bus = NULL;
            for(uint8_t i=0; i<2; i++) {
                if(can_bus[i].hcan == hcan) {
                    bus = &can_bus[i];
                    break;
                }
            }
            if(!bus) return;

            // 遍历设备查找匹配ID
            for(uint8_t i=0; i<MAX_DEVICES_PER_BUS; i++) {
                if(bus->devices[i].can_handle != NULL && 
                   rx_header.StdId == bus->devices[i].rx_id && 
                   bus->devices[i].can_handle == hcan) 
                {
                    __disable_irq();
                    memcpy(bus->devices[i].rx_buff, data, rx_header.DLC);
                    bus->devices[i].rx_len = rx_header.DLC;
                    if(bus->devices[i].can_callback) {
                        bus->devices[i].can_callback(bus->devices[i].can_handle, rx_header.StdId);
                    }
                    __enable_irq();
                }
            }
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    static CAN_RxHeaderTypeDef rx_header;
    static uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1)) {
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, data) == HAL_OK) {
            CANBusManager *bus = NULL;
            for(uint8_t i=0; i<2; i++) {
                if(can_bus[i].hcan == hcan) {
                    bus = &can_bus[i];
                    break;
                }
            }
            if(!bus) return;

            // 遍历设备查找匹配ID
            for(uint8_t i=0; i<MAX_DEVICES_PER_BUS; i++) {
                if(bus->devices[i].can_handle != NULL && 
                   rx_header.StdId == bus->devices[i].rx_id && 
                   bus->devices[i].can_handle == hcan) 
                {
                    __disable_irq();
                    memcpy(bus->devices[i].rx_buff, data, rx_header.DLC);
                    bus->devices[i].rx_len = rx_header.DLC;
                    if(bus->devices[i].can_callback) {
                        bus->devices[i].can_callback(bus->devices[i].can_handle, rx_header.StdId);
                    }
                    __enable_irq();
            }
            }
        }
    }
}



// 增强错误处理回调
// void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
// {
//     uint32_t error_code = HAL_CAN_GetError(hcan);
    
//     // 重置错误标志
//     HAL_CAN_ResetError(hcan);
    
//     // 如果是严重错误，执行完整的重启流程
//     if(error_code) {
//         HAL_CAN_Stop(hcan);
//         HAL_CAN_Start(hcan);
        
//         // 重新激活所有中断
//         HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
//                                           CAN_IT_RX_FIFO1_MSG_PENDING |
//                                           CAN_IT_ERROR);
        
//     }
// }
