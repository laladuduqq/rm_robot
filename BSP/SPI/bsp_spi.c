#include "bsp_spi.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "spi.h"

#define LOG_TAG "bsp_spi"
#include "elog.h"

// SPI总线管理
static SPI_Bus_t spi_bus_pool[SPI_BUS_MAX] = {
    [SPI_BUS1] = {.hspi = NULL, .mutex = NULL, .ref_count = 0},
    [SPI_BUS2] = {.hspi = NULL, .mutex = NULL, .ref_count = 0}
};

static void SPI_BusInit(SPI_BusType bus_type)
{
    SPI_Bus_t* bus = &spi_bus_pool[bus_type];
    if(bus->mutex == NULL)
    {
        bus->mutex = xSemaphoreCreateBinary();
        xSemaphoreGive(bus->mutex); // 初始化为可用状态
        log_i("SPI bus %d mutex created", bus_type);
    }
    // 运行时绑定HAL句柄
    switch(bus_type)
    {
        case SPI_BUS1:
            bus->hspi = &hspi1;
            log_i("SPI1 bus initialized");
            break;
        case SPI_BUS2:
            bus->hspi = &hspi2;
            log_i("SPI2 bus initialized");
            break;
        default:
            log_e("Invalid SPI bus type: %d", bus_type);
            break;
    }
}

// 设备注册函数
SPI_DeviceInstance_t* SPI_DeviceRegister(const SPI_DeviceInstance_t* config)
{
    // 参数有效性检查
    if (!config) {log_e("NULL config pointer");return NULL;}

    SPI_Bus_t* bus = &spi_bus_pool[config->target_bus];
    // 检查设备数量限制
    if(bus->ref_count >= BSP_SPI_BUS_MAX_DEVICE_NUM) {
        log_e("SPI bus %d reached max device limit", config->target_bus);
        return NULL;
    }
    // 首次注册时初始化总线
    if(bus->hspi == NULL) { 
        log_i("First device on SPI bus %d, initializing bus", config->target_bus);
        SPI_BusInit(config->target_bus);}

    // 创建设备实例（深拷贝配置）
    SPI_DeviceInstance_t* dev = malloc(sizeof(SPI_DeviceInstance_t));
    memcpy(dev, config, sizeof(SPI_DeviceInstance_t)); // 复制全部配置

    // 配置CS引脚（与原来相同）
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = dev->cs_pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(dev->cs_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    // 添加到总线
    bus->ref_count++;
    log_i("SPI device registered on bus %d, CS pin: %d, ref count: %d", 
        config->target_bus, dev->cs_pin, bus->ref_count);
    return dev;
}

// 设备反注册函数
void SPI_DeviceUnRegister(const SPI_DeviceInstance_t* dev)
{
    if (!dev) return;
    SPI_BusType bus_type = dev->target_bus; // 提前保存总线类型

    // 获取总线锁保护共享资源
    if(xSemaphoreTake(spi_bus_pool[bus_type].mutex, portMAX_DELAY) != pdTRUE) return;
    // 引用计数操作
    if(spi_bus_pool[bus_type].ref_count > 0) {
        spi_bus_pool[bus_type].ref_count--;
    }
    // 判断是否需要删除互斥量
    bool delete_mutex = (spi_bus_pool[bus_type].ref_count == 0);
    SemaphoreHandle_t mutex = spi_bus_pool[bus_type].mutex;
    xSemaphoreGive(spi_bus_pool[bus_type].mutex); // 先释放总线锁

    // 删除互斥量（需在无锁状态下操作）
    if(delete_mutex) {
        vSemaphoreDelete(mutex);
        spi_bus_pool[bus_type].mutex = NULL; // 清除指针避免野指针
    }

    // 释放设备内存（需强制转换去掉 const 限定）
    free((SPI_DeviceInstance_t*)dev);
}


// 数据传输封装
void BSP_SPI_TransReceive(SPI_DeviceInstance_t* dev, const uint8_t* tx_data, uint8_t* rx_data, const uint16_t size)
{
    SPI_HandleTypeDef* hspi = spi_bus_pool[dev->target_bus].hspi;

    if(xSemaphoreTake(spi_bus_pool[dev->target_bus].mutex, 100) != pdTRUE) {return;}

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    spi_bus_pool[dev->target_bus].active_dev = dev;

    if (dev->tx_mode == BSP_SPI_MODE_DMA && dev->rx_mode == BSP_SPI_MODE_DMA) {
        HAL_SPI_TransmitReceive_DMA(hspi, tx_data, rx_data, size);
    }
    else if (dev->tx_mode == BSP_SPI_MODE_IT && dev->rx_mode == BSP_SPI_MODE_IT) {
        HAL_SPI_TransmitReceive_IT(hspi, tx_data, rx_data, size);
    }
    else {
        HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, size,1000);
        xSemaphoreGive(spi_bus_pool[dev->target_bus].mutex);
        spi_bus_pool[dev->target_bus].active_dev = NULL;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }
}

void BSP_SPI_Transmit(SPI_DeviceInstance_t* dev, const uint8_t* tx_data,uint16_t size)
{
    SPI_HandleTypeDef* hspi = spi_bus_pool[dev->target_bus].hspi;

    if(xSemaphoreTake(spi_bus_pool[dev->target_bus].mutex, 100) != pdTRUE) {return;}

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    spi_bus_pool[dev->target_bus].active_dev = dev;

    if (dev->tx_mode == BSP_SPI_MODE_DMA) {
        HAL_SPI_Transmit_DMA(hspi, tx_data, size);
    }
    else if (dev->tx_mode == BSP_SPI_MODE_IT) {
        HAL_SPI_Transmit_IT(hspi, tx_data, size);
    }
    else {
        HAL_SPI_Transmit(hspi, tx_data, size,1000);
        xSemaphoreGive(spi_bus_pool[dev->target_bus].mutex);
        spi_bus_pool[dev->target_bus].active_dev = NULL;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }
}

void BSP_SPI_Receive(SPI_DeviceInstance_t* dev,uint8_t* rx_data,uint16_t size)
{
    SPI_HandleTypeDef* hspi = spi_bus_pool[dev->target_bus].hspi;

    if(xSemaphoreTake(spi_bus_pool[dev->target_bus].mutex, 100) != pdTRUE) {return;}

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    spi_bus_pool[dev->target_bus].active_dev = dev;

    if (dev->tx_mode == BSP_SPI_MODE_DMA) {
        HAL_SPI_Receive_DMA(hspi, rx_data, size);
    }
    else if (dev->tx_mode == BSP_SPI_MODE_IT) {
        HAL_SPI_Receive_IT(hspi, rx_data, size);
    }
    else {
        HAL_SPI_Receive(hspi, rx_data, size,1000);
        xSemaphoreGive(spi_bus_pool[dev->target_bus].mutex);
        spi_bus_pool[dev->target_bus].active_dev = NULL;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }
}

HAL_StatusTypeDef BSP_SPI_TransAndTrans(SPI_DeviceInstance_t* dev, const uint8_t* tx_data1, uint16_t size1, const uint8_t* tx_data2, uint16_t size2)
{
    SPI_HandleTypeDef* hspi = spi_bus_pool[dev->target_bus].hspi;
    HAL_StatusTypeDef status = HAL_OK;

    // 获取总线锁
    if(xSemaphoreTake(spi_bus_pool[dev->target_bus].mutex, dev->timeout) != pdTRUE) {
        return HAL_TIMEOUT;
    }

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    spi_bus_pool[dev->target_bus].active_dev = dev;

    // 阻塞模式连续传输
    do {
        // 第一次传输
        if((status = HAL_SPI_Transmit(hspi, tx_data1, size1, dev->timeout)) != HAL_OK) break;
        // 第二次传输
        status = HAL_SPI_Transmit(hspi, tx_data2, size2, dev->timeout);
    } while(0);

    // 无论成功与否都要恢复CS和释放锁
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    xSemaphoreGive(spi_bus_pool[dev->target_bus].mutex);
    spi_bus_pool[dev->target_bus].active_dev = NULL;

    return status;
}


// 中断部分处理

//这里是HAL_SPI_TransmitReceive_it/dma回调
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{

    for (int i = 0; i < SPI_BUS_MAX; ++i)
    {
        if (spi_bus_pool[i].mutex && spi_bus_pool[i].hspi == hspi)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(spi_bus_pool[i].mutex,&xHigherPriorityTaskWoken);
            HAL_GPIO_WritePin(spi_bus_pool[i].active_dev->cs_port, spi_bus_pool[i].active_dev->cs_pin, GPIO_PIN_SET);
            // 清除活动设备记录
            spi_bus_pool[i].active_dev = NULL;
            if (xHigherPriorityTaskWoken){portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}// 必要时触发上下文切换
            break;
        }
    }
}

//这里是HAL_SPI_Transmit_it/dma回调
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    for (int i = 0; i < SPI_BUS_MAX; ++i)
    {
        if (spi_bus_pool[i].mutex && spi_bus_pool[i].hspi == hspi)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(spi_bus_pool[i].mutex,&xHigherPriorityTaskWoken);
            HAL_GPIO_WritePin(spi_bus_pool[i].active_dev->cs_port, spi_bus_pool[i].active_dev->cs_pin, GPIO_PIN_SET);
            // 清除活动设备记录
            spi_bus_pool[i].active_dev = NULL;
            if (xHigherPriorityTaskWoken){portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}// 必要时触发上下文切换
            break;
        }
    }
}

//这里是HAL_SPI_Receive_it/dma回调
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (int i = 0; i < SPI_BUS_MAX; ++i)
    {
        if (spi_bus_pool[i].mutex && spi_bus_pool[i].hspi == hspi)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(spi_bus_pool[i].mutex,&xHigherPriorityTaskWoken);
            HAL_GPIO_WritePin(spi_bus_pool[i].active_dev->cs_port, spi_bus_pool[i].active_dev->cs_pin, GPIO_PIN_SET);
            // 清除活动设备记录
            spi_bus_pool[i].active_dev = NULL;
            if (xHigherPriorityTaskWoken){portYIELD_FROM_ISR(xHigherPriorityTaskWoken);}// 必要时触发上下文切换
            break;
        }
    }
}
