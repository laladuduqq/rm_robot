#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define BSP_SPI_BUS_MAX_DEVICE_NUM 3

typedef enum {
    BSP_SPI_MODE_BLOCKING,
    BSP_SPI_MODE_IT,
    BSP_SPI_MODE_DMA
} BSP_SPI_Mode;

//spi bus 部分
typedef enum {
    SPI_BUS1,
    SPI_BUS2,
    SPI_BUS_MAX
} SPI_BusType;

//spi 设备部分
typedef struct {
    SPI_BusType target_bus;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    BSP_SPI_Mode tx_mode;
    BSP_SPI_Mode rx_mode;
    uint32_t timeout;
    void (*callback)(void);
} SPI_DeviceInstance_t;

typedef struct {
    SPI_HandleTypeDef* hspi;
    SemaphoreHandle_t mutex;
    uint8_t ref_count; // 设备引用计数
    SPI_DeviceInstance_t* active_dev;
} SPI_Bus_t;

//传输状态
typedef enum {
    SPI_TRANSFER_IDLE,
    SPI_TRANSFER_FIRST,
    SPI_TRANSFER_SECOND
} SPI_TransferState;

SPI_DeviceInstance_t* SPI_DeviceRegister(const SPI_DeviceInstance_t* config);
void BSP_SPI_TransReceive(SPI_DeviceInstance_t* dev, const uint8_t* tx_data, uint8_t* rx_data,uint16_t size);
void BSP_SPI_Transmit(SPI_DeviceInstance_t* dev, const uint8_t* tx_data,uint16_t size);
void BSP_SPI_Receive(SPI_DeviceInstance_t* dev,uint8_t* rx_data,uint16_t size);
HAL_StatusTypeDef BSP_SPI_TransAndTrans(SPI_DeviceInstance_t* dev, const uint8_t* tx_data1, uint16_t size1, const uint8_t* tx_data2, uint16_t size2);

#endif /* __BSP_SPI_H__ */
