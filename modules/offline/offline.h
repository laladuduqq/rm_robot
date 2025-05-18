#ifndef OFFLINE_H
#define OFFLINE_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

// 配置定义
#define MAX_OFFLINE_DEVICES    10      // 最大离线设备数量  beep 10ms周期 最多响10下
#define OFFLINE_INVALID_INDEX  0xFF

// 状态定义
#define STATE_ONLINE           0
#define STATE_OFFLINE          1
#define OFFLINE_ENABLE         1
#define OFFLINE_DISABLE        0

// 离线等级定义
typedef enum {
    OFFLINE_LEVEL_LOW = 1,    // 低优先级
    OFFLINE_LEVEL_MEDIUM = 2, // 中优先级
    OFFLINE_LEVEL_HIGH = 3    // 高优先级
} OfflineLevel_e;

// 设备初始化配置结构体
typedef struct {
    const char* name;         // 设备名称
    uint32_t timeout_ms;      // 超时时间
    OfflineLevel_e level;     // 离线等级
    uint8_t beep_times;       // 蜂鸣次数
    uint8_t enable;           // 是否启用检测
} OfflineDeviceInit_t;

// 离线设备结构体
typedef struct {
    char name[32];          
    uint32_t timeout_ms;    
    OfflineLevel_e level;   
    uint8_t beep_times;     
    bool is_offline;        
    uint32_t last_time;
    uint8_t index;          // 添加索引字段，用于快速更新
    uint8_t enable;         // 是否启用检测

    float dt;
    uint32_t dt_cnt;
} OfflineDevice_t; 

// 离线管理器结构体
typedef struct {
    OfflineDevice_t devices[MAX_OFFLINE_DEVICES];
    uint8_t device_count;
} OfflineManager_t;

// 函数声明
void offline_init(void);
uint8_t offline_device_register(const OfflineDeviceInit_t* init);
void offline_device_update(uint8_t device_index);
void offline_device_enable(uint8_t device_index);
void offline_device_disable(uint8_t device_index);
uint8_t get_device_status(uint8_t device_index);
uint8_t get_system_status(void);

#endif /* OFFLINE_H */
