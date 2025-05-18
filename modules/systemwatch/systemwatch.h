#ifndef __SYSTEMWATCH_H__
#define __SYSTEMWATCH_H__

#include <stdint.h>
#include "cmsis_os.h"

// 最大监控任务数
#define MAX_MONITORED_TASKS 10
// 任务阻塞判定时间 (ms)
#define TASK_BLOCK_TIMEOUT 1000
// 监控任务运行周期 (ms)
#define MONITOR_PERIOD 100

typedef struct {
    osThreadId handle;    // 任务句柄
    uint32_t counter;       // 任务心跳计数器
    const char* name;       // 任务名称
    uint8_t isActive;       // 是否在监控
} TaskMonitor_t;

// 初始化系统监控
void SystemWatch_Init(void);

// 注册需要监控的任务
// 返回值: 0-成功, -1-失败
int8_t SystemWatch_RegisterTask(osThreadId taskHandle, const char* taskName);
// 在被监控的任务中调用此函数更新计数器
void SystemWatch_ReportTaskAlive(osThreadId taskHandle);
void sysytemwatch_it_callback(void);
void sysytemwatch_it_callback(void);

#endif /* __SYSTEMWATCH_H__ */
