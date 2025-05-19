#include "systemwatch.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "dwt.h"
#include "task.h"
#include "tim.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG  "systemwatch"
#include "elog.h"

// 静态变量
static uint8_t systemwatch_init=0;
static TaskMonitor_t taskList[MAX_MONITORED_TASKS];
static uint8_t taskCount = 0;
static osThreadId watchTaskHandle;
static volatile uint32_t watch_task_last_active = 0;

// 辅助函数声明
static void PrintTaskInfo(TaskStatus_t *pxTaskStatus, TaskMonitor_t *pxTaskMonitor);
static void PrintSystemStatus(void);

static void SystemWatch_Task(const void *argument)
{
    UNUSED(argument);
    float last_dt[MAX_MONITORED_TASKS] = {0};
    
    while(1) {
        taskList[0].dt = DWT_GetDeltaT(&taskList[0].dt_cnt); //自身更新
        watch_task_last_active = xTaskGetTickCount();
        for(uint8_t i = 0; i < taskCount; i++) {
            if(taskList[i].isActive) {
                // 检查任务执行间隔是否过长
                if(taskList[i].dt > TASK_BLOCK_TIMEOUT) {
                    taskENTER_CRITICAL();
                    
                    log_e("\r\n**** Task Blocked Detected! System State Dump ****");
                    log_e("Time: %.3f s", DWT_GetTimeline_s());
                    log_e("----------------------------------------");
                    
                    TaskStatus_t taskStatus;
                    vTaskGetInfo(taskList[i].handle,
                               &taskStatus,
                               pdTRUE,
                               eInvalid);
                    
                    log_e("Blocked Task Information:");
                    PrintTaskInfo(&taskStatus, &taskList[i]);
                    
                    PrintSystemStatus();
                    taskEXIT_CRITICAL();
                    osDelay(1000);
                    HAL_NVIC_SystemReset();   
                }
                last_dt[i] = taskList[i].dt;
            }
        }
        osDelay(MONITOR_PERIOD);
    }
}

// 定时器中断回调，用于监控 SystemWatch 任务本身
void sysytemwatch_it_callback(void){
    if (systemwatch_init==1)
    {
        {
            if (taskList[0].dt > TASK_BLOCK_TIMEOUT) //列表第一个就是watchtask本身
            {
                log_e("\r\n**** SystemWatch Task Blocked! ****");
                log_e("Last dt: %.3f ms", taskList[0].dt * 1000.0f);
                HAL_NVIC_SystemReset();
            }
        }
    }
}


void SystemWatch_Init(void)
{
    // 防止重复初始化
    if(systemwatch_init) {
        return;
    }
    
    // 初始化任务列表
    memset(taskList, 0, sizeof(taskList));
    taskCount = 0;
    
    systemwatch_init = 1;

    watch_task_last_active = xTaskGetTickCount();
    HAL_TIM_Base_Start_IT(&htim6);
    
    osThreadDef(WatchTask, SystemWatch_Task, osPriorityRealtime, 0, 256);
    watchTaskHandle = osThreadCreate(osThread(WatchTask), NULL);
    
    if(watchTaskHandle == NULL) {
        log_e("Failed to create SystemWatch task!");
        return;
    }

    SystemWatch_RegisterTask(watchTaskHandle, "WatchTask");
    
    log_i("SystemWatch initialized, watch task created.");
}

static void PrintTaskInfo(TaskStatus_t *pxTaskStatus, TaskMonitor_t *pxTaskMonitor)
{
    log_e("Name: %s", pxTaskMonitor->name);
    log_e("Handle: 0x%x", (unsigned int)pxTaskMonitor->handle);
    log_e("Last dt: %.3f ms", pxTaskMonitor->dt * 1000.0f);
    log_e("Stack HWM: %lu words (%lu bytes)", 
          pxTaskStatus->usStackHighWaterMark,
          pxTaskStatus->usStackHighWaterMark * sizeof(StackType_t));
    log_e("Base Priority: %lu", pxTaskStatus->uxBasePriority);
    log_e("Current Priority: %lu", pxTaskStatus->uxCurrentPriority);
}

static void PrintSystemStatus(void)
{
    log_e("\r\nSystem Status:");
    log_e("----------------------------------------");
    
    // 内存使用情况
    log_e("Memory Usage:");
    log_e("- Free Heap: %u bytes", xPortGetFreeHeapSize());
    log_e("- Minimum Ever Free: %u bytes", xPortGetMinimumEverFreeHeapSize());
    log_e("----------------------------------------\r\n");
}

static const char* GetTaskStateString(eTaskState state)
{
    switch (state) {
        case eRunning:   return "Running";
        case eReady:     return "Ready";
        case eBlocked:   return "Blocked";
        case eSuspended: return "Suspended";
        case eDeleted:   return "Deleted";
        default:         return "Unknown";
    }
}

int8_t SystemWatch_RegisterTask(osThreadId taskHandle, const char* taskName)
{
    // 1. 严格的参数验证
    if (taskHandle == NULL || taskName == NULL) {
        return -1;
    }
    
    // 2. 验证任务句柄的有效性
    if (xTaskGetCurrentTaskHandle() == NULL) {
        return -1;
    }
    
    // 3. 确保系统监控已初始化
    if (!systemwatch_init) {
        return -1;
    }
    
    // 4. 进入临界区（使用FreeRTOS的API）
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    
    // 5. 检查是否已注册
    for(uint8_t i = 0; i < taskCount; i++) {
        if(taskList[i].handle == taskHandle) {
            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
            return -1;
        }
    }
    
    // 6. 检查数组边界
    if(taskCount >= MAX_MONITORED_TASKS) {
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return -1;
    }
    
    TaskMonitor_t* newTask = &taskList[taskCount];
    newTask->handle = taskHandle;
    newTask->name = taskName;
    newTask->dt = DWT_GetDeltaT(&taskList[taskCount].dt_cnt);;
    newTask->isActive = 1;
    
    // 8. 原子递增计数器
    taskCount++;
    
    // 9. 退出临界区
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    
    return 0;
}

void SystemWatch_ReportTaskAlive(osThreadId taskHandle)
{
    for(uint8_t i = 0; i < taskCount; i++) {
        if(taskList[i].handle == taskHandle) {
            taskList[i].dt = DWT_GetDeltaT(&taskList[i].dt_cnt);
            break;
        }
    }
}
