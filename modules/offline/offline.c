#include "offline.h"
#include "RGB.h"
#include "dwt.h"
#include "projdefs.h"
#include "systemwatch.h"
#include <stdint.h>
#include <string.h>
#include "timers.h"
#include "tim.h"


#define LOG_TAG  "offline"
#include "elog.h"

// 全局变量
static OfflineManager_t offline_manager;
static TaskHandle_t offline_task_handle = NULL;
static osTimerId beep_timer_handle  = NULL;

// 内部函数声明
static void offline_task(const void *argument);
static inline uint32_t get_time_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
//对于beep 部分定义
#define BEEP_PERIOD   1500
#define BEEP_ON_TIME  50
#define BEEP_OFF_TIME 100

#define BEEP_TUNE_VALUE 500
#define BEEP_CTRL_VALUE 150

#define BEEP_TUNE        TIM4->ARR
#define BEEP_CTRL        TIM4->CCR3

static uint8_t beep_times;
static inline int32_t beep_set_times(uint8_t times);
static void beep_ctrl_times(void);
void beep_set_tune(uint16_t tune, uint16_t ctrl);

void test_timer_callback()
{
    beep_ctrl_times();
}
void offline_init(void)
{
    // 初始化管理器
    memset(&offline_manager, 0, sizeof(offline_manager));


    osThreadDef(offlineTask, offline_task, osPriorityHigh, 0, 256);
    offline_task_handle = osThreadCreate(osThread(offlineTask), NULL);
    
    if(offline_task_handle == NULL) {
        log_e("Failed to create offline task!");
        return;
    }

    HAL_TIM_PWM_Start(&htim4,  TIM_CHANNEL_3);

    // 创建定时器
    osTimerDef(BeepTimer, test_timer_callback);
    beep_timer_handle = osTimerCreate(osTimer(BeepTimer), osTimerPeriodic, NULL);
    
    // 启动定时器，10ms周期
    if (beep_timer_handle != NULL) {
        osStatus status = osTimerStart(beep_timer_handle, 10);
        if (status != osOK) {
            log_e("Failed to start beep timer, status: %d", status);
        } else {
            log_i("Beep timer started successfully");
        }
    } else {
        log_e("Failed to create beep timer");
    }
}

void offline_task(const void *argument)
{    
    UNUSED(argument);
    SystemWatch_RegisterTask(offline_task_handle, "offline_task");
    for (;;)
    {
        SystemWatch_ReportTaskAlive(osThreadGetId());

        static uint8_t beep_time = 0xFF;
        static uint8_t error_level = 0;
        static uint8_t display_index = OFFLINE_INVALID_INDEX;
        uint32_t current_time = xTaskGetTickCount();
        
        // 重置错误状态
        error_level = 0;
        display_index = OFFLINE_INVALID_INDEX;
        bool any_device_offline = false;
    
        // 检查所有设备状态
        for (uint8_t i = 0; i < offline_manager.device_count; i++) {
            OfflineDevice_t* device = &offline_manager.devices[i];
            
            if (!device->enable) {
                continue;
            }
    
            if (current_time - device->last_time > device->timeout_ms) {
                device->is_offline = true;
                any_device_offline = true;
                
                // 更新最高优先级设备
                if (device->level > error_level) {
                    error_level = device->level;
                    display_index = i;
                    beep_time = device->beep_times;
                }
                // 相同优先级时的处理
                else if (device->level == error_level) {
                    // 如果当前设备不需要蜂鸣（beep_times=0），保持原来的设备
                    if (device->beep_times == 0) {
                        continue;
                    }
                    // 如果之前选中的设备不需要蜂鸣，或者当前设备蜂鸣次数更少
                    if (beep_time == 0 || 
                        (device->beep_times > 0 && device->beep_times < beep_time)) {
                        display_index = i;
                        beep_time = device->beep_times;
                    }
                }
            } else {
                device->is_offline = false;
            }
        }
    
        // 触发报警或清除报警
        if (display_index != OFFLINE_INVALID_INDEX && any_device_offline) {
            beep_set_times(offline_manager.devices[display_index].beep_times);
        } else {
            // 所有设备都在线，清除报警
            beep_set_times(0);
            beep_set_tune(0, 0);         // 立即关闭蜂鸣器
            RGB_show(LED_Black);    // 关闭LED
        }
        
        osDelay(10);
    }
}

uint8_t offline_device_register(const OfflineDeviceInit_t* init)
{
    if (init == NULL || offline_manager.device_count >= MAX_OFFLINE_DEVICES) {
        return OFFLINE_INVALID_INDEX;
    }
    
    uint8_t index = offline_manager.device_count;
    OfflineDevice_t* device = &offline_manager.devices[index];
    
    strncpy(device->name, init->name, sizeof(device->name) - 1);
    device->timeout_ms = init->timeout_ms;
    device->level = init->level;
    device->beep_times = init->beep_times;
    device->is_offline = false;
    device->last_time = xTaskGetTickCount();
    device->index = index;
    device->enable = init->enable;
    
    offline_manager.device_count++;
    return index;
}

void offline_device_update(uint8_t device_index)
{
    if (device_index < offline_manager.device_count) {
        offline_manager.devices[device_index].last_time = xTaskGetTickCount();
        offline_manager.devices[device_index].dt = DWT_GetDeltaT(&offline_manager.devices[device_index].dt_cnt);
    }
}

void offline_device_enable(uint8_t device_index)
{
    if (device_index < offline_manager.device_count) {
        offline_manager.devices[device_index].enable = OFFLINE_ENABLE;
    }
}

void offline_device_disable(uint8_t device_index)
{
    if (device_index < offline_manager.device_count) {
        offline_manager.devices[device_index].enable = OFFLINE_DISABLE;
    }
}
uint8_t get_device_status(uint8_t device_index){
    return offline_manager.devices[device_index].is_offline;
}

uint8_t get_system_status(void){
    uint8_t status = 0;
    for (uint8_t i = 0; i < offline_manager.device_count; i++) {
        if (offline_manager.devices[i].is_offline) {
            status |= (1 << i);
        }
    }
    return status;
}


// 设置蜂鸣器状态

int32_t beep_set_times(uint8_t times)
{
    beep_times = times;
    return 0;
}

void beep_set_tune(uint16_t tune, uint16_t ctrl)
{
    BEEP_TUNE = tune;
    BEEP_CTRL = ctrl;
}

/**
  * @brief  called by cycle, control beep times.(one BEEP_PERIOD)
  * @param  NULL
  * @retval
  */
void beep_ctrl_times(void)
{
    static uint32_t beep_tick;
    static uint32_t times_tick;
    static uint8_t times;

    if (get_time_ms() - beep_tick > BEEP_PERIOD)
    {
        times = beep_times;
        beep_tick = get_time_ms();
        times_tick = get_time_ms();
    }
    else if (times != 0)
    {
        if (get_time_ms() - times_tick < BEEP_ON_TIME)
        {
            beep_set_tune(BEEP_TUNE_VALUE, BEEP_CTRL_VALUE);
            RGB_show(LED_Red);
        }
        else if (get_time_ms() - times_tick < BEEP_ON_TIME + BEEP_OFF_TIME)
        {
            beep_set_tune(0, 0);
            RGB_show(LED_Black);
        }
        else
        {
            times--;
            times_tick = get_time_ms();
        }
    }
}
