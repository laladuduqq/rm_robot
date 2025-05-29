#include "BMI088.h"
#include "RGB.h"
#include "SEGGER_RTT.h"
#include "chassiscmd.h"
#include "cm_backtrace.h"
#include "dm_imu.h"
#include "dwt.h"
#include "elog.h"
#include "gimbalcmd.h"
#include "imu.h"
#include "iwdg.h"
#include "motor_task.h"
#include "offline.h"
#include "powercontroller.h"
#include "referee.h"
#include "robot_task.h"
#include "sbus.h"
#include "shootcmd.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_iwdg.h"
#include "systemwatch.h"
#include "robotdef.h"
#include "vcom.h"

#define HARDWARE_VERSION               "V1.0.0"
#define SOFTWARE_VERSION               "V0.1.0"

void base_init(void)
{
    __HAL_DBGMCU_FREEZE_IWDG();
    cm_backtrace_init("CmBacktrace", HARDWARE_VERSION, SOFTWARE_VERSION);
    RGB_init();
    DWT_Init(168);
    SEGGER_RTT_Init();
    if (elog_user_init() == ELOG_NO_ERR) 
    { elog_start();}
    BMI088_init();
}
void robot_init(void)
{
    SystemWatch_Init();
    offline_init();
    INS_TASK_init();
    #if defined (GIMBAL_BOARD)
    vcom_init();
    DM_IMU_Init();
        #if CONTROL_SOURCE == 1
            Remote_init();
        #endif
    #else
    RefereeInit();
    #endif

    // task init
    powercontrol_init();
    motor_task_init();
    robot_control_task_init();
    #if defined (GIMBAL_BOARD)
    gimbal_task_init();
    shoot_task_init();
    #else
    chassis_task_init();
    #endif
    MX_IWDG_Init();
}
