#include "chassiscmd.h"
#include "dm_imu.h"
#include "gimbalcmd.h"
#include "imu.h"
#include "motor_task.h"
#include "offline.h"
#include "powercontroller.h"
#include "referee.h"
#include "robot_task.h"
#include "sbus.h"
#include "shootcmd.h"
#include "systemwatch.h"
#include "robotdef.h"
void robot_init(void)
{

    SystemWatch_Init();
    offline_init();
    INS_TASK_init();
    #if defined (GIMBAL_BOARD)
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
}
