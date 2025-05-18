#include "dm_imu.h"
#include "imu.h"
#include "motor_task.h"
#include "offline.h"
#include "referee.h"
#include "sbus.h"
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
    motor_task_init();

    
}
