#include "imu.h"
#include "motor_task.h"
#include "offline.h"
#include "referee.h"
#include "systemwatch.h"
void robot_init(void)
{

    SystemWatch_Init();
    offline_init();
    INS_TASK_init();
    #if defined (GIMBAL_BOARD)
        #if CONTROL_SOURCE == 1
            Remote_init();
        #endif
    #else
    RefereeInit();
    #endif

    // task init
    motor_task_init();

    
}
