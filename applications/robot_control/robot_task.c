#include "cmsis_os.h"
#include "robot_control.h"
#include "systemwatch.h"
#include "robot_task.h"

#define LOG_TAG              "robottask"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

static osThreadId robotTaskHandle;

void robotcmdtask(const void *parameter)
{
    robot_control_init();
    SystemWatch_RegisterTask(robotTaskHandle, "robotTask");
    for (;;)
    {
        SystemWatch_ReportTaskAlive(osThreadGetId());
        robot_control();  
        osDelay(3);
    }
}

void robot_control_task_init(void){
    osThreadDef(robotcmdTask, robotcmdtask, osPriorityNormal, 0, 256);
    robotTaskHandle = osThreadCreate(osThread(robotcmdTask), NULL);

    if (robotTaskHandle == NULL)
    {
        log_e("Failed to create robot_control task");
    }
    log_i("robot_control task created");
}

