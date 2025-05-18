#include "motor_task.h"
#include "cmsis_os.h"
#include "damiao.h"
#include "dji.h"
#include "systemwatch.h"

#define LOG_TAG  "motortask"
#include "elog.h"

static osThreadId motorTaskHandle;

void motortask(const void *parameter)
{
    SystemWatch_RegisterTask(motorTaskHandle, "motorTask");
    for (;;)
    {
        SystemWatch_ReportTaskAlive(osThreadGetId());
        DJIMotorControl();
        DMMotorcontrol();
        osDelay(2);
    }
}

void motor_task_init(void){
    osThreadDef(motorTask, motortask, osPriorityAboveNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);
    if (motorTaskHandle == NULL)
    {
        log_e("motorTask create failed");
    }
    log_i("motorTask create success");
}
