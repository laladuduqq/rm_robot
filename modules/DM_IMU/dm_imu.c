#include "dm_imu.h"
#include "bsp_can.h"
#include "can.h"
#include "cmsis_os.h"
#include "offline.h"
#include "systemwatch.h"
#include <stdint.h>

#define LOG_TAG              "dmimu"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

static dm_imu_t dm_imu;
static osThreadId dmimuTaskHandle;
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}



void IMU_RequestData(uint16_t can_id,uint8_t reg)
{
    dm_imu.can_device->txconf.DLC = 4;
    dm_imu.can_device->txconf.StdId = 0x6FF;

    dm_imu.can_device->tx_buff[0] = (uint8_t)can_id;
    dm_imu.can_device->tx_buff[1] = (uint8_t)(can_id>>8);
    dm_imu.can_device->tx_buff[2] = reg;
    dm_imu.can_device->tx_buff[3] = 0XCC;

    if (dm_imu.can_device != NULL)
    {
        CAN_SendMessage(dm_imu.can_device, dm_imu.can_device->txconf.DLC);
    }
}


void IMU_UpdateAccel(const Can_Device *candevice)
{
	uint16_t accel[3];
	
	accel[0]=candevice->rx_buff[3]<<8|candevice->rx_buff[2];
	accel[1]=candevice->rx_buff[5]<<8|candevice->rx_buff[4];
	accel[2]=candevice->rx_buff[7]<<8|candevice->rx_buff[6];
	
	dm_imu.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	dm_imu.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	dm_imu.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void IMU_UpdateGyro(const Can_Device *candevice)
{
	uint16_t gyro[3];
	
	gyro[0]=candevice->rx_buff[3]<<8|candevice->rx_buff[2];
	gyro[1]=candevice->rx_buff[5]<<8|candevice->rx_buff[4];
	gyro[2]=candevice->rx_buff[7]<<8|candevice->rx_buff[6];
	
	dm_imu.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	dm_imu.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	dm_imu.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void IMU_UpdateEuler(const Can_Device *candevice)
{
	int euler[3];
	
	euler[0]=candevice->rx_buff[3]<<8|candevice->rx_buff[2];
	euler[1]=candevice->rx_buff[5]<<8|candevice->rx_buff[4];
	euler[2]=candevice->rx_buff[7]<<8|candevice->rx_buff[6];
	
	dm_imu.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	dm_imu.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	dm_imu.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (dm_imu.yaw - dm_imu.yawlast > 180.0f){dm_imu.yaw_round--;}
    else if (dm_imu.yaw - dm_imu.yawlast < -180.0f){dm_imu.yaw_round++;}
    dm_imu.YawTotalAngle = 360.0f * dm_imu.yaw_round + dm_imu.yaw;
    dm_imu.yawlast = dm_imu.yaw;
}


void IMU_UpdateQuaternion(const Can_Device *candevice)
{
	int w = candevice->rx_buff[1]<<6| ((candevice->rx_buff[2]&0xF8)>>2);
	int x = (candevice->rx_buff[2]&0x03)<<12|(candevice->rx_buff[3]<<4)|((candevice->rx_buff[4]&0xF0)>>4);
	int y = (candevice->rx_buff[4]&0x0F)<<10|(candevice->rx_buff[5]<<2)|(candevice->rx_buff[6]&0xC0)>>6;
	int z = (candevice->rx_buff[6]&0x3F)<<8|candevice->rx_buff[7];
	
	dm_imu.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	dm_imu.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	dm_imu.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	dm_imu.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

void IMU_UpdateData(const CAN_HandleTypeDef* hcan, const uint32_t rx_id)
{
    if (dm_imu.can_device->rx_id == rx_id)
    {
        offline_device_update(dm_imu.offline_index);
        switch(dm_imu.can_device->rx_buff[0])
        {
            case 1:
                IMU_UpdateAccel(dm_imu.can_device);
                break;
            case 2:
                IMU_UpdateGyro(dm_imu.can_device);
                break;
            case 3:
                IMU_UpdateEuler(dm_imu.can_device);
                break;
            case 4:
                IMU_UpdateQuaternion(dm_imu.can_device);
                break;
        }
    }
}

DM_IMU_DATA_T DMI_IMU_GetData(void)
{
    DM_IMU_DATA_T data;
    data.Yaw = &dm_imu.yaw;
    data.Pitch = &dm_imu.pitch;
    data.Roll = &dm_imu.roll;
    data.YawTotalAngle = &dm_imu.YawTotalAngle;
    data.gyro = (const float (*)[3])&dm_imu.gyro;    // 使用类型转换确保类型匹配
    return data;
}

void DMimuTask(const void *parameter)
{
    SystemWatch_RegisterTask(dmimuTaskHandle, "dmimuTask");
    for (;;) 
    {
        SystemWatch_ReportTaskAlive(osThreadGetId());
        IMU_RequestData(dm_imu.can_device->tx_id,DM_RID_GYRO);
        IMU_RequestData(dm_imu.can_device->tx_id,DM_RID_EULER);
        osDelay(2);
    }
}

void DM_IMU_Init(void)
{
    OfflineDeviceInit_t offline_init = {
        .name = "dm_imu",
        .timeout_ms = 100,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 1,
        .enable = OFFLINE_ENABLE,
    };
    dm_imu.offline_index = offline_device_register(&offline_init);
    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = &hcan2,
        .tx_id = 0x01,
        .rx_id = 0x11,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
        .can_callback = IMU_UpdateData
    };
    // 注册 CAN 设备并获取引用
    Can_Device *can_dev = BSP_CAN_Device_Init(&can_config);
    if (can_dev == NULL) {
        log_e("Failed to initialize CAN device for DM imu");
    }
    // 保存设备指针
    dm_imu.can_device = can_dev;

    osThreadDef(dmimuTask, DMimuTask, osPriorityAboveNormal, 0, 256);
    dmimuTaskHandle = osThreadCreate(osThread(dmimuTask), NULL);

    if (dmimuTaskHandle == NULL)
    {
        log_e("Failed to create DMimuTask");
    }
    log_i("DMimuTask created");
}
