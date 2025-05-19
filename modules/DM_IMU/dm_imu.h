#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "bsp_can.h"


#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define DM_RID_ACCEL 1
#define DM_RID_GYRO  2
#define DM_RID_EULER 3
#define DM_RID_Quaternion 4

typedef struct
{
	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;
    float yawlast;
    float yaw_round;
    float YawTotalAngle;
	Can_Device *can_device;
	uint8_t offline_index;
}dm_imu_t;

typedef struct {
    const float *Yaw;
    const float *Pitch; 
    const float *Roll;
    const float *YawTotalAngle;
    const float (*gyro)[3];    // 指向float[3]数组的指针
} DM_IMU_DATA_T;

extern dm_imu_t dm_imu;

void IMU_UpdateData(const CAN_HandleTypeDef* hcan, const uint32_t rx_id);
void IMU_RequestData(uint16_t can_id,uint8_t reg);
void DM_IMU_Init(void);
DM_IMU_DATA_T DMI_IMU_GetData(void);
#endif
