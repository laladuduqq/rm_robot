#include <stdint.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "BMI088_reg.h"
#include "BMI088.h"
#include "bsp_flash.h"
#include "bsp_spi.h"
#include "controller.h"
#include "RGB.h"
#include "dwt.h"
#include "tim.h"

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#define LOG_TAG "bmi088"
#include "elog.h"


static BMI088_Data_t BMI088_Data;
float gyroDiff[3], gNormDiff;

static SPI_DeviceInstance_t *bmi_gyro_device;
static SPI_DeviceInstance_t *bmi_acc_device;

static void bmi088_get_accel(void);
static void bmi088_get_gyro(void);


/* 内部使用的bmi088读写函数*/
void _bmi088_writedata(SPI_DeviceInstance_t *device  , const uint8_t addr, const uint8_t *data, const uint8_t data_len){
    uint8_t ptdata = addr & BMI088_SPI_WRITE_CODE;
    BSP_SPI_TransAndTrans(device,&ptdata,1,data,data_len);
}

void _bmi088_readdata(SPI_DeviceInstance_t *device  , const uint8_t addr, uint8_t *data, const uint8_t data_len){
    static uint8_t tmp[8];
    static uint8_t tx[8]={0};

    configASSERT(data_len <= 6)

    if (device==bmi_acc_device)
    {
        tx[0] = BMI088_SPI_READ_CODE | addr;
        BSP_SPI_TransReceive(device, tx,tmp, data_len+2);
        memcpy(data, tmp+2, data_len);//前2字节dummy byte
    }
    else if(device==bmi_gyro_device){
        tx[0] = BMI088_SPI_READ_CODE | addr;
        BSP_SPI_TransReceive(device, tx,tmp, data_len+1);
        memcpy(data, tmp+1, data_len);//前1字节dummy byte
    }
}

/*bmi088底层函数，用于配置bmi088和读取acc,gyro数据*/
#define BMI088REG 0
#define BMI088DATA 1
#define BMI088ERROR 2
// BMI088初始化配置数组for accel,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};
// BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

void VerifyAccSelfTest(void) {
    float pos_data[3], neg_data[3]={0.0f};
    static uint8_t tmp=0;
    static uint8_t buf[6] = {0};

    tmp=BMI088_ACC_RANGE_24G;
    _bmi088_writedata(bmi_acc_device,BMI088_ACC_RANGE,&tmp,1);
    DWT_Delay(0.01);
    tmp=0XA7;
    _bmi088_writedata(bmi_acc_device,BMI088_ACC_CONF,&tmp,1);
    DWT_Delay(0.01);
    tmp=BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL;
    _bmi088_writedata(bmi_acc_device,BMI088_ACC_SELF_TEST,&tmp,1);
    DWT_Delay(0.01);
    
    _bmi088_readdata(bmi_acc_device, BMI088_ACCEL_XOUT_L, buf, 6);
    for (uint8_t i = 0; i < 3; i++)
    {
        pos_data[i] = BMI088_ACCEL_6G_SEN *9.80f * (float)(int16_t)(buf[2 * i + 1] << 8 | buf[2 * i]);
    }

    tmp = BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL;
    _bmi088_writedata(bmi_acc_device,BMI088_ACC_SELF_TEST,&tmp,1);
    DWT_Delay(0.1);

    _bmi088_readdata(bmi_acc_device, BMI088_ACCEL_XOUT_L, buf, 6);
    for (uint8_t i = 0; i < 3; i++)
    {
        neg_data[i] = BMI088_ACCEL_6G_SEN *9.80f * (float)(int16_t)(buf[2 * i + 1] << 8 | buf[2 * i]);
    }

    tmp = BMI088_ACC_SELF_TEST_OFF;
    _bmi088_writedata(bmi_acc_device,BMI088_ACC_SELF_TEST,&tmp,1);
    DWT_Delay(0.1);
    if ((abs(pos_data[0] - neg_data[0]) > 0.1f) || (abs(pos_data[1] - neg_data[1]) > 0.1f) || (abs(pos_data[2] - neg_data[2]) > 0.1f)) {
        BMI088_Data.BMI088_ERORR_CODE= BMI088_SELF_TEST_ACCEL_ERROR;
        log_e("Accel Self Test Failed!\n");
    }
}

void VerifyGyroSelfTest(void) {
    static uint8_t tmp;

    tmp=0X01;
    _bmi088_writedata(bmi_gyro_device,BMI088_GYRO_SELF_TEST,&tmp,1);
    DWT_Delay(0.1);
    uint8_t bist_rdy = 0x00, bist_fail;
    while (bist_rdy == 0) {
        _bmi088_readdata(bmi_gyro_device,BMI088_GYRO_SELF_TEST,&bist_rdy,1);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
    _bmi088_readdata(bmi_gyro_device,BMI088_GYRO_SELF_TEST,&bist_fail,1);
    bist_fail = (bist_fail & 0x04) >> 2;
    if (bist_fail != 0) {
        BMI088_Data.BMI088_ERORR_CODE= BMI088_SELF_TEST_GYRO_ERROR;
        log_e("gyro Self Test Failed!\n");
    }
}

/*加速度计部分*/
void bmi088_acc_init(void){
    //self check
    //id
    uint8_t whoami_check = 0;
    uint8_t tmp;
    // 加速度计以I2C模式启动,需要一次上升沿来切换到SPI模式,因此进行一次fake write
    _bmi088_readdata(bmi_acc_device, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    if (whoami_check != BMI088_ACC_CHIP_ID_VALUE){
        BMI088_Data.BMI088_ERORR_CODE= BMI088_NO_SENSOR;
        log_e("No acc Sensor!\n");
    }
    DWT_Delay(0.001);

    tmp=BMI088_ACC_SOFTRESET_VALUE;
    _bmi088_writedata(bmi_acc_device, BMI088_ACC_SOFTRESET, &tmp, 1); // 软复位
    DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME);
    
    VerifyAccSelfTest();
    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]); i++)
    {
        reg = BMI088_Accel_Init_Table[i][BMI088REG];
        data = BMI088_Accel_Init_Table[i][BMI088DATA];
        _bmi088_writedata(bmi_acc_device, reg, &data, 1);// 写入寄存器
        DWT_Delay(0.001);
        _bmi088_readdata(bmi_acc_device, reg, &tmp, 1);// 写完之后立刻读回检查
        DWT_Delay(0.001);
        if (tmp != BMI088_Accel_Init_Table[i][BMI088DATA])
        {    
            error |= BMI088_Accel_Init_Table[i][BMI088ERROR];
            BMI088_Data.BMI088_ERORR_CODE=error;
            log_e("ACCEL Init failed reg:%d\r\n",BMI088_Accel_Init_Table[i][BMI088REG]);
        }
        //{i--;} 可以设置retry次数,如果retry次数用完了,则返回error
    }
}

void bmi088_gyro_init(void){
    uint8_t tmp;
    
    tmp=BMI088_GYRO_SOFTRESET_VALUE;
    _bmi088_writedata(bmi_gyro_device, BMI088_GYRO_SOFTRESET, &tmp, 1);// 软复位
    DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查ID,如果不是0x0F(bmi088 whoami寄存器值),则返回错误
    uint8_t whoami_check = 0;
    _bmi088_readdata(bmi_gyro_device, BMI088_GYRO_CHIP_ID, &whoami_check, 1);
    if (whoami_check != BMI088_GYRO_CHIP_ID_VALUE)
    {
        BMI088_Data.BMI088_ERORR_CODE= BMI088_NO_SENSOR;
        log_e("No gyro Sensor!\n");
    }
    DWT_Delay(0.001);

    VerifyGyroSelfTest();

    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]); i++)
    {
        reg = BMI088_Gyro_Init_Table[i][BMI088REG];
        data = BMI088_Gyro_Init_Table[i][BMI088DATA];
        _bmi088_writedata(bmi_gyro_device, reg, &data, 1);
        DWT_Delay(0.001);
        _bmi088_readdata(bmi_gyro_device, reg, &tmp, 1);// 写完之后立刻读回对应寄存器检查是否写入成功
        DWT_Delay(0.001);
        if (tmp != BMI088_Gyro_Init_Table[i][BMI088DATA])
        {
            error |= BMI088_Gyro_Init_Table[i][BMI088ERROR];
            BMI088_Data.BMI088_ERORR_CODE=error;
            log_e("GYRO Init failed reg:%d\r\n",BMI088_Gyro_Init_Table[i][BMI088REG]);
        }
    }
}

void bmi088_imu_temp_init(void){
    HAL_TIM_Base_Start(&htim10);
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
}


void BMI088_data_acquire(void)
{
    static uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
    // 读取accel的x轴数据首地址,bmi088内部自增读取地址 // 3* sizeof(int16_t)
    _bmi088_readdata(bmi_acc_device, BMI088_ACCEL_XOUT_L, buf, 6);
    for (uint8_t i = 0; i < 3; i++)
        {BMI088_Data.acc[i] = (BMI088_ACCEL_6G_SEN) * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i])* BMI088_Data.AccelScale;}
    _bmi088_readdata(bmi_gyro_device, BMI088_GYRO_X_L, buf, 6); // 连续读取3个(3*2=6)轴的角速度
    for (uint8_t i = 0; i < 3; i++)
        {BMI088_Data.gyro[i] = BMI088_GYRO_2000_SEN * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]) - BMI088_Data.GyroOffset[i];}
    _bmi088_readdata(bmi_acc_device, BMI088_TEMP_M, buf, 2);// 读温度,温度传感器在accel上
    int16_t tmp = (((buf[0] << 3) | (buf[1] >> 5)));
    if (tmp > 1023)
    {
        tmp-=2048;
    }
    BMI088_Data.temperature = (float)(int16_t)tmp*BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

BMI088_GET_Data_t BMI088_GET_DATA(void){
    BMI088_data_acquire();
    BMI088_GET_Data_t data;
    data.acc =  (const float (*)[3])&BMI088_Data.acc;    // 使用类型转换确保类型匹配
    data.gyro = (const float (*)[3])&BMI088_Data.gyro;   // 使用类型转换确保类型匹配
    return data; 
}

void bmi088_get_accel(void)
{
    static uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
    // 读取accel的x轴数据首地址,bmi088内部自增读取地址 // 3* sizeof(int16_t)
    _bmi088_readdata(bmi_acc_device, BMI088_ACCEL_XOUT_L, buf, 6);
    for (uint8_t i = 0; i < 3; i++)
        {BMI088_Data.acc[i] = (BMI088_ACCEL_6G_SEN) * (float)((int16_t)((buf[2 * i + 1]) << 8) | buf[2 * i]);}

    _bmi088_readdata(bmi_acc_device, BMI088_TEMP_M, buf, 2);// 读温度,温度传感器在accel上
    int16_t tmp = (((buf[0] << 3) | (buf[1] >> 5)));
    if (tmp > 1023)
    {
        tmp-=2048;
    }
    BMI088_Data.temperature = (float)(int16_t)tmp*BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void bmi088_get_gyro(void)
{
    static uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
    _bmi088_readdata(bmi_gyro_device, BMI088_GYRO_X_L, buf, 6); // 连续读取3个(3*2=6)轴的角速度
    for (uint8_t i = 0; i < 3; i++)
        {BMI088_Data.gyro[i] = BMI088_GYRO_2000_SEN * (float)((int16_t)((buf[2 * i + 1]) << 8) | buf[2 * i]);}
}


void Calibrate_MPU_Offset(void)
{
    static float dt,t;
    static uint32_t dt_cnt;
    static uint16_t CaliTimes = 6000;
    float gyroMax[3], gyroMin[3];
    float gNormTemp=0.0f, gNormMax=0.0f, gNormMin=0.0f;

    RGB_show(LED_Yellow);
    bmi088_get_accel();

    while (1)
    {
        bmi088_get_accel();
        PIDCalculate(&BMI088_Data.imu_temp_pid, BMI088_Data.temperature, 40);
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, BMI088_Data.imu_temp_pid.Output);
        if (abs(40-BMI088_Data.temperature) < 0.5f)
        {
            log_i("temperature set finish\n");
            break;
        }
        dt = DWT_GetDeltaT(&dt_cnt);
        t+=dt;
        if (t > 12){t=0.0f; break;}
    }
    do
    {
        dt = DWT_GetDeltaT(&dt_cnt);
        t+=dt;
        if (t > 12)
        {

            BMI088_Data.GyroOffset[0] = GxOFFSET;
            BMI088_Data.GyroOffset[1] = GyOFFSET;
            BMI088_Data.GyroOffset[2] = GzOFFSET;
            BMI088_Data.gNorm = gNORM;
            BMI088_Data.TempWhenCali =40;
            log_e("Calibrate MPU Offset Failed!\n");
            RGB_show(LED_Red);
            break;
        }

        DWT_Delay(0.005);
        BMI088_Data.gNorm = 0;
        BMI088_Data.GyroOffset[0] = 0;
        BMI088_Data.GyroOffset[1] = 0;
        BMI088_Data.GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            PIDCalculate(&BMI088_Data.imu_temp_pid, BMI088_Data.temperature, 40);
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, BMI088_Data.imu_temp_pid.Output);

            bmi088_get_accel();
            gNormTemp = sqrtf(BMI088_Data.acc[0] * BMI088_Data.acc[0] +
                              BMI088_Data.acc[1] * BMI088_Data.acc[1] +
                              BMI088_Data.acc[2] * BMI088_Data.acc[2]);
                              BMI088_Data.gNorm += gNormTemp;

            bmi088_get_gyro();
            BMI088_Data.GyroOffset[0] += BMI088_Data.gyro[0];
            BMI088_Data.GyroOffset[1] += BMI088_Data.gyro[1];
            BMI088_Data.GyroOffset[2] += BMI088_Data.gyro[2];
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = BMI088_Data.gyro[j];
                    gyroMin[j] = BMI088_Data.gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (BMI088_Data.gyro[j] > gyroMax[j])
                        gyroMax[j] = BMI088_Data.gyro[j];
                    if (BMI088_Data.gyro[j] < gyroMin[j])
                        gyroMin[j] = BMI088_Data.gyro[j];
                }
            }
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                {break;}
            DWT_Delay(0.001);
        }

        BMI088_Data.gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; i++){BMI088_Data.GyroOffset[i] /= (float)CaliTimes;}

        //记录标定时的温度
        bmi088_get_accel();
        BMI088_Data.TempWhenCali =BMI088_Data.temperature;
        log_i("tempcali:%d\n",BMI088_Data.TempWhenCali);

    } while (gNormDiff > 0.5f ||
             fabsf(BMI088_Data.gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(BMI088_Data.GyroOffset[0]) > 0.01f ||
             fabsf(BMI088_Data.GyroOffset[1]) > 0.01f ||
             fabsf(BMI088_Data.GyroOffset[2]) > 0.01f);

    BMI088_Data.AccelScale = 9.81f / BMI088_Data.gNorm;

    uint8_t tmpdata[32]={0};
    memcpy(tmpdata,&BMI088_Data.GyroOffset[0],sizeof(BMI088_Data.GyroOffset));
    memcpy(tmpdata+4,&BMI088_Data.GyroOffset[1],sizeof(BMI088_Data.GyroOffset));
    memcpy(tmpdata+8,&BMI088_Data.GyroOffset[2],sizeof(BMI088_Data.GyroOffset));
    memcpy(tmpdata+12,&BMI088_Data.gNorm,sizeof(BMI088_Data.gNorm));
    memcpy(tmpdata+16,&BMI088_Data.TempWhenCali,sizeof(BMI088_Data.TempWhenCali));
    tmpdata[31]=0XAA;

    BSP_Flash_Erase(FLASH_SECTOR_11, 1);
    BSP_Flash_Write(ADDR_FLASH_SECTOR_11, tmpdata, sizeof(tmpdata));
    log_i("calibrate MPU offset finished!\n");
}


void bmi088_temp_ctrl(void) {
    PIDCalculate(&BMI088_Data.imu_temp_pid, BMI088_Data.temperature, 40);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, BMI088_Data.imu_temp_pid.Output);
}

void BMI088_init(void){
    static SPI_DeviceInstance_t gyro_cfg = {
        .target_bus = SPI_BUS1,
        .cs_port    = GPIOB,
        .cs_pin     = GPIO_PIN_0,
        .tx_mode    = BSP_SPI_MODE_BLOCKING,
        .rx_mode    = BSP_SPI_MODE_BLOCKING,
        .callback   = NULL,
        .timeout    = 1000
    };
    bmi_gyro_device = SPI_DeviceRegister(&gyro_cfg);
    static SPI_DeviceInstance_t acc_cfg = {
        .target_bus = SPI_BUS1,
        .cs_port    = GPIOA,
        .cs_pin     = GPIO_PIN_4,
        .tx_mode    = BSP_SPI_MODE_BLOCKING,
        .rx_mode    = BSP_SPI_MODE_BLOCKING,
        .callback   = NULL,
        .timeout    = 1000
      };
    bmi_acc_device = SPI_DeviceRegister(&acc_cfg);
    if (!bmi_acc_device || !bmi_gyro_device){}
    else{
        bmi088_acc_init();
        bmi088_gyro_init();
        bmi088_imu_temp_init();
        PID_Init_Config_s config = {.MaxOut = 5000,
                        .IntegralLimit = 2000,
                        .DeadBand = 0,
                        .Kp = 2000,
                        .Ki = 200,
                        .Kd = 0,
                        .Improve = 0x01}; // enable integratiaon limit
        PIDInit(&BMI088_Data.imu_temp_pid, &config);
        uint8_t tmpdata[32]={0};
        BSP_Flash_Read(ADDR_FLASH_SECTOR_11, tmpdata, sizeof(tmpdata));
        if (tmpdata[31]!=0XAA)
        {
            Calibrate_MPU_Offset();
        }
        else
        {
            float tmp[5] = {0.0f};
            memcpy(&tmp[0], tmpdata, sizeof(tmp[0]));
            memcpy(&tmp[1], tmpdata + 4, sizeof(tmp[1]));
            memcpy(&tmp[2], tmpdata + 8, sizeof(tmp[2]));
            memcpy(&tmp[3], tmpdata + 12, sizeof(tmp[3]));
            memcpy(&tmp[4],tmpdata+16,sizeof(tmp[4]));

            BMI088_Data.GyroOffset[0] = tmp[0];
            BMI088_Data.GyroOffset[1] = tmp[1];
            BMI088_Data.GyroOffset[2] = tmp[2];
            BMI088_Data.gNorm         = tmp[3];
            BMI088_Data.TempWhenCali  = tmp[4];
            BMI088_Data.AccelScale    = 9.81f / BMI088_Data.gNorm;
        }
    }

    if (BMI088_Data.BMI088_ERORR_CODE==BMI088_NO_ERROR) {
        log_i("BMI088 init success!\n");
    }
}
