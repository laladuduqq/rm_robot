#include "vcom.h"
#include "offline.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

#define LOG_TAG              "vcom"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>
 
vcom_receive_t vcom_receive;
void usb_callback(uint8_t* Buf, uint32_t Len)
{
    if (Len ==sizeof(struct Recv_s) && Buf[0]==0xFF && Buf[sizeof(struct Recv_s)-1] == 0x0D)
    {
        offline_device_update(vcom_receive.offline_index);
        memcpy(&vcom_receive.recv,Buf,sizeof(struct Recv_s));
    }
}


void vcom_init(void){
    OfflineDeviceInit_t offline_init = {
        .name = "minipc",
        .timeout_ms = 100,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 9,
        .enable = OFFLINE_DISABLE,
    };
    vcom_receive.offline_index = offline_device_register(&offline_init);
    // 注册回调函数
    USB_RegisterCallback(usb_callback);
}




