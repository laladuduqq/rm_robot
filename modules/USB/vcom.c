#include "vcom.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"

#define LOG_TAG              "vcom"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

static struct Nav_Recv_s nav_recv;
static struct Vision_Recv_s vision_recv;

void usb_callback(uint8_t* Buf, uint32_t Len)
{
    if (Len ==sizeof(struct Nav_Recv_s) && Buf[0]==0xBB && Buf[sizeof(struct Nav_Recv_s)-1] == 0x55)
    {
        memcpy(&nav_recv,Buf,sizeof(struct Nav_Recv_s));
    }
    if (Len ==sizeof(struct Vision_Recv_s) && Buf[0]==0xAA && Buf[sizeof(struct Vision_Recv_s)-1] == 0x55) {
        memcpy(&vision_recv,Buf,sizeof(struct Vision_Recv_s));
    }
}


void vcom_init(void){
    // 注册回调函数
    USB_RegisterCallback(usb_callback);
}


void Get_data(struct Nav_Recv_s *nav_recv_user,struct Vision_Recv_s *vision_recv_user)
{
    if (nav_recv_user!=NULL)
    {
        memcpy(nav_recv_user,&nav_recv,sizeof(struct Nav_Recv_s));
    }
    if (vision_recv_user!=NULL)
    {
        memcpy(vision_recv_user,&vision_recv,sizeof(struct Vision_Recv_s));
    }
}


