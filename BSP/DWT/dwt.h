#ifndef __DWT_H
#define __DWT_H
#include <stdint.h>

void DWT_Init(void);
float DWT_GetDeltaT(uint32_t *cnt_last);
void DWT_Delay(float Delay); //单位为s
#endif
