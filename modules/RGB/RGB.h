#ifndef __RGB_H
#define __RGB_H

#include <stdint.h>

#define LED_Black 0XFF000000
#define LED_White 0XFFFFFFFF
#define LED_Red   0XFFFF0000
#define LED_Green 0XFF00FF00
#define LED_Blue  0XFF0000FF
#define LED_Yellow 0XFFFFFF00

void RGB_init(void);
void RGB_show(uint32_t aRGB);

#endif
