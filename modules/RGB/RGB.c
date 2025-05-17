#include "RGB.h"
#include "tim.h"


void RGB_init(void)
{
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
}

/**
  * @brief          显示RGB
  * @param      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
void RGB_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;
    /* 设置PWM周期和脉冲宽度默认值 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, red);
}

