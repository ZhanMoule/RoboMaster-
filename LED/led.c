#include "led.h"
#include <stdio.h>


// 默认频率配置
LED_FreqConfig led_freq_config = {
    .t1 = 500,  // 红灯默认500ms
    .t2 = 500,  // 绿灯默认500ms
    .t3 = 500   // 蓝灯默认500ms
};

void StartTask02(void const * argument)//点亮红灯的函数
{
	uint32_t current_delay = led_freq_config.t1;
	while (1)
		{
			HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
			osDelay(current_delay);
		}
}

void led2_task(void const * argument)//点亮绿灯函数
{
	uint32_t current_delay = led_freq_config.t2;
	while (1)
		{
			HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
			osDelay(current_delay);
		}
}

void led3_task(void const * argument)//点亮蓝灯的函数
{
	uint32_t current_delay = led_freq_config.t3;
	while (1)
		{
			HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_13);
			osDelay(current_delay);
		}
}
//初始化led闪烁频率
void LED_InitFrequencies(uint32_t t1, uint32_t t2, uint32_t t3)
{
    led_freq_config.t1 = t1;
    led_freq_config.t2 = t2;
    led_freq_config.t3 = t3;
}


