#ifndef LED_H
#define LED_H

#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"


typedef struct {
    uint32_t t1;  // LED1闪烁频率(ms)
    uint32_t t2;  // LED2闪烁频率(ms)
    uint32_t t3;  // LED3闪烁频率(ms)
} LED_FreqConfig;


void StartTask02(void const * argument);
void led3_task(void const * argument);
void led2_task(void const * argument);

// 新增频率改变函数
void LED_InitFrequencies(uint32_t t1, uint32_t t2, uint32_t t3); // 初始化频率

#endif

