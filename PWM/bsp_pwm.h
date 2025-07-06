/**
 * @file bsp_pwm.h
 * @author zhan xile
 * @brief 对pwm进行封库
 * @version 0.1
 * @date 2025-07-2
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "stdint.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f407xx.h"
#define PWM_DEVICE_CNT 16 // 支持的最大PWM设备数量
/* pwm实例结构体 */
typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t tclk;                           // 时钟频率
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(struct pwm_ins_temp *); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWMInstance;

typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(PWMInstance*); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWM_Init_Config_s;


/**
 * @brief 注册一个pwm实例
 *
 * @param config 初始化配置
 * @return PWMInstance*
 */
PWMInstance *PWMRegister(PWM_Init_Config_s *config);


void PWMStart(PWMInstance *pwm);//启动pwm
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);//设置pwm占空比
void PWMStop(PWMInstance *pwm);//停止pwm
void PWMSetPeriod(PWMInstance *pwm, float period);//设置pwm周期
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);//启动pwm dma传输
