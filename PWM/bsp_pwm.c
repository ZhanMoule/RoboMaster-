#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "bsp_pwm.h"
static uint8_t idx = 0;  // 当前PWM实例索引
static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL}; // PWM实例指针数组

/**
 * @brief 获取定时器时钟频率
 * @param htim TIM句柄指针
 * @return uint32_t 时钟频率(Hz)
 */
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim);

/**
 * @brief PWM DMA传输完成中断回调函数
 * @param htim 触发中断的TIM句柄
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // 遍历所有PWM实例查找匹配的中断源
    for (uint8_t i = 0; i < idx; i++) {
        // 检查定时器和通道是否匹配
        if (pwm_instance[i]->htim == htim && 
            htim->Channel == (1 << (pwm_instance[i]->channel / 4))) {
            // 调用用户设置的回调函数
            if (pwm_instance[i]->callback) {
                pwm_instance[i]->callback(pwm_instance[i]);
            }
            return;  // 一次中断只处理一个通道
        }
    }
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    // 检查实例数量是否超过限制
    if (idx >= PWM_DEVICE_CNT) {
        return NULL;  // 返回NULL而不是死循环，更友好
    }
    
    // 分配内存并初始化
    PWMInstance *pwm = (PWMInstance *)malloc(sizeof(PWMInstance));
    if (pwm == NULL) {
        return NULL;  // 内存分配失败
    }
    memset(pwm, 0, sizeof(PWMInstance));
    
    // 填充实例参数
    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->dutyratio = config->dutyratio;
    pwm->callback = config->callback;
    pwm->id = config->id;
    pwm->tclk = PWMSelectTclk(pwm->htim);
    
    // 配置PWM参数
    PWMSetPeriod(pwm, pwm->period);
    PWMSetDutyRatio(pwm, pwm->dutyratio);
    
    // 启动PWM
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    
    // 保存实例指针
    pwm_instance[idx++] = pwm;
    
    return pwm;
}

void PWMStart(PWMInstance *pwm)
{
    if (pwm != NULL) {
        HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    }
}

void PWMStop(PWMInstance *pwm)
{
    if (pwm != NULL) {
        HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
    }
}

void PWMSetPeriod(PWMInstance *pwm, float period)
{
    if (pwm == NULL || pwm->htim == NULL) return;
    
    // 计算ARR值: ARR = (周期 * 时钟频率) / (预分频 + 1)
    uint32_t arr = (uint32_t)(period * (pwm->tclk / (pwm->htim->Init.Prescaler + 1)));
    __HAL_TIM_SetAutoreload(pwm->htim, arr);
    pwm->period = period;
    
    // 保持占空比不变，更新比较值
    PWMSetDutyRatio(pwm, pwm->dutyratio);
}

void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    if (pwm == NULL || pwm->htim == NULL) return;
    
    // 限制占空比范围
    if (dutyratio < 0.0f) dutyratio = 0.0f;
    if (dutyratio > 1.0f) dutyratio = 1.0f;
    
    // 计算比较值: CCR = 占空比 * ARR
    uint32_t ccr = (uint32_t)(dutyratio * __HAL_TIM_GetAutoreload(pwm->htim));
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, ccr);
    pwm->dutyratio = dutyratio;
}

void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size)
{
    if (pwm != NULL && pwm->htim != NULL) {
        HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
    }
}

static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) return 0;
    
    uintptr_t tclk_temp = (uintptr_t)(htim->Instance);
    
    // 判断定时器属于APB1还是APB2总线
    if ((tclk_temp >= APB1PERIPH_BASE) && 
        (tclk_temp <= (APB1PERIPH_BASE + 0x2000UL))) {
        // APB1总线定时器(TIM2~7,12~14)
        uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        uint32_t presc = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
        return pclk * (APBPrescTable[presc] == 0 ? 1 : 2);
    } 
    else if (((tclk_temp >= APB2PERIPH_BASE) && 
             (tclk_temp <= (APB2PERIPH_BASE + 0x0400UL))) ||
            ((tclk_temp >= (APB2PERIPH_BASE + 0x4000UL)) && 
             (tclk_temp <= (APB2PERIPH_BASE + 0x4800UL)))) {
        // APB2总线定时器(TIM1,8~11)
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();
        uint32_t presc = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
        return pclk * (APBPrescTable[presc] == 0 ? 1 : 2);
    }
    
    return 0;  // 未知定时器
}

