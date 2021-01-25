#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct              //测量高电平脉宽
{   
	uint8_t   ucFinishFlag;
  uint8_t   ucStartFlag;
  uint32_t  usLowPulse;
  uint32_t  usHighPulse;
	uint32_t  usCtr;
	uint32_t  usPeriod;

}STRUCT_CAPTURE;
/* 宏定义 --------------------------------------------------------------------*/
/********************通用定时器TIM参数定义，TIM2~TIM5************/
#define GENERAL_TIMx                     TIM2
#define GENERAL_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM2_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM2_CLK_DISABLE()
#define GENERAL_TIM_IRQ                  TIM2_IRQn
#define GENERAL_TIM_IRQHANDLER           TIM2_IRQHandler

// 定义定时器预分频，定时器实际时钟频率为：84MHz/（GENERAL_TIM_PRESCALER+1）
#define GENERAL_TIM_PRESCALER           0  
// 定义定时器周期，当定时器开始计数到BASIC_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define GENERAL_TIM_PERIOD              0xFFFFFFFF  // 定时器产生中断频率为：定时器频率/0xFFFF Hz

#define GENERAL_TIMX_GPIO_RCC_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE();
#define GENERAL_TIM_CH1_PORT                GPIOA
#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_15
#define GENERAL_TIMx_GPIO_AF                GPIO_AF1_TIM2

#define GENERAL_TIM_CHANNEL1                TIM_CHANNEL_1
#define GENERAL_TIM_IT_CC1                  TIM_IT_CC1

#define GENERAL_TIM_CHANNEL2                TIM_CHANNEL_2
#define GENERAL_TIM_IT_CC2                  TIM_IT_CC2

/* 根据设置的脉冲极性,测量脉宽 */
#define GENERAL_TIM_STRAT_ICPolarity        TIM_INPUTCHANNELPOLARITY_RISING         //测量的起始边沿
#define GENERAL_TIM_END_ICPolarity          TIM_INPUTCHANNELPOLARITY_FALLING          //测量的结束边沿

/*
 *      +-------------+
 *      |             |
 *    +-+             +----+
 *      ^             ^
 *      |             |
 *Start Polarity    End Polarity
 *
 * PWM Frequency 1 KHz
 */
/* 以下参数详情看PWM 接口通信协议 */
#define PWM_PERIOD        4119  // PWM 脉冲周期 4119 bit
#define PWM_EXIT_CLK      8     // Exit 8 clocks 
#define PWM_ZERO_DEG_CLK  16    // Zero degree 16 clocks
#define PWM_DATA_CLK      4095  // Data 4095 clocks
#define PWM_UNUSEDCLK   (PWM_ZERO_DEG_CLK+PWM_EXIT_CLK)// unused clocks
#define ANGLE_RESOLUTION  (360.0f/4095.0f);
/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_GeneralTIM;
/* 函数声明 ------------------------------------------------------------------*/

void GENERAL_TIMx_Init(void);
float API_GetPWMDuty(TIM_HandleTypeDef* htim);
float API_GetMotorAngle(TIM_HandleTypeDef* htim);
#endif	/* __GENERAL_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
