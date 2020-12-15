#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define ENCODER_TIMx                        TIM2
#define ENCODER_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM2_CLK_ENABLE()
#define ENCODER_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM2_CLK_DISABLE()

#define ENCODER_TIM_GPIO_CLK_ENABLE()       {__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();}
#define ENCODER_TIM_CH1_PIN                 GPIO_PIN_15
#define ENCODER_TIM_CH1_GPIO                GPIOA
#define ENCODER_TIM_CH2_PIN                 GPIO_PIN_3
#define ENCODER_TIM_CH2_GPIO                GPIOB

#define ENCODER_TIM_GPIO_AF                 GPIO_AF1_TIM2

#define TIM_ENCODERMODE_TIx                 TIM_ENCODERMODE_TI12

#define ENCODER_TIM_IRQn                    TIM2_IRQn
#define ENCODER_TIM_IRQHANDLER              TIM2_IRQHandler

// 定义定时器预分频，定时器实际时钟频率为：84MHz/（ENCODER_TIMx_PRESCALER+1）
#define ENCODER_TIM_PRESCALER               0  // 

// 使用32bits 的计数器作为编码器计数,F4系列的TIM2,TIM5
// 定义定时器周期，当定时器开始计数到ENCODER_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define ENCODER_TIM_PERIOD                  0xFFFFFFFF
#define CNT_MAX                             4294967296
// 使用16bits 的计数器作为编码器计数,F4系列的TIM3,TIM4
//#define ENCODER_TIM_PERIOD                0xFFFF
//#define CNT_MAX                           65535

#define ENCODER_LINE                        600
#if (TIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI12)
  #define ENCODER_RESOLUTION                (4*ENCODER_LINE)//4倍频,同时使用CH1,CH2
#else 
  #if ((TIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI2)||(TIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI1))
    #define ENCODER_RESOLUTION              (2*ENCODER_LINE)//2倍频,只使用CH1或者CH2
  #else 
    #error " NOT Init @TIM_Encoder:Parameter error"
  #endif
#endif
/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_Encoder;
extern int32_t OverflowCount ;//定时器溢出次数
/* 函数声明 ------------------------------------------------------------------*/
void ENCODER_TIMx_Init(void);

#endif	/* __ENCODER_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
