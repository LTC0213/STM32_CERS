#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define XENCODER_TIMx                        TIM2
#define XENCODER_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM2_CLK_ENABLE()
#define XENCODER_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM2_CLK_DISABLE()

#define YENCODER_TIMx                        TIM3
#define YENCODER_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM3_CLK_ENABLE()
#define YENCODER_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM3_CLK_DISABLE()

#define XENCODER_TIM_GPIO_CLK_ENABLE()       {__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();}
#define XENCODER_TIM_CH1_PIN                 GPIO_PIN_15
#define XENCODER_TIM_CH1_GPIO                GPIOA
#define XENCODER_TIM_CH2_PIN                 GPIO_PIN_3
#define XENCODER_TIM_CH2_GPIO                GPIOB

#define YENCODER_TIM_GPIO_CLK_ENABLE()       {__HAL_RCC_GPIOC_CLK_ENABLE();}
#define YENCODER_TIM_CH1_PIN                 GPIO_PIN_6
#define YENCODER_TIM_CH1_GPIO                GPIOC
#define YENCODER_TIM_CH2_PIN                 GPIO_PIN_7
#define YENCODER_TIM_CH2_GPIO                GPIOC

#define XENCODER_TIM_GPIO_AF                 GPIO_AF1_TIM2

#define YENCODER_TIM_GPIO_AF                 GPIO_AF2_TIM3

#define XTIM_ENCODERMODE_TIx                 TIM_ENCODERMODE_TI12

#define YTIM_ENCODERMODE_TIx                 TIM_ENCODERMODE_TI12

#define XENCODER_TIM_IRQn                    TIM2_IRQn
#define XENCODER_TIM_IRQHANDLER              TIM2_IRQHandler

#define YENCODER_TIM_IRQn                    TIM3_IRQn
#define YENCODER_TIM_IRQHANDLER              TIM3_IRQHandler

// 定义定时器预分频，定时器实际时钟频率为：84MHz/（XENCODER_TIMx_PRESCALER+1）
#define XENCODER_TIM_PRESCALER               0  // 

#define YENCODER_TIM_PRESCALER               0  // 

// 使用32bits 的计数器作为编码器计数,F4系列的TIM2,TIM5
// 定义定时器周期，当定时器开始计数到XENCODER_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define XENCODER_TIM_PERIOD                  0xFFFFFFFF
#define XCNT_MAX                             4294967296

#define YENCODER_TIM_PERIOD                  0xFFFF
#define YCNT_MAX                             65535

// 使用16bits 的计数器作为编码器计数,F4系列的TIM3,TIM4
//#define XENCODER_TIM_PERIOD                0xFFFF
//#define XCNT_MAX                           65535

#define XENCODER_LINE                        1000

#define YENCODER_LINE                        1024

#if (XTIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI12)
  #define XENCODER_RESOLUTION                (4*XENCODER_LINE)//4倍频,同时使用CH1,CH2
#else 
  #if ((XTIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI2)||(XTIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI1))
    #define XENCODER_RESOLUTION              (2*XENCODER_LINE)//2倍频,只使用CH1或者CH2
  #else 
    #error " NOT Init @TIM_Encoder:Parameter error"
  #endif
#endif

#if (YTIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI12)
  #define YENCODER_RESOLUTION                (4*YENCODER_LINE)//4倍频,同时使用CH1,CH2
#else 
  #if ((YTIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI2)||(YTIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI1))
    #define YENCODER_RESOLUTION              (2*YENCODER_LINE)//2倍频,只使用CH1或者CH2
  #else 
    #error " NOT Init @TIM_Encoder:Parameter error"
  #endif
#endif

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef xhtimx_Encoder;
extern int32_t XOverflowCount ;//定时器溢出次数

extern TIM_HandleTypeDef yhtimx_Encoder;
extern int32_t YOverflowCount ;//定时器溢出次数
/* 函数声明 ------------------------------------------------------------------*/
void XENCODER_TIMx_Init(void);

void YENCODER_TIMx_Init(void);

#endif	/* __ENCODER_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
