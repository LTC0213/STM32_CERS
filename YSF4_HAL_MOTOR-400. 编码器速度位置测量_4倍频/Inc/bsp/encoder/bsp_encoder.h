#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
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

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��84MHz/��ENCODER_TIMx_PRESCALER+1��
#define ENCODER_TIM_PRESCALER               0  // 

// ʹ��32bits �ļ�������Ϊ����������,F4ϵ�е�TIM2,TIM5
// ���嶨ʱ�����ڣ�����ʱ����ʼ������ENCODER_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define ENCODER_TIM_PERIOD                  0xFFFFFFFF
#define CNT_MAX                             4294967296
// ʹ��16bits �ļ�������Ϊ����������,F4ϵ�е�TIM3,TIM4
//#define ENCODER_TIM_PERIOD                0xFFFF
//#define CNT_MAX                           65535

#define ENCODER_LINE                        600
#if (TIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI12)
  #define ENCODER_RESOLUTION                (4*ENCODER_LINE)//4��Ƶ,ͬʱʹ��CH1,CH2
#else 
  #if ((TIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI2)||(TIM_ENCODERMODE_TIx==TIM_ENCODERMODE_TI1))
    #define ENCODER_RESOLUTION              (2*ENCODER_LINE)//2��Ƶ,ֻʹ��CH1����CH2
  #else 
    #error " NOT Init @TIM_Encoder:Parameter error"
  #endif
#endif
/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_Encoder;
extern int32_t OverflowCount ;//��ʱ���������
/* �������� ------------------------------------------------------------------*/
void ENCODER_TIMx_Init(void);

#endif	/* __ENCODER_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
