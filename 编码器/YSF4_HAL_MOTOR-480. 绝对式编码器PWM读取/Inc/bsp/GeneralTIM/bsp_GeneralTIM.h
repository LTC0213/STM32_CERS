#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct              //�����ߵ�ƽ����
{   
	uint8_t   ucFinishFlag;
  uint8_t   ucStartFlag;
  uint32_t  usLowPulse;
  uint32_t  usHighPulse;
	uint32_t  usCtr;
	uint32_t  usPeriod;

}STRUCT_CAPTURE;
/* �궨�� --------------------------------------------------------------------*/
/********************ͨ�ö�ʱ��TIM�������壬TIM2~TIM5************/
#define GENERAL_TIMx                     TIM2
#define GENERAL_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM2_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM2_CLK_DISABLE()
#define GENERAL_TIM_IRQ                  TIM2_IRQn
#define GENERAL_TIM_IRQHANDLER           TIM2_IRQHandler

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��84MHz/��GENERAL_TIM_PRESCALER+1��
#define GENERAL_TIM_PRESCALER           0  
// ���嶨ʱ�����ڣ�����ʱ����ʼ������BASIC_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define GENERAL_TIM_PERIOD              0xFFFFFFFF  // ��ʱ�������ж�Ƶ��Ϊ����ʱ��Ƶ��/0xFFFF Hz

#define GENERAL_TIMX_GPIO_RCC_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE();
#define GENERAL_TIM_CH1_PORT                GPIOA
#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_15
#define GENERAL_TIMx_GPIO_AF                GPIO_AF1_TIM2

#define GENERAL_TIM_CHANNEL1                TIM_CHANNEL_1
#define GENERAL_TIM_IT_CC1                  TIM_IT_CC1

#define GENERAL_TIM_CHANNEL2                TIM_CHANNEL_2
#define GENERAL_TIM_IT_CC2                  TIM_IT_CC2

/* �������õ����弫��,�������� */
#define GENERAL_TIM_STRAT_ICPolarity        TIM_INPUTCHANNELPOLARITY_RISING         //��������ʼ����
#define GENERAL_TIM_END_ICPolarity          TIM_INPUTCHANNELPOLARITY_FALLING          //�����Ľ�������

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
/* ���²������鿴PWM �ӿ�ͨ��Э�� */
#define PWM_PERIOD        4119  // PWM �������� 4119 bit
#define PWM_EXIT_CLK      8     // Exit 8 clocks 
#define PWM_ZERO_DEG_CLK  16    // Zero degree 16 clocks
#define PWM_DATA_CLK      4095  // Data 4095 clocks
#define PWM_UNUSEDCLK   (PWM_ZERO_DEG_CLK+PWM_EXIT_CLK)// unused clocks
#define ANGLE_RESOLUTION  (360.0f/4095.0f);
/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_GeneralTIM;
/* �������� ------------------------------------------------------------------*/

void GENERAL_TIMx_Init(void);
float API_GetPWMDuty(TIM_HandleTypeDef* htim);
float API_GetMotorAngle(TIM_HandleTypeDef* htim);
#endif	/* __GENERAL_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
