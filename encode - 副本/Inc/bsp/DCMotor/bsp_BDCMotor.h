#ifndef __BDCMOTOR_TIM_H__
#define __BDCMOTOR_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define BDCMOTOR_TIMx                         TIM1
#define BDCMOTOR_TIM_RCC_CLK_ENABLE()         __HAL_RCC_TIM1_CLK_ENABLE()
#define BDCMOTOR_TIM_RCC_CLK_DISABLE()        __HAL_RCC_TIM1_CLK_DISABLE()

#define BDCMOTOR_TIM_CH1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()     // ���PWM���������������ĵ�IN����
#define BDCMOTOR_TIM_CH1_PORT                 GPIOA                            // CH1��CH1N������������ʹ��
#define BDCMOTOR_TIM_CH1_PIN                  GPIO_PIN_8                       // ������������������OUT1��OUT2������
#define BDCMOTOR_TIM_CH1N_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()     // CH1��CH1N��Ӧ����IN1��IN2
#define BDCMOTOR_TIM_CH1N_PORT                GPIOB                            // ������������������OUT3��OUT4������
#define BDCMOTOR_TIM_CH1N_PIN                 GPIO_PIN_13                      // CH1��CH1N��Ӧ����IN3��IN4

#define SHUTDOWN_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1��CH1N��Ӧ����IN1��IN2
#define SHUTDOWN_PORT                         GPIOH                            // ������������������OUT3��OUT4������
#define SHUTDOWN_PIN                          GPIO_PIN_6                       // CH1��CH1N��Ӧ����IN3��IN4

#define ENABLE_MOTOR()                        HAL_GPIO_WritePin(SHUTDOWN_PORT,SHUTDOWN_PIN,GPIO_PIN_SET)
#define SHUTDOWN_MOTOR()                      HAL_GPIO_WritePin(SHUTDOWN_PORT,SHUTDOWN_PIN,GPIO_PIN_RESET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��BDCMOTOR_TIMx_PRESCALER+1��
#define BDCMOTOR_TIM_PRESCALER               1    // ʵ��ʱ��Ƶ��Ϊ��84MHz

// ���嶨ʱ�����ڣ�PWMƵ��Ϊ��168MHz/��BDCMOTOR_TIMx_PRESCALER+1��/��BDCMOTOR_TIM_PERIOD+1��
#define BDCMOTOR_TIM_PERIOD                  3999  // PWMƵ��Ϊ84MHz/(3999+1)=21KHz

#define BDCMOTOR_DUTY_ZERO                   ((BDCMOTOR_TIM_PERIOD+1)>>1)       // 0%ռ�ձ�
#define BDCMOTOR_DUTY_FULL                   (BDCMOTOR_TIM_PERIOD-100)            // 100%ռ�ձ�

// ����߼���ʱ���ظ������Ĵ���ֵ
// ʵ��PWMƵ��Ϊ��168MHz/��BDCMOTOR_TIMx_PRESCALER+1��/��BDCMOTOR_TIM_PERIOD+1��/��BDCMOTOR_TIM_REPETITIONCOUNTER+1��
#define BDCMOTOR_TIM_REPETITIONCOUNTER       0

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_BDCMOTOR;
extern __IO uint16_t PWM_Duty;

/* �������� ------------------------------------------------------------------*/

void BDCMOTOR_TIMx_Init(void);

#endif	/* __BDCMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
