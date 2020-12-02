/**
  ******************************************************************************
  * �ļ�����: bsp_BDCMOTOR.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-10-09
  * ��    ��: ��ˢֱ����������������������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "DCMotor/bsp_BDCMotor.h" 

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_BDCMOTOR;
__IO uint16_t PWM_Duty=BDCMOTOR_DUTY_ZERO;         // ռ�ձȣ�PWM_Duty/BDCMOTOR_TIM_PERIOD*100%
                                    // ռ�ձ�Ϊ��50%ʱ�����ת
                                    // ռ�ձȲ�Ϊ��50%ʱ�����ת����50%�ľ��Բ�Խ����ת�ٶ�Ҳ��
                                    // ��ת���򲻽�������йأ�Ҳ���������йأ���Ҫ�������
                                    // �򵥵ķ����ǣ�������Ʒ�����Ҫ���෴����������PWM�����߽ӷ�
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: BDCMOTOR���GPIO��ʼ������,�ú�����HAL���ڲ�����.
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  /* BDCMOTOR���GPIO��ʼ������ */
  if(htim == &htimx_BDCMOTOR)
  {
    GPIO_InitTypeDef GPIO_InitStruct; 
    /* ���Ŷ˿�ʱ��ʹ�� */
    BDCMOTOR_TIM_CH1_GPIO_CLK_ENABLE();
    BDCMOTOR_TIM_CH1N_GPIO_CLK_ENABLE();
    SHUTDOWN_GPIO_CLK_ENABLE();

    /* BDCMOTOR��������������IO��ʼ�� */
    GPIO_InitStruct.Pin = BDCMOTOR_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(BDCMOTOR_TIM_CH1_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = BDCMOTOR_TIM_CH1N_PIN;
    HAL_GPIO_Init(BDCMOTOR_TIM_CH1N_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = SHUTDOWN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(SHUTDOWN_PORT, &GPIO_InitStruct);
    
    /* ʹ�ܵ���������� */
    ENABLE_MOTOR();
  }
}

/**
  * ��������: BDCMOTOR��ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void BDCMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  
  /* ������ʱ������ʱ��ʹ�� */
  BDCMOTOR_TIM_RCC_CLK_ENABLE();
  
  /* ��ʱ�������������� */
  htimx_BDCMOTOR.Instance = BDCMOTOR_TIMx;                                 // ��ʱ�����
  htimx_BDCMOTOR.Init.Prescaler = BDCMOTOR_TIM_PRESCALER;                  // ��ʱ��Ԥ��Ƶ��
  htimx_BDCMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;                  // �����������ϼ���
  htimx_BDCMOTOR.Init.Period = BDCMOTOR_TIM_PERIOD;                        // ��ʱ������
  htimx_BDCMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // ʱ�ӷ�Ƶ
  htimx_BDCMOTOR.Init.RepetitionCounter = BDCMOTOR_TIM_REPETITIONCOUNTER;  // �ظ�������
  /* ��ʼ����ʱ���Ƚ�������� */
  HAL_TIM_PWM_Init(&htimx_BDCMOTOR);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_BDCMOTOR, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  // �Ƚ����ģʽ��PWM1ģʽ
  sConfigOC.Pulse =  PWM_Duty;                         // ռ�ձ�
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;        // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_PWM_ConfigChannel(&htimx_BDCMOTOR, &sConfigOC, TIM_CHANNEL_1);
}


/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BDCMOTOR_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    BDCMOTOR_TIM_RCC_CLK_DISABLE();
    
    HAL_GPIO_DeInit(BDCMOTOR_TIM_CH1_PORT,BDCMOTOR_TIM_CH1_PIN);
    HAL_GPIO_DeInit(BDCMOTOR_TIM_CH1N_PORT,BDCMOTOR_TIM_CH1N_PIN);
  }
} 

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
