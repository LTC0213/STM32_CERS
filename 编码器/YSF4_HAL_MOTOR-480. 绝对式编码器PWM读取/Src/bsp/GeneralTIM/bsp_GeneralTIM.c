/**
  ******************************************************************************
  * �ļ�����: bsp_BasicTIM.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2018-04-12
  * ��    ��: ����ʽ������PWMռ�ձȶ�ȡ
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
#include "GeneralTIM/bsp_GeneralTIM.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_GeneralTIM;
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ��ʱ��ͨ��1���ų�ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base == &htimx_GeneralTIM)
  {
    /* ������ʱ������ʱ��ʹ�� */
    GENERAL_TIMX_GPIO_RCC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GENERAL_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;    
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GENERAL_TIMx_GPIO_AF;
    HAL_GPIO_Init(GENERAL_TIM_CH1_PORT, &GPIO_InitStruct);
    
    /* ���ö�ʱ���ж����ȼ���ʹ�� */
//    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
//    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
  }
}
/**
  * ��������: ������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void GENERAL_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sslaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
 
  /* ������ʱ������ʱ��ʹ�� */  
  GENERAL_TIM_RCC_CLK_ENABLE();
  
  htimx_GeneralTIM.Instance = GENERAL_TIMx;
  htimx_GeneralTIM.Init.Prescaler = GENERAL_TIM_PRESCALER;
  htimx_GeneralTIM.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx_GeneralTIM.Init.Period = GENERAL_TIM_PERIOD;
  htimx_GeneralTIM.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htimx_GeneralTIM);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htimx_GeneralTIM, &sClockSourceConfig);

  /* ���ö�ʱ��Ϊ��ģʽ,IT1�������¼����� */
  sslaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sslaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sslaveConfig.TriggerFilter = 0;
  sslaveConfig.TriggerPolarity = GENERAL_TIM_STRAT_ICPolarity;
  sslaveConfig.TriggerPrescaler = 0;
  HAL_TIM_SlaveConfigSynchronization(&htimx_GeneralTIM,&sslaveConfig);
  
  sConfigIC.ICPolarity = GENERAL_TIM_STRAT_ICPolarity;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htimx_GeneralTIM, &sConfigIC, GENERAL_TIM_CHANNEL1);
  
  sConfigIC.ICPolarity = GENERAL_TIM_END_ICPolarity;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htimx_GeneralTIM, &sConfigIC, GENERAL_TIM_CHANNEL2);
  
  /* ������ʱ��ͨ�����벶��*/
  HAL_TIM_IC_Start(&htimx_GeneralTIM,GENERAL_TIM_CHANNEL1);
  HAL_TIM_IC_Start(&htimx_GeneralTIM,GENERAL_TIM_CHANNEL2);
}

/**
  * ��������: ��ȡPWM��ռ�ձ�
  * �������: htim |��ʱ�����,���ӱ������Ķ�ʱ��
  * �� �� ֵ: -1  | ���ִ��� 
  *           0< return <100 |  ����ռ�ձ�
  * ˵    ��: ��ȡ��ʱ���ıȽϼĴ���,����ռ�ձ�,
  */
float API_GetPWMDuty(TIM_HandleTypeDef* htim)
{
  uint32_t capPeriod = 0;
  uint32_t capPulseWidth = 0;
  uint32_t capDuty = 0;
  uint32_t unitCLK = 0; // ʱ����Ԫ
  /* ��ȡ���񵽵����ں������� */
  capPeriod = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1;
  capPulseWidth = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
  
  /*------------------------------*/
  /* ���㵥��ʱ������ */
  unitCLK = capPeriod / PWM_PERIOD;  
  if(capPulseWidth < (PWM_ZERO_DEG_CLK * unitCLK))
    return -1;
  else
  {
    /* �õ�PWM���� */
    capPeriod = capPeriod - (unitCLK * PWM_UNUSEDCLK); 
    /* �õ�PWM������ */
    capPulseWidth = capPulseWidth - (unitCLK * PWM_ZERO_DEG_CLK);
    /* �õ�ռ�ձ� */
    capDuty =(float )(capPulseWidth) / (float )(capPeriod) * 1000.0f ; 
    return (float)capDuty/10.0f; 
  }
}
/**
  * ��������: ��ȡPWM��ռ�ձ�
  * �������: htim |��ʱ�����,���ӱ������Ķ�ʱ��
  * �� �� ֵ:  0< return <360 |  �����Ƕ�
  * ˵    ��: ��ȡռ�ձ�,����Ƕ�,
  */
float API_GetMotorAngle(TIM_HandleTypeDef* htim)
{
  float Angle = 0;
  float Duty = API_GetPWMDuty(htim);
  Angle = Duty * 3.6f;
  return Angle;
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
