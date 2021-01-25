/**
  ******************************************************************************
  * 文件名程: bsp_BasicTIM.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2018-04-12
  * 功    能: 绝对式编码器PWM占空比读取
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "GeneralTIM/bsp_GeneralTIM.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_GeneralTIM;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 定时器通道1引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base == &htimx_GeneralTIM)
  {
    /* 基本定时器外设时钟使能 */
    GENERAL_TIMX_GPIO_RCC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GENERAL_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;    
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GENERAL_TIMx_GPIO_AF;
    HAL_GPIO_Init(GENERAL_TIM_CH1_PORT, &GPIO_InitStruct);
    
    /* 配置定时器中断优先级并使能 */
//    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
//    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
  }
}
/**
  * 函数功能: 基本定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void GENERAL_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sslaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
 
  /* 基本定时器外设时钟使能 */  
  GENERAL_TIM_RCC_CLK_ENABLE();
  
  htimx_GeneralTIM.Instance = GENERAL_TIMx;
  htimx_GeneralTIM.Init.Prescaler = GENERAL_TIM_PRESCALER;
  htimx_GeneralTIM.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx_GeneralTIM.Init.Period = GENERAL_TIM_PERIOD;
  htimx_GeneralTIM.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htimx_GeneralTIM);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htimx_GeneralTIM, &sClockSourceConfig);

  /* 配置定时器为从模式,IT1触发更新计数器 */
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
  
  /* 启动定时器通道输入捕获*/
  HAL_TIM_IC_Start(&htimx_GeneralTIM,GENERAL_TIM_CHANNEL1);
  HAL_TIM_IC_Start(&htimx_GeneralTIM,GENERAL_TIM_CHANNEL2);
}

/**
  * 函数功能: 获取PWM的占空比
  * 输入参数: htim |定时器句柄,连接编码器的定时器
  * 返 回 值: -1  | 出现错误 
  *           0< return <100 |  正常占空比
  * 说    明: 读取定时器的比较寄存器,计算占空比,
  */
float API_GetPWMDuty(TIM_HandleTypeDef* htim)
{
  uint32_t capPeriod = 0;
  uint32_t capPulseWidth = 0;
  uint32_t capDuty = 0;
  uint32_t unitCLK = 0; // 时基单元
  /* 读取捕获到的周期和正脉宽 */
  capPeriod = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1;
  capPulseWidth = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
  
  /*------------------------------*/
  /* 计算单个时钟周期 */
  unitCLK = capPeriod / PWM_PERIOD;  
  if(capPulseWidth < (PWM_ZERO_DEG_CLK * unitCLK))
    return -1;
  else
  {
    /* 得到PWM周期 */
    capPeriod = capPeriod - (unitCLK * PWM_UNUSEDCLK); 
    /* 得到PWM正脉宽 */
    capPulseWidth = capPulseWidth - (unitCLK * PWM_ZERO_DEG_CLK);
    /* 得到占空比 */
    capDuty =(float )(capPulseWidth) / (float )(capPeriod) * 1000.0f ; 
    return (float)capDuty/10.0f; 
  }
}
/**
  * 函数功能: 获取PWM的占空比
  * 输入参数: htim |定时器句柄,连接编码器的定时器
  * 返 回 值:  0< return <360 |  正常角度
  * 说    明: 读取占空比,计算角度,
  */
float API_GetMotorAngle(TIM_HandleTypeDef* htim)
{
  float Angle = 0;
  float Duty = API_GetPWMDuty(htim);
  Angle = Duty * 3.6f;
  return Angle;
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
