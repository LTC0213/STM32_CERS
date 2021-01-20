/**
  ******************************************************************************
  * 文件名程: bsp_BDCMOTOR.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-10-09
  * 功    能: 有刷直流电机驱动板基本驱动程序
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
#include "DCMotor/bsp_BDCMotor.h" 

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_BDCMOTOR;
__IO uint16_t PWM_Duty=BDCMOTOR_DUTY_ZERO;         // 占空比：PWM_Duty/BDCMOTOR_TIM_PERIOD*100%
                                    // 占空比为：50%时电机不转
                                    // 占空比不为：50%时电机旋转，与50%的绝对差越大旋转速度也高
                                    // 旋转方向不仅与程序有关，也与电机接线有关，需要具体分析
                                    // 简单的方法是：如果控制方向与要求相反，调换两根PWM控制线接法
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: BDCMOTOR相关GPIO初始化配置,该函数被HAL库内部调用.
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  /* BDCMOTOR相关GPIO初始化配置 */
  if(htim == &htimx_BDCMOTOR)
  {
    GPIO_InitTypeDef GPIO_InitStruct; 
    /* 引脚端口时钟使能 */
    BDCMOTOR_TIM_CH1_GPIO_CLK_ENABLE();
    BDCMOTOR_TIM_CH1N_GPIO_CLK_ENABLE();
    SHUTDOWN_GPIO_CLK_ENABLE();

    /* BDCMOTOR输出脉冲控制引脚IO初始化 */
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
    
    /* 使能电机控制引脚 */
    ENABLE_MOTOR();
  }
}

/**
  * 函数功能: BDCMOTOR定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void BDCMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // 定时器时钟
  TIM_OC_InitTypeDef sConfigOC;                          // 定时器通道比较输出
  
  /* 基本定时器外设时钟使能 */
  BDCMOTOR_TIM_RCC_CLK_ENABLE();
  
  /* 定时器基本环境配置 */
  htimx_BDCMOTOR.Instance = BDCMOTOR_TIMx;                                 // 定时器编号
  htimx_BDCMOTOR.Init.Prescaler = BDCMOTOR_TIM_PRESCALER;                  // 定时器预分频器
  htimx_BDCMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;                  // 计数方向：向上计数
  htimx_BDCMOTOR.Init.Period = BDCMOTOR_TIM_PERIOD;                        // 定时器周期
  htimx_BDCMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // 时钟分频
  htimx_BDCMOTOR.Init.RepetitionCounter = BDCMOTOR_TIM_REPETITIONCOUNTER;  // 重复计数器
  /* 初始化定时器比较输出环境 */
  HAL_TIM_PWM_Init(&htimx_BDCMOTOR);

  /* 定时器时钟源配置 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // 使用内部时钟源
  HAL_TIM_ConfigClockSource(&htimx_BDCMOTOR, &sClockSourceConfig);

  /* 定时器比较输出配置 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  // 比较输出模式：PWM1模式
  sConfigOC.Pulse =  PWM_Duty;                         // 占空比
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;        // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // 互补通道空闲电平
  HAL_TIM_PWM_ConfigChannel(&htimx_BDCMOTOR, &sConfigOC, TIM_CHANNEL_1);
}


/**
  * 函数功能: 基本定时器硬件反初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BDCMOTOR_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    BDCMOTOR_TIM_RCC_CLK_DISABLE();
    
    HAL_GPIO_DeInit(BDCMOTOR_TIM_CH1_PORT,BDCMOTOR_TIM_CH1_PIN);
    HAL_GPIO_DeInit(BDCMOTOR_TIM_CH1N_PORT,BDCMOTOR_TIM_CH1N_PIN);
  }
} 

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
