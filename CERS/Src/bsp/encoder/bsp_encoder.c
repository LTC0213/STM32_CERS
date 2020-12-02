/**
  ******************************************************************************
  * 文件名程: bsp_EncoderTIM.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-6-06
  * 功    能: 编码器
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "encoder/bsp_encoder.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
int32_t XOverflowCount = 0;//定时器溢出次数

int32_t YOverflowCount = 0;//定时器溢出次数

/* Timer handler declaration */
TIM_HandleTypeDef    xhtimx_Encoder;

TIM_HandleTypeDef    yhtimx_Encoder;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 通用定时器初始化并配置通道PWM输出
  * 输入参数: 无
  * 返 回 值: 无 
  * 说    明: 无
  */
void XENCODER_TIMx_Init(void)
{
  /* Timer Encoder Configuration Structure declaration */
  TIM_Encoder_InitTypeDef xsEncoderConfig;

  XENCODER_TIM_RCC_CLK_ENABLE();
  xhtimx_Encoder.Instance = XENCODER_TIMx;
  xhtimx_Encoder.Init.Prescaler = XENCODER_TIM_PRESCALER;
  xhtimx_Encoder.Init.CounterMode = TIM_COUNTERMODE_UP;
  xhtimx_Encoder.Init.Period = XENCODER_TIM_PERIOD;
  xhtimx_Encoder.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;

  xsEncoderConfig.EncoderMode        = XTIM_ENCODERMODE_TIx;    
  xsEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;   
  xsEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;  
  xsEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1; 
  xsEncoderConfig.IC1Filter          = 0;
  
  xsEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;   
  xsEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;  
  xsEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1; 
  xsEncoderConfig.IC2Filter          = 0;
  __HAL_TIM_SET_COUNTER(&xhtimx_Encoder,0);
  
  HAL_TIM_Encoder_Init(&xhtimx_Encoder, &xsEncoderConfig);

  __HAL_TIM_CLEAR_IT(&xhtimx_Encoder, TIM_IT_UPDATE);  // 清除更新中断标志位
  __HAL_TIM_URS_ENABLE(&xhtimx_Encoder);               // 仅允许计数器溢出才产生更新中断
  __HAL_TIM_ENABLE_IT(&xhtimx_Encoder,TIM_IT_UPDATE);  // 使能更新中断
  
  HAL_NVIC_SetPriority(XENCODER_TIM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(XENCODER_TIM_IRQn);
}

void YENCODER_TIMx_Init(void)
{
  /* Timer Encoder Configuration Structure declaration */
  TIM_Encoder_InitTypeDef ysEncoderConfig;

  YENCODER_TIM_RCC_CLK_ENABLE();
  yhtimx_Encoder.Instance = YENCODER_TIMx;
  yhtimx_Encoder.Init.Prescaler = YENCODER_TIM_PRESCALER;
  yhtimx_Encoder.Init.CounterMode = TIM_COUNTERMODE_UP;
  yhtimx_Encoder.Init.Period = YENCODER_TIM_PERIOD;
  yhtimx_Encoder.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;

  ysEncoderConfig.EncoderMode        = YTIM_ENCODERMODE_TIx;    
  ysEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;   
  ysEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;  
  ysEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1; 
  ysEncoderConfig.IC1Filter          = 0;
  
  ysEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;   
  ysEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;  
  ysEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1; 
  ysEncoderConfig.IC2Filter          = 0;
  __HAL_TIM_SET_COUNTER(&yhtimx_Encoder,0);
  
  HAL_TIM_Encoder_Init(&yhtimx_Encoder, &ysEncoderConfig);

  __HAL_TIM_CLEAR_IT(&yhtimx_Encoder, TIM_IT_UPDATE);  // 清除更新中断标志位
  __HAL_TIM_URS_ENABLE(&yhtimx_Encoder);               // 仅允许计数器溢出才产生更新中断
  __HAL_TIM_ENABLE_IT(&yhtimx_Encoder,TIM_IT_UPDATE);  // 使能更新中断
  
  HAL_NVIC_SetPriority(YENCODER_TIM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(YENCODER_TIM_IRQn);
}
/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef XGPIO_InitStruct;

  GPIO_InitTypeDef YGPIO_InitStruct;

  if(htim_base->Instance==XENCODER_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    XENCODER_TIM_GPIO_CLK_ENABLE();

    /* 定时器通道1功能引脚IO初始化 */
    XGPIO_InitStruct.Pin = XENCODER_TIM_CH1_PIN;
    XGPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    XGPIO_InitStruct.Pull=GPIO_PULLUP;
    XGPIO_InitStruct.Alternate = XENCODER_TIM_GPIO_AF;
    HAL_GPIO_Init(XENCODER_TIM_CH1_GPIO, &XGPIO_InitStruct);
    
    /* 定时器通道2功能引脚IO初始化 */
    XGPIO_InitStruct.Pin = XENCODER_TIM_CH2_PIN;
    XGPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    XGPIO_InitStruct.Pull = GPIO_PULLUP;
    XGPIO_InitStruct.Alternate = XENCODER_TIM_GPIO_AF;
    HAL_GPIO_Init(XENCODER_TIM_CH2_GPIO, &XGPIO_InitStruct);
  }
	if(htim_base->Instance==YENCODER_TIMx)
  {
    /* 基本定时器外设时钟使能 */
		YENCODER_TIM_GPIO_CLK_ENABLE();

    /* 定时器通道1功能引脚IO初始化 */
    YGPIO_InitStruct.Pin = YENCODER_TIM_CH1_PIN;
    YGPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    YGPIO_InitStruct.Pull=GPIO_PULLUP;
    YGPIO_InitStruct.Alternate = YENCODER_TIM_GPIO_AF;
    HAL_GPIO_Init(YENCODER_TIM_CH1_GPIO, &YGPIO_InitStruct);
    
    /* 定时器通道2功能引脚IO初始化 */
    YGPIO_InitStruct.Pin = YENCODER_TIM_CH2_PIN;
    YGPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    YGPIO_InitStruct.Pull = GPIO_PULLUP;
    YGPIO_InitStruct.Alternate = YENCODER_TIM_GPIO_AF;
    HAL_GPIO_Init(YENCODER_TIM_CH2_GPIO, &YGPIO_InitStruct);
  }

}

/**
  * 函数功能: 基本定时器硬件反初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==XENCODER_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    XENCODER_TIM_RCC_CLK_DISABLE();
    
    HAL_GPIO_DeInit(XENCODER_TIM_CH1_GPIO, XENCODER_TIM_CH1_PIN);
    HAL_GPIO_DeInit(XENCODER_TIM_CH2_GPIO, XENCODER_TIM_CH2_PIN);
  }
  if(htim_base->Instance==YENCODER_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    YENCODER_TIM_RCC_CLK_DISABLE();
    
    HAL_GPIO_DeInit(YENCODER_TIM_CH1_GPIO, YENCODER_TIM_CH1_PIN);
    HAL_GPIO_DeInit(YENCODER_TIM_CH2_GPIO, YENCODER_TIM_CH2_PIN);
  }
} 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&xhtimx_Encoder))
    XOverflowCount--;       //向下计数溢出
  else
    XOverflowCount++;       //向上计数溢出
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&yhtimx_Encoder))
    YOverflowCount--;       //向下计数溢出
  else
    YOverflowCount++;       //向上计数溢出
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
