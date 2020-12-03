/**
  ******************************************************************************
  * 文件名程: bsp_usartx_HMI.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 板载串口底层驱动程序
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
#include "HMI/bsp_usartx_HMI.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
UART_HandleTypeDef husartx_HMI;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: HMI通信功能引脚GPIO初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HMI_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* 串口外设时钟使能 */
  HMI_USART_RCC_CLK_ENABLE();
  HMI_USARTx_GPIO_ClK_ENABLE();

  /* 串口外设功能GPIO配置 */
  GPIO_InitStruct.Pin = HMI_USARTx_Tx_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(HMI_USARTx_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HMI_USARTx_Rx_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(HMI_USARTx_PORT, &GPIO_InitStruct);
  
}

/**
  * 函数功能: 串口参数配置.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HMI_USARTx_Init(void)
{ 
  /* HMI通信功能引脚GPIO初始化 */
  HMI_GPIO_Init();
  
  husartx_HMI.Instance = HMI_USARTx;
  husartx_HMI.Init.BaudRate = HMI_USARTx_BAUDRATE;
  husartx_HMI.Init.WordLength = UART_WORDLENGTH_8B;
  husartx_HMI.Init.StopBits = UART_STOPBITS_1;
  husartx_HMI.Init.Parity = UART_PARITY_NONE;
  husartx_HMI.Init.Mode = UART_MODE_TX_RX;
  husartx_HMI.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx_HMI.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx_HMI);
  
    /* HMI_USARTx interrupt configuration */
  HAL_NVIC_SetPriority(HMI_USARTx_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(HMI_USARTx_IRQn);
  
  __HAL_UART_CLEAR_FLAG(&husartx_HMI, UART_FLAG_RXNE);
  __HAL_UART_DISABLE_IT(&husartx_HMI, UART_IT_RXNE);
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
