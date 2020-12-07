/**
  ******************************************************************************
  * �ļ�����: bsp_usartx_HMI.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���ش��ڵײ���������
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
#include "HMI/bsp_usartx_HMI.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
UART_HandleTypeDef husartx_HMI;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: HMIͨ�Ź�������GPIO��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void HMI_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��������ʱ��ʹ�� */
  HMI_USART_RCC_CLK_ENABLE();
  HMI_USARTx_GPIO_ClK_ENABLE();

  /* �������蹦��GPIO���� */
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
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void HMI_USARTx_Init(void)
{ 
  /* HMIͨ�Ź�������GPIO��ʼ�� */
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

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
