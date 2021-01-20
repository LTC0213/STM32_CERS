#ifndef __BSP_USARTX_H__
#define __BSP_USARTX_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
//#define _RS485_
//#define _RS232_
#define _USART_

#ifdef _USART_

  #define USARTx                                 USART1
  #define USARTx_BAUDRATE                        115200
  #define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART1_CLK_ENABLE()
  #define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART1_CLK_DISABLE()

  #define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
  #define USARTx_Tx_GPIO_PIN                     GPIO_PIN_6
  #define USARTx_Tx_GPIO                         GPIOB
  #define USARTx_Rx_GPIO_PIN                     GPIO_PIN_7   
  #define USARTx_Rx_GPIO                         GPIOB

  #define USARTx_AFx                             GPIO_AF7_USART1

  #define USARTx_IRQHANDLER                      USART1_IRQHandler
  #define USARTx_IRQn                            USART1_IRQn
#else 
  #define USARTx                                 USART3
  #define USARTx_BAUDRATE                        115200
  #define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART3_CLK_ENABLE()
  #define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART3_CLK_DISABLE()

  #define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
  #define USARTx_Tx_GPIO_PIN                     GPIO_PIN_10
  #define USARTx_Tx_GPIO                         GPIOB
  #define USARTx_Rx_GPIO_PIN                     GPIO_PIN_11
  #define USARTx_Rx_GPIO                         GPIOB

  #define USARTx_AFx                             GPIO_AF7_USART3
  
  #define USARTx_IRQHANDLER                      USART3_IRQHandler
  #define USARTx_IRQn                            USART3_IRQn
  
  #ifdef _RS485_  
    #define RS485_REDE_GPIO_ClK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
    #define RS485_REDE_PORT                              GPIOC
    #define RS485_REDE_PIN                               GPIO_PIN_12
    #define RS485_RX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
    #define RS485_TX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)
  #endif
  
#endif

/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx;

/* 函数声明 ------------------------------------------------------------------*/
void MX_USARTx_Init(void);


#endif  /* __BSP_USARTX_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
