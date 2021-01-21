#ifndef __BSP_USARTX_HMI_H__
#define __BSP_USARTX_HMI_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define HMI_USARTx                                 USART2
#define HMI_USARTx_BAUDRATE                        115200
#define HMI_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define HMI_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()

#define HMI_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define HMI_USARTx_PORT                            GPIOD
#define HMI_USARTx_Tx_PIN                          GPIO_PIN_5
#define HMI_USARTx_Rx_PIN                          GPIO_PIN_6

#define HMI_USARTx_IRQHANDLER                      USART2_IRQHandler
#define HMI_USARTx_IRQn                            USART2_IRQn
 
/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx_HMI;

/* 函数声明 ------------------------------------------------------------------*/
void HMI_USARTx_Init(void);


#endif  /* __BSP_USARTX_HMI_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
