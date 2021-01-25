#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define SPIx                            SPI1
#define SPIx_RCC_CLK_ENABLE()           __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_RCC_CLK_DISABLE()          __HAL_RCC_SPI1_CLK_DISABLE()

#define SPIx_SCK_ClK_ENABLE()            __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_SCK_PORT                    GPIOA
#define SPIx_SCK_PIN                     GPIO_PIN_5

#define SPIx_MOSI_ClK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_PORT                   GPIOB
#define SPIx_MOSI_PIN                    GPIO_PIN_5

#define SPIx_MISO_ClK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_PORT                   GPIOB
#define SPIx_MISO_PIN                    GPIO_PIN_4

#define SPIx_CS_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()    
#define SPIx_CS_PORT                     GPIOC
#define SPIx_CS_PIN                      GPIO_PIN_13
#define SPIx_CS_ENABLE()                 HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_RESET)
#define SPIx_CS_DISABLE()                HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_SET)
               
#define SPI_DATA_SIZE                    SPI_DATASIZE_16BIT
/* 扩展变量 ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspix;

/* 函数声明 ------------------------------------------------------------------*/

void SPIx_Init(void);

#if (SPI_DATA_SIZE)==(SPI_DATASIZE_16BIT)                    
  uint16_t SPIx_Write_Buf(uint16_t reg, uint16_t *pBuf, uint16_t uint8_ts);//写数据区
  uint16_t SPIx_Read_Buf(uint16_t reg, uint16_t *pBuf, uint16_t uint8_ts);	//读数据区		  
  uint16_t SPIx_Read_Reg(uint16_t reg);					//读寄存器
  uint16_t SPIx_Write_Reg(uint16_t reg, uint16_t value);		//写寄存器
  uint16_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint16_t Byte);
#else
  uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t Byte);
  uint8_t SPIx_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);//写数据区
  uint8_t SPIx_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);	//读数据区		  
  uint8_t SPIx_Read_Reg(uint8_t reg);					//读寄存器
  uint8_t SPIx_Write_Reg(uint8_t reg, uint8_t value);		//写寄存器
#endif


#endif  /* __BSP_SPI_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
