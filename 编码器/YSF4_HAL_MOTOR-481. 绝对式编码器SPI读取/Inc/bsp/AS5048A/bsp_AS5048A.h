#ifndef __BSP_AS5048A_H__
#define __BSP_AS5048A_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct 
{
  uint32_t Uint32;
  float Float;
}Float_ByteTypeDef;

typedef struct 
{
  uint16_t RegAutoGainCtrl;// 输出寄存器->Automatic Gain Control 
  uint16_t RegMagnitude;   // 输出寄存器->Magnirude 
  uint16_t RegAngle;       // 输出寄存器->Angle (Angle value in Register (0x0...0x3FFF))
  Float_ByteTypeDef AngleValue;// 角度值(degree) (0..359°)/(0..359.9°)
}AS5048_ReadDataTypeDef ;

/* 宏定义 --------------------------------------------------------------------*/
#define SPI_CMD_READ          0x4000  // 读标志位bit 14
#define SPI_CMD_WRITE         0x0000  // 写标志位bit 14
#define SPI_REG_AGC           0X3FFD  // ARG 寄存器地址
#define SPI_REG_MAG           0X3FFE  // MAGNITUDE寄存器地址
#define SPI_REG_DATA          0X3FFF  // 数据寄存器地址

#define SPI_REG_CLRERR        0x0001  // 错误标志寄存器地址
#define SPI_REG_ZEROPOS_HI    0x0016  // 零点设定高寄存器地址
#define SPI_REG_ZEROPOS_LO    0x0017  // 零点设定低寄存器地址

/* 扩展变量 ------------------------------------------------------------------*/
extern AS5048_ReadDataTypeDef ReadoutREG; // 输出寄存器值
/* 函数声明 ------------------------------------------------------------------*/
void AS5048AReadData(void);
uint16_t AS5048A_GetAverageAngle(void);
#endif

/* __BSP_NRF24L01_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
