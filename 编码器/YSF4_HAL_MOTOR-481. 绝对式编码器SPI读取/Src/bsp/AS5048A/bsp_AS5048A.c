/**
  ******************************************************************************
  * 文件名程: bsp_AS5048A.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2018-04-18
  * 功    能: AS5048A_SPI通信基础
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
#include "AS5048A/bsp_AS5048A.h"
#include "SPI/bsp_SPI.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define AVERAGE       4 // 取4次数据做平均值,Sensor output noise 2.73LSB@14bit
#define AVERAGE_POWER 2 // 2^2 = AVERAGE
/* 私有变量 ------------------------------------------------------------------*/
AS5048_ReadDataTypeDef ReadoutREG; // 输出寄存器
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
static uint8_t CalcEvenParity(uint16_t value);
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 计算偶校验位函数
  * 输入参数: value | uint6_t 待校验的数据
  * 返 回 值: 校验位结果
  * 说    明: 16位无符号整数的奇偶校验,如果是偶数个1,则返回0. 
  */
static uint8_t CalcEvenParity(uint16_t value)
{
  uint16_t cnt = 0;
  while(value)
  {
    cnt += value & 0x1;
    value >>= 1;
  }
  return cnt & 0x1;
}

/**
  * 函数功能: SPI读取数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：读取输出寄存器的数据,Readout Registers
  *           
  */ 
void AS5048AReadData()
{
  uint16_t dat; // 16-bit 数据,用于SPI通信
  uint16_t magreg,agcreg;
  uint16_t value;
  /* 发送读AGC指令,丢弃接收到的数据:数据来自于上一个指令(未知)*/
  dat = SPI_CMD_READ | SPI_REG_AGC;
  dat |= CalcEvenParity(dat) << 15;
  dat = SPIx_ReadWriteByte(&hspix,dat);

  /* 发送读MAG指令. 接收到的数据是AGC值:数据来自于上一个指令 (SPI_REG_AGC)*/
  dat = SPI_CMD_READ | SPI_REG_MAG;
  dat |= CalcEvenParity(dat) << 15;
  dat = SPIx_ReadWriteByte(&hspix,dat);
  agcreg = dat;
  
  /* 发送读Angle指令. 接收到的数据是MAG值:数据来自于上一个指令 (SPI_REG_MAG)*/
  dat = SPI_CMD_READ | SPI_REG_DATA;
  dat |= CalcEvenParity(dat) << 15;
  dat = SPIx_ReadWriteByte(&hspix,dat);
  magreg = dat;
  
  /* 发送读NOP指令. 接收到的数据是Angle值:数据来自于上一个指令 (SPI_REG_DATA)*/
  dat = 0x0000; // NOP command.芯片响应命令是0x0000
  dat = SPIx_ReadWriteByte(&hspix,dat);
  
  /* error flag set - need to reset it */
  if (dat & 0x4000 )
  {
    dat = SPI_CMD_READ | SPI_REG_CLRERR;
    dat |= CalcEvenParity(dat)<<15;
    SPIx_ReadWriteByte(&hspix,dat);
  }
  else
  {
    value = dat & (16384 - 31 - 1); // Angle value (0.. 16384 steps)
    ReadoutREG.RegAngle = value;
    ReadoutREG.RegMagnitude = magreg & (16384 - 31 - 1);
    ReadoutREG.RegAutoGainCtrl = agcreg & 0XFF;// AGC value (0..255)
    
    ReadoutREG.AngleValue.Float = (float)(value * 360) / 16384.0f;// Angle value in degree (0..359.9°)
    ReadoutREG.AngleValue.Uint32 = (uint32_t)ReadoutREG.AngleValue.Float;
//    HAL_Delay(5);
  }
}

/**
  * 函数功能: 读取传感器的角度值
  * 输入参数: 无
  * 返 回 值: Angle | 角度值
  * 说    明: 读取编码器的角度值
  */
uint16_t Sensor_Read()
{
  uint16_t dat = 0;
  uint32_t Angle = 0 ;
  uint32_t value = 0;
  /* 发送读Angle指令. 接收到的数据是MAG值:数据来自于上一个指令 (SPI_REG_MAG)*/
  dat = SPI_CMD_READ | SPI_REG_DATA;
  dat |= CalcEvenParity(dat) << 15;
  dat = SPIx_ReadWriteByte(&hspix,dat);
  
  if (dat & 0x4000 )
  {
    dat = SPI_CMD_READ | SPI_REG_CLRERR;
    dat |= CalcEvenParity(dat)<<15;
    SPIx_ReadWriteByte(&hspix,dat);
  }
  else
  {
    value = dat & (16384 - 31 - 1); // Angle value (0.. 16384 steps)
    Angle = (value * 360) / 16384;// Angle value in degree (0..359.9°)
  }
  return Angle;
}

/**
  * 函数功能: 简易的求平均算法
  * 输入参数: 无
  * 返 回 值: Position | 采样n次之后的均值
  * 说    明: 简单的平均算法来减少噪音的输出.
  */
uint16_t AS5048A_GetAverageAngle()
{
  uint16_t count = AVERAGE;
  uint16_t Position = 0;
  uint16_t data ;
  while(count--)
  {
    data = Sensor_Read(); //reading,computing sensor output
    Position += data;
  }
  Position = Position >> AVERAGE_POWER; // 2^2 = AVERAGE;
  return Position;
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/


