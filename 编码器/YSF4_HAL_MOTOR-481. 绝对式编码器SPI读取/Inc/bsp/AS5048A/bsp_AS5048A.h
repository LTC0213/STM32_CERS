#ifndef __BSP_AS5048A_H__
#define __BSP_AS5048A_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct 
{
  uint32_t Uint32;
  float Float;
}Float_ByteTypeDef;

typedef struct 
{
  uint16_t RegAutoGainCtrl;// ����Ĵ���->Automatic Gain Control 
  uint16_t RegMagnitude;   // ����Ĵ���->Magnirude 
  uint16_t RegAngle;       // ����Ĵ���->Angle (Angle value in Register (0x0...0x3FFF))
  Float_ByteTypeDef AngleValue;// �Ƕ�ֵ(degree) (0..359��)/(0..359.9��)
}AS5048_ReadDataTypeDef ;

/* �궨�� --------------------------------------------------------------------*/
#define SPI_CMD_READ          0x4000  // ����־λbit 14
#define SPI_CMD_WRITE         0x0000  // д��־λbit 14
#define SPI_REG_AGC           0X3FFD  // ARG �Ĵ�����ַ
#define SPI_REG_MAG           0X3FFE  // MAGNITUDE�Ĵ�����ַ
#define SPI_REG_DATA          0X3FFF  // ���ݼĴ�����ַ

#define SPI_REG_CLRERR        0x0001  // �����־�Ĵ�����ַ
#define SPI_REG_ZEROPOS_HI    0x0016  // ����趨�߼Ĵ�����ַ
#define SPI_REG_ZEROPOS_LO    0x0017  // ����趨�ͼĴ�����ַ

/* ��չ���� ------------------------------------------------------------------*/
extern AS5048_ReadDataTypeDef ReadoutREG; // ����Ĵ���ֵ
/* �������� ------------------------------------------------------------------*/
void AS5048AReadData(void);
uint16_t AS5048A_GetAverageAngle(void);
#endif

/* __BSP_NRF24L01_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
