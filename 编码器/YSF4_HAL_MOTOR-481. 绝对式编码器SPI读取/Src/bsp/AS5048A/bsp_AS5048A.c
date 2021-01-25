/**
  ******************************************************************************
  * �ļ�����: bsp_AS5048A.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2018-04-18
  * ��    ��: AS5048A_SPIͨ�Ż���
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
#include "AS5048A/bsp_AS5048A.h"
#include "SPI/bsp_SPI.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define AVERAGE       4 // ȡ4��������ƽ��ֵ,Sensor output noise 2.73LSB@14bit
#define AVERAGE_POWER 2 // 2^2 = AVERAGE
/* ˽�б��� ------------------------------------------------------------------*/
AS5048_ReadDataTypeDef ReadoutREG; // ����Ĵ���
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
static uint8_t CalcEvenParity(uint16_t value);
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ����żУ��λ����
  * �������: value | uint6_t ��У�������
  * �� �� ֵ: У��λ���
  * ˵    ��: 16λ�޷�����������żУ��,�����ż����1,�򷵻�0. 
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
  * ��������: SPI��ȡ����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������ȡ����Ĵ���������,Readout Registers
  *           
  */ 
void AS5048AReadData()
{
  uint16_t dat; // 16-bit ����,����SPIͨ��
  uint16_t magreg,agcreg;
  uint16_t value;
  /* ���Ͷ�AGCָ��,�������յ�������:������������һ��ָ��(δ֪)*/
  dat = SPI_CMD_READ | SPI_REG_AGC;
  dat |= CalcEvenParity(dat) << 15;
  dat = SPIx_ReadWriteByte(&hspix,dat);

  /* ���Ͷ�MAGָ��. ���յ���������AGCֵ:������������һ��ָ�� (SPI_REG_AGC)*/
  dat = SPI_CMD_READ | SPI_REG_MAG;
  dat |= CalcEvenParity(dat) << 15;
  dat = SPIx_ReadWriteByte(&hspix,dat);
  agcreg = dat;
  
  /* ���Ͷ�Angleָ��. ���յ���������MAGֵ:������������һ��ָ�� (SPI_REG_MAG)*/
  dat = SPI_CMD_READ | SPI_REG_DATA;
  dat |= CalcEvenParity(dat) << 15;
  dat = SPIx_ReadWriteByte(&hspix,dat);
  magreg = dat;
  
  /* ���Ͷ�NOPָ��. ���յ���������Angleֵ:������������һ��ָ�� (SPI_REG_DATA)*/
  dat = 0x0000; // NOP command.оƬ��Ӧ������0x0000
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
    
    ReadoutREG.AngleValue.Float = (float)(value * 360) / 16384.0f;// Angle value in degree (0..359.9��)
    ReadoutREG.AngleValue.Uint32 = (uint32_t)ReadoutREG.AngleValue.Float;
//    HAL_Delay(5);
  }
}

/**
  * ��������: ��ȡ�������ĽǶ�ֵ
  * �������: ��
  * �� �� ֵ: Angle | �Ƕ�ֵ
  * ˵    ��: ��ȡ�������ĽǶ�ֵ
  */
uint16_t Sensor_Read()
{
  uint16_t dat = 0;
  uint32_t Angle = 0 ;
  uint32_t value = 0;
  /* ���Ͷ�Angleָ��. ���յ���������MAGֵ:������������һ��ָ�� (SPI_REG_MAG)*/
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
    Angle = (value * 360) / 16384;// Angle value in degree (0..359.9��)
  }
  return Angle;
}

/**
  * ��������: ���׵���ƽ���㷨
  * �������: ��
  * �� �� ֵ: Position | ����n��֮��ľ�ֵ
  * ˵    ��: �򵥵�ƽ���㷨���������������.
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

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/


