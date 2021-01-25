/**
  ******************************************************************************
  * �ļ�����: bsp_spiflash.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���ش���Flash�ײ�����ʵ��
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
#include "SPI/bsp_SPI.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/
SPI_HandleTypeDef hspix;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����FLASH��ʼ��
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
*/
void SPIx_Init(void)
{
  SPIx_RCC_CLK_ENABLE();

  hspix.Instance = SPIx;
  hspix.Init.Mode = SPI_MODE_MASTER;
  hspix.Init.Direction = SPI_DIRECTION_2LINES;
  hspix.Init.DataSize = SPI_DATA_SIZE;
  hspix.Init.CLKPolarity = SPI_POLARITY_LOW; // ʱ�ӿ��е�ƽ����
  hspix.Init.CLKPhase = SPI_PHASE_2EDGE;     // ʱ����λ, ʱ�ӵ�һ�����ز���
  hspix.Init.NSS = SPI_NSS_SOFT;
  hspix.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspix.Init.FirstBit = SPI_FIRSTBIT_MSB;    // MSB 
  hspix.Init.TIMode = SPI_TIMODE_DISABLE;
  hspix.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspix.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspix);
  __HAL_SPI_ENABLE(&hspix);
}

/**
  * ��������: SPI����ϵͳ����ʼ��
  * �������: hspi��SPI�������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi == &hspix)
  {
    SPIx_MOSI_ClK_ENABLE();
    SPIx_MISO_ClK_ENABLE();
    SPIx_SCK_ClK_ENABLE();
    SPIx_CS_CLK_ENABLE();  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI 
    */
    HAL_GPIO_WritePin( SPIx_CS_PORT,  SPIx_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin =  SPIx_CS_PIN;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_RTC_50Hz;
    HAL_GPIO_Init( SPIx_CS_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( SPIx_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =  SPIx_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( SPIx_MISO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( SPIx_MOSI_PORT, &GPIO_InitStruct);
  }
}

#if (SPI_DATA_SIZE)==(SPI_DATASIZE_16BIT)  
/**
  * ��������: ��SPI�豸��ȡд��һ���ֽ����ݲ�����һ���ֽ�����
  * �������: byte������������
  * �� �� ֵ: uint8_t�����յ�������
  * ˵    ������
  */
uint16_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint16_t Byte)
{
  uint16_t d_read = 0xFFFF;
  uint16_t d_send = Byte;
  SPIx_CS_ENABLE();                 //ʹ��SPIx����
  if(HAL_SPI_TransmitReceive(hspi,(uint8_t*)&d_send,(uint8_t*)&d_read,1,0x0F)!=HAL_OK)
  {
    d_read = 0xFFFF;
  }
  SPIx_CS_DISABLE();                 //��ֹSPIx����	   
  return d_read; 
} 
/**
  * ��������: SPIд�Ĵ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:ָ���Ĵ�����ַ,value:д��Ĵ�������ֵ
  *           
  */ 
uint16_t SPIx_Write_Reg(uint16_t reg,uint16_t value)
{
	uint16_t status;	
  status = SPIx_ReadWriteByte(&hspix,reg);//���ͼĴ����� 
  SPIx_ReadWriteByte(&hspix,value);      //д��Ĵ�����ֵ
  return(status);       			//����״ֵ̬
}


/**
  * ��������: ��ȡSPI�Ĵ���ֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:Ҫ���ļĴ���
  *           
  */ 
uint16_t  SPIx_Read_Reg(uint16_t reg)
{
	uint16_t reg_val;	    
  SPIx_ReadWriteByte(&hspix,reg);   //���ͼĴ�����
  reg_val=SPIx_ReadWriteByte(&hspix,0XFFFF);//��ȡ�Ĵ�������
  return(reg_val);           //����״ֵ̬
}	

/**
  * ��������: ��ָ��λ�ö���ָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: �˴ζ�����״̬�Ĵ���ֵ 
  * ˵    ������
  *           
  */ 
uint16_t  SPIx_Read_Buf(uint16_t reg,uint16_t *pBuf,uint16_t len)
{
	uint16_t status,uint8_t_ctr;	   
  
  status=SPIx_ReadWriteByte(&hspix,reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspix,0XFFFF);//��������
  }
  return status;        //���ض�����״ֵ̬
}

/**
  * ��������: ��ָ��λ��дָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:�Ĵ���(λ��)  *pBuf:����ָ��  len:���ݳ���
  *           
  */ 
uint16_t SPIx_Write_Buf(uint16_t reg, uint16_t *pBuf, uint16_t len)
{
	uint16_t status,uint8_t_ctr;	    
  status = SPIx_ReadWriteByte(&hspix,reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hspix,*pBuf++); //д������	 
  }
  return status;          //���ض�����״ֵ̬
}				 
#else
/**
  * ��������: ��SPI�豸��ȡд��һ���ֽ����ݲ�����һ���ֽ�����
  * �������: byte������������
  * �� �� ֵ: uint8_t�����յ�������
  * ˵    ������
  */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t Byte)
{
  uint8_t d_read = 0xFF;
  uint8_t d_send = Byte;
  if(HAL_SPI_TransmitReceive(hspi,(uint8_t*)&d_send,(uint8_t*)&d_read,1,0xFF)!=HAL_OK)
  {
    d_read = 0xFF;
  }
  return d_read; 
} 

/**
  * ��������: SPIд�Ĵ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:ָ���Ĵ�����ַ,value:д��Ĵ�������ֵ
  *           
  */ 
uint8_t SPIx_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
  SPIx_CS_ENABLE();                 //ʹ��SPIx����
  status = SPIx_ReadWriteByte(&hspix,reg);//���ͼĴ����� 
  SPIx_ReadWriteByte(&hspix,value);      //д��Ĵ�����ֵ
  SPIx_CS_DISABLE();                 //��ֹSPIx����	   
  return(status);       			//����״ֵ̬
}

/**
  * ��������: ��ȡSPI�Ĵ���ֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:Ҫ���ļĴ���
  *           
  */ 
uint8_t  SPIx_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	SPIx_CS_ENABLE();          //ʹ��SPI����		
  SPIx_ReadWriteByte(&hspix,reg);   //���ͼĴ�����
  reg_val=SPIx_ReadWriteByte(&hspix,0XFF);//��ȡ�Ĵ�������
  SPIx_CS_DISABLE();          //��ֹSPIx����		    
  return(reg_val);           //����״ֵ̬
}	

/**
  * ��������: ��ָ��λ�ö���ָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: �˴ζ�����״̬�Ĵ���ֵ 
  * ˵    ������
  *           
  */ 
uint8_t  SPIx_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	   
  
  SPIx_CS_ENABLE();           //ʹ��SPIx����
  status=SPIx_ReadWriteByte(&hspix,reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspix,0XFF);//��������
  }
  SPIx_CS_DISABLE();       //�ر�SPIx����
  return status;        //���ض�����״ֵ̬
}

/**
  * ��������: ��ָ��λ��дָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:�Ĵ���(λ��)  *pBuf:����ָ��  len:���ݳ���
  *           
  */ 
uint8_t SPIx_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	SPIx_CS_ENABLE();          //ʹ��SPIx����
  status = SPIx_ReadWriteByte(&hspix,reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hspix,*pBuf++); //д������	 
  }
  SPIx_CS_DISABLE();       //�ر�SPIx����
  return status;          //���ض�����״ֵ̬
}				   
#endif



/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

