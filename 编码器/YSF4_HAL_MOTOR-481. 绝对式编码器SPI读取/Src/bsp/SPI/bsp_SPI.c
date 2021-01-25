/**
  ******************************************************************************
  * 文件名程: bsp_spiflash.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 板载串行Flash底层驱动实现
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
#include "SPI/bsp_SPI.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
SPI_HandleTypeDef hspix;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 串行FLASH初始化
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
void SPIx_Init(void)
{
  SPIx_RCC_CLK_ENABLE();

  hspix.Instance = SPIx;
  hspix.Init.Mode = SPI_MODE_MASTER;
  hspix.Init.Direction = SPI_DIRECTION_2LINES;
  hspix.Init.DataSize = SPI_DATA_SIZE;
  hspix.Init.CLKPolarity = SPI_POLARITY_LOW; // 时钟空闲电平极性
  hspix.Init.CLKPhase = SPI_PHASE_2EDGE;     // 时钟相位, 时钟第一个边沿采样
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
  * 函数功能: SPI外设系统级初始化
  * 输入参数: hspi：SPI句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
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
  * 函数功能: 往SPI设备读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
  */
uint16_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint16_t Byte)
{
  uint16_t d_read = 0xFFFF;
  uint16_t d_send = Byte;
  SPIx_CS_ENABLE();                 //使能SPIx传输
  if(HAL_SPI_TransmitReceive(hspi,(uint8_t*)&d_send,(uint8_t*)&d_read,1,0x0F)!=HAL_OK)
  {
    d_read = 0xFFFF;
  }
  SPIx_CS_DISABLE();                 //禁止SPIx传输	   
  return d_read; 
} 
/**
  * 函数功能: SPI写寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:指定寄存器地址,value:写入寄存器的数值
  *           
  */ 
uint16_t SPIx_Write_Reg(uint16_t reg,uint16_t value)
{
	uint16_t status;	
  status = SPIx_ReadWriteByte(&hspix,reg);//发送寄存器号 
  SPIx_ReadWriteByte(&hspix,value);      //写入寄存器的值
  return(status);       			//返回状态值
}


/**
  * 函数功能: 读取SPI寄存器值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:要读的寄存器
  *           
  */ 
uint16_t  SPIx_Read_Reg(uint16_t reg)
{
	uint16_t reg_val;	    
  SPIx_ReadWriteByte(&hspix,reg);   //发送寄存器号
  reg_val=SPIx_ReadWriteByte(&hspix,0XFFFF);//读取寄存器内容
  return(reg_val);           //返回状态值
}	

/**
  * 函数功能: 在指定位置读出指定长度的数据
  * 输入参数: 无
  * 返 回 值: 此次读到的状态寄存器值 
  * 说    明：无
  *           
  */ 
uint16_t  SPIx_Read_Buf(uint16_t reg,uint16_t *pBuf,uint16_t len)
{
	uint16_t status,uint8_t_ctr;	   
  
  status=SPIx_ReadWriteByte(&hspix,reg);//发送寄存器值(位置),并读取状态值   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspix,0XFFFF);//读出数据
  }
  return status;        //返回读到的状态值
}

/**
  * 函数功能: 在指定位置写指定长度的数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
  *           
  */ 
uint16_t SPIx_Write_Buf(uint16_t reg, uint16_t *pBuf, uint16_t len)
{
	uint16_t status,uint8_t_ctr;	    
  status = SPIx_ReadWriteByte(&hspix,reg);//发送寄存器值(位置),并读取状态值
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hspix,*pBuf++); //写入数据	 
  }
  return status;          //返回读到的状态值
}				 
#else
/**
  * 函数功能: 往SPI设备读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
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
  * 函数功能: SPI写寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:指定寄存器地址,value:写入寄存器的数值
  *           
  */ 
uint8_t SPIx_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
  SPIx_CS_ENABLE();                 //使能SPIx传输
  status = SPIx_ReadWriteByte(&hspix,reg);//发送寄存器号 
  SPIx_ReadWriteByte(&hspix,value);      //写入寄存器的值
  SPIx_CS_DISABLE();                 //禁止SPIx传输	   
  return(status);       			//返回状态值
}

/**
  * 函数功能: 读取SPI寄存器值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:要读的寄存器
  *           
  */ 
uint8_t  SPIx_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	SPIx_CS_ENABLE();          //使能SPI传输		
  SPIx_ReadWriteByte(&hspix,reg);   //发送寄存器号
  reg_val=SPIx_ReadWriteByte(&hspix,0XFF);//读取寄存器内容
  SPIx_CS_DISABLE();          //禁止SPIx传输		    
  return(reg_val);           //返回状态值
}	

/**
  * 函数功能: 在指定位置读出指定长度的数据
  * 输入参数: 无
  * 返 回 值: 此次读到的状态寄存器值 
  * 说    明：无
  *           
  */ 
uint8_t  SPIx_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	   
  
  SPIx_CS_ENABLE();           //使能SPIx传输
  status=SPIx_ReadWriteByte(&hspix,reg);//发送寄存器值(位置),并读取状态值   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspix,0XFF);//读出数据
  }
  SPIx_CS_DISABLE();       //关闭SPIx传输
  return status;        //返回读到的状态值
}

/**
  * 函数功能: 在指定位置写指定长度的数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
  *           
  */ 
uint8_t SPIx_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	SPIx_CS_ENABLE();          //使能SPIx传输
  status = SPIx_ReadWriteByte(&hspix,reg);//发送寄存器值(位置),并读取状态值
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hspix,*pBuf++); //写入数据	 
  }
  SPIx_CS_DISABLE();       //关闭SPIx传输
  return status;          //返回读到的状态值
}				   
#endif



/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

