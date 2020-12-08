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
#include "spiflash/bsp_spiweight.h"
#include "usart/bsp_debug_usart.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_weight;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 串行FLASH初始化
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
void MX_WEIGHT_SPI_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  WEIGHT_SPIx_CLK_ENABLE();
  WEIGHT_GPIO_CLK_ENABLE();
  /**SPI1 GPIO Configuration    
  PA5     ------> SPI1_SCK
  PB4     ------> SPI1_MISO
  PB5     ------> SPI1_MOSI 
  */    
  GPIO_InitStruct.Pin = WEIGHT_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(WEIGHT_SCK_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = WEIGHT_MISO_Pin|WEIGHT_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(WEIGHT_MISO_GPIO_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = WEIGHT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WEIGHT_CS_GPIO_Port, &GPIO_InitStruct);  
  
  
  hspi_weight.Instance = WEIGHT_SPIx;
  hspi_weight.Init.Mode = SPI_MODE_MASTER;
  hspi_weight.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_weight.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_weight.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi_weight.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_weight.Init.NSS = SPI_NSS_SOFT;
  hspi_weight.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi_weight.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_weight.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_weight.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_weight.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi_weight);
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 * @param bytesNumber - Number of bytes to be written.
 *
 * @return none.
*******************************************************************************/
void AD7190_SetRegisterValue(unsigned char registerAddress,
                             unsigned int  registerValue,
                             unsigned char bytesNumber)
{
    unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;
    
    writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);
    while(bytesNr > 0)
    {
        writeCommand[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }
    HAL_SPI_Transmit(&hspi_weight,writeCommand, bytesNumber+1,0xFFFFFF);
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes that will be read.
 *
 * @return buffer - Value of the register.
*******************************************************************************/
unsigned int AD7190_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber)
{
    unsigned char registerWord[4] = {0, 0, 0, 0}; 
    unsigned char address         = 0;
    unsigned int  buffer          = 0x0;
    unsigned char i               = 0;
    
    address = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);
    
    HAL_SPI_Transmit(&hspi_weight,&address, 1,0xFFFFFF);    
    HAL_SPI_Receive(&hspi_weight,registerWord,bytesNumber,0xFFFFFF);
    for(i = 0; i < bytesNumber; i++) 
    {
      buffer = (buffer << 8) + registerWord[i];
    }
    return buffer;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return none.
*******************************************************************************/
void AD7190_Reset(void)
{
    unsigned char registerWord[7];
    
    registerWord[0] = 0x01;
    registerWord[1] = 0xFF;
    registerWord[2] = 0xFF;
    registerWord[3] = 0xFF;
    registerWord[4] = 0xFF;
    registerWord[5] = 0xFF;
    registerWord[6] = 0xFF;
    HAL_SPI_Transmit(&hspi_weight,registerWord, 7,0xFFFFFF);
}
/***************************************************************************//**
 * @brief Checks if the AD7190 part is present.
 *
 * @return status - Indicates if the part is present or not.
*******************************************************************************/
unsigned char AD7190_Init(void)
{
    unsigned char status = 1;
    uint32_t regVal = 0;
    
    MX_WEIGHT_SPI_Init();  
    AD7190_Reset();
  
    /* Allow at least 500 us before accessing any of the on-chip registers. */
    HAL_Delay(1);
    regVal = AD7190_GetRegisterValue(AD7190_REG_ID, 1);
//    printf("ad7190:0x%X\n",regVal);
    if( (regVal & AD7190_ID_MASK) != ID_AD7190)
    {
        status = 0;
    }
    return status ;
}


/***************************************************************************//**
 * @brief Set device to idle or power-down.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
 *
 * @return none.
*******************************************************************************/
void AD7190_SetPower(unsigned char pwrMode)
{
     unsigned int oldPwrMode = 0x0;
     unsigned int newPwrMode = 0x0; 
 
     oldPwrMode = AD7190_GetRegisterValue(AD7190_REG_MODE, 3);
     oldPwrMode &= ~(AD7190_MODE_SEL(0x7));
     newPwrMode = oldPwrMode | AD7190_MODE_SEL((pwrMode * (AD7190_MODE_IDLE)) | (!pwrMode * (AD7190_MODE_PWRDN)));
     AD7190_SetRegisterValue(AD7190_REG_MODE, newPwrMode, 3);
}

/***************************************************************************//**
 * @brief Waits for RDY pin to go low.
 *
 * @return none.
*******************************************************************************/
void AD7190_WaitRdyGoLow(void)
{
    unsigned int timeOutCnt = 0xFFFFF;
    
    while(AD7190_RDY_STATE && timeOutCnt--)
    {
        ;
    }
}

/***************************************************************************//**
 * @brief Selects the channel to be enabled.
 *
 * @param channel - Selects a channel.
 *  
 * @return none.
*******************************************************************************/
void AD7190_ChannelSelect(unsigned short channel)
{
    unsigned int oldRegValue = 0x0;
    unsigned int newRegValue = 0x0;   
     
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << channel);   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3);
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7190_Calibrate(unsigned char mode, unsigned char channel)
{
    unsigned int oldRegValue = 0x0;
    unsigned int newRegValue = 0x0;
    
    AD7190_ChannelSelect(channel);
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_MODE, 3);
    oldRegValue &= ~AD7190_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7190_MODE_SEL(mode);
//    WEIGHT_CS_ENABLE(); 
    AD7190_SetRegisterValue(AD7190_REG_MODE, newRegValue, 3); // CS is not modified.
    AD7190_WaitRdyGoLow();
//    WEIGHT_CS_DISABLE();
}

/***************************************************************************//**
 * @brief Selects the polarity of the conversion and the ADC input range.
 *
 * @param polarity - Polarity select bit. 
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
* @param range - Gain select bits. These bits are written by the user to select 
                 the ADC input range.     
 *
 * @return none.
*******************************************************************************/
void AD7190_RangeSetup(unsigned char polarity, unsigned char range)
{
    unsigned int oldRegValue = 0x0;
    unsigned int newRegValue = 0x0;
    
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF,3);
    oldRegValue &= ~(AD7190_CONF_UNIPOLAR | AD7190_CONF_GAIN(0x7));
    newRegValue = oldRegValue | (polarity * AD7190_CONF_UNIPOLAR) | AD7190_CONF_GAIN(range) | AD7190_CONF_BUF; 
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3);
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned int AD7190_SingleConversion(void)
{
    unsigned int command = 0x0;
    unsigned int regData = 0x0;
 
    command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);    
//    WEIGHT_CS_ENABLE(); 
    AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3); // CS is not modified.
    AD7190_WaitRdyGoLow();
    regData = AD7190_GetRegisterValue(AD7190_REG_DATA, 3);
//    WEIGHT_CS_DISABLE();
    
    return regData;
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned int AD7190_ContinuousReadAvg(unsigned char sampleNumber)
{
    unsigned int samplesAverage = 0x0;
    unsigned char count = 0x0;
    unsigned int command = 0x0;
    
    command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);
//    WEIGHT_CS_ENABLE(); 
    AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3);
    for(count = 0;count < sampleNumber;count ++)
    {
        AD7190_WaitRdyGoLow();
        samplesAverage += AD7190_GetRegisterValue(AD7190_REG_DATA, 3);
    }
//    WEIGHT_CS_DISABLE();
    samplesAverage = samplesAverage / sampleNumber;
    
    return samplesAverage ;
}

/***************************************************************************//**
 * @brief Read data from temperature sensor and converts it to Celsius degrees.
 *
 * @return temperature - Celsius degrees.
*******************************************************************************/
unsigned int AD7190_TemperatureRead(void)
{
    unsigned char temperature = 0x0;
    unsigned int dataReg = 0x0;
    
    AD7190_RangeSetup(0, AD7190_CONF_GAIN_1);
    AD7190_ChannelSelect(AD7190_CH_TEMP_SENSOR);
    dataReg = AD7190_SingleConversion();
    dataReg -= 0x800000;
    dataReg /= 2815;   // Kelvin Temperature
    dataReg -= 273;    //Celsius Temperature
    temperature = (unsigned int) dataReg;
    
    return temperature;
}

void weight_ad7190_conf(void)
{
  unsigned int command = 0x0;
  /* Calibrates channel AIN1(+) - AIN2(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M);
  /* Selects unipolar operation and ADC's input range to +-Vref/1. */
  AD7190_RangeSetup(0, AD7190_CONF_GAIN_128);
  AD7190_Calibrate(AD7190_MODE_CAL_INT_FULL,AD7190_CH_AIN1P_AIN2M);
  /* Performs a single conversion. */
  AD7190_ChannelSelect(AD7190_CH_AIN1P_AIN2M);    
  command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) |\
            AD7190_MODE_RATE(384)|AD7190_MODE_SINC3;
  AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3);
  AD7190_WaitRdyGoLow();
  AD7190_GetRegisterValue(AD7190_REG_DATA, 3);
  AD7190_WaitRdyGoLow();
  AD7190_GetRegisterValue(AD7190_REG_DATA, 3);
}

unsigned int weight_ad7190_ReadAvg(unsigned char sampleNumber)
{
#if 1
    unsigned int samplesAverage = 0x0;
    unsigned char count = 0x0;

    for(count = 0;count < sampleNumber;count ++)
    {
      AD7190_WaitRdyGoLow();
      samplesAverage += (AD7190_GetRegisterValue(AD7190_REG_DATA, 3)>>4);
    }
    samplesAverage = samplesAverage / sampleNumber;
    
    return samplesAverage ;
#else
    unsigned int samplesValue = 0x0;
    
    AD7190_WaitRdyGoLow();
    samplesValue = (AD7190_GetRegisterValue(AD7190_REG_DATA, 3)>>4);
    
    return samplesValue;
#endif
}

