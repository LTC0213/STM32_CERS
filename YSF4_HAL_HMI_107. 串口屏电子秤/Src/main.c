/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 串口屏电子秤
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
#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart/bsp_debug_usart.h"
#include "led/bsp_led.h"
#include "HMI/bsp_usartx_HMI.h"
#include "spiflash/bsp_spiweight.h"
#include "spiflash/bsp_spiflash.h"
#include "beep/bsp_beep.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define HMI_RX_BUFFER_SIZE       30

#define  FLASH_WriteAddress      0x0000
#define  FLASH_SectorToErase    FLASH_WriteAddress

#define HMI_SECTOR_ADDREE       4096*2

/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t  HMI_Rx_buf[HMI_RX_BUFFER_SIZE]={0};
__IO uint8_t  HMI_RX_flag=0;       //0:未接收到数据头  1：已经接收到数据头  2：一帧接收完毕
__IO uint8_t  usart_rx_flag;

__IO uint16_t timer_count=0;
__IO uint16_t pwm_data=0;

__IO uint8_t  weight_Zero_IsInit=0; //0: 零值未获取  1：已获取零值
__IO int32_t  weight_Zero_Data=0;   // [0]:无重物零值  [1]:有外皮重物零值

__IO uint8_t Is_thres_stamp=0;  //是否有阀值设定
__IO uint8_t Is_tare_stamp=0;   //是否有皮重标志
__IO int32_t Record_weight1;    //皮重记录值（用于每次称重前记录皮重） 
__IO int32_t Record_weight2;    //皮重记录值（用于每次称重前记录皮重） 
__IO int32_t cali_weight;       //校准使用
__IO int32_t weight_Zero_Data;  //皮重初始值
__IO int32_t weight_proportion; //换算值记录
__IO int32_t weight_current;    //皮重记录值
__IO int32_t second_count;      
__IO int32_t third_count;
__IO uint8_t Process_Step=0;

uint32_t DeviceID = 0;
uint32_t FlashID = 0;

uint8_t Tx_Buffer[3] = {0};
uint8_t Rx_Buffer[3] = {0};

int32_t Result_data=0;

/* 扩展变量 ------------------------------------------------------------------*/


/* 私有函数原形 --------------------------------------------------------------*/
void HMI_value_setting(const char *val_str,uint32_t value);
void HMI_string_setting(const char *val_str,int32_t value);

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

int32_t int_abs(int32_t value1,int32_t value2)
{
  if(value1>value2)
    return (value1-value2);
  else
    return (value2-value1);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  static int16_t timecount=0;  
  int32_t weight_first;
  uint32_t tmp[2];
  int32_t Compa_value;
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  HMI_USARTx_Init();
  /* 初始化LED */
  LED_GPIO_Init(); 
  
  __HAL_UART_ENABLE_IT(&husartx_HMI, UART_IT_RXNE);
  /* 初始化SPI */
  MX_SPIFlash_Init();
  /* 初始化BEEP */
  BEEP_GPIO_Init();
  /* Get SPI Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
  
  HAL_Delay(100);

	FlashID = SPI_FLASH_ReadID(); 
  printf("FlashID is 0x%X,  Manufacturer Device ID is 0x%X\n", FlashID, DeviceID);
	if (FlashID == SPI_FLASH_ID)  /* #define  sFLASH_ID  0XEF4018 */
	{	
		printf("检测到华邦串行flash W25Q128 !\n");
    SPI_FLASH_BufferRead(Rx_Buffer,HMI_SECTOR_ADDREE,sizeof(Rx_Buffer));
    Result_data=Rx_Buffer[0]*10000+Rx_Buffer[1]*100+Rx_Buffer[2];
    printf("Result_data=%d\n",Result_data);		
    
    if((Result_data>30550)&&(Result_data<30950)) 
    {
      weight_proportion=Result_data;
      weight_Zero_IsInit=1;
    }      
  }  	  
  if(AD7190_Init())
  {
    printf("检测到  AD7190 !\n");
    weight_ad7190_conf();
  }
  /* 无限循环 */
  while (1)
  {  
    timecount++;
    if(timecount>=10)
    {

      if(weight_Zero_IsInit==1)
      {
        weight_Zero_Data = weight_ad7190_ReadAvg(4);    
        weight_Zero_IsInit=2;              
      }
      else if(weight_Zero_IsInit==2)
      {
        
        int32_t data_temp,temp;
        int32_t weight_count;        
        
        weight_count=weight_ad7190_ReadAvg(1);
        data_temp=weight_count-weight_Zero_Data;
        temp=data_temp*100000/weight_proportion;
        weight_current=temp; 
//        printf("weight_current=%d\n",weight_current); 
        printf("Process_Step=%d\n",Process_Step); 
        if(Is_thres_stamp==1)  //如果有超出预设值，那么蜂鸣器响
        {
          if((weight_current-Record_weight1)>=Compa_value)
          {
            BEEP_ON;
            HAL_Delay(200);
            BEEP_OFF;
          }        
        }        
        if(Is_tare_stamp==1)   //如果有清零按钮按下
        {
          Record_weight2=Record_weight1-weight_current;
          if(Record_weight2>15)   
          {
            Record_weight1=0;
            Record_weight2=0;
            Is_tare_stamp=0;
            Process_Step=0;
            HMI_value_setting("page0.gross.val",0);
            HMI_value_setting("page0.net.val",0); 
          }           
        }
        //进行称重，过程是，第一件放入的物品都会被当成皮重，第二次放入的才是实际重量
        //本程序就是这样，如果需要其他功能，可以在此基础上进行修改
        switch(Process_Step)  
        {
          case 0:  //检测到有物品
            if(weight_count>weight_Zero_Data)
            {
              Process_Step=1;
              weight_count=0;
            }
            else 
            {
              HMI_value_setting("page0.gross.val",0);
              Process_Step=0;
              weight_count=0;
            }
          break;          
          case 1:  //检测到有物品
            if(int_abs(weight_count,weight_Zero_Data)>10)
            {
              Process_Step=2;
            }
            else 
            {
              HMI_value_setting("page0.gross.val",0);
              Process_Step=0;
            }
          break;
          case 2:  //测量皮重
            HAL_Delay(100);
            second_count=weight_ad7190_ReadAvg(3);
            if(int_abs(second_count,weight_count)<10)
            {
              HAL_Delay(10);
              weight_count=weight_ad7190_ReadAvg(3);
              third_count=weight_count;
              data_temp=weight_count-weight_Zero_Data;
              temp=data_temp*100000/weight_proportion;
              Record_weight1=temp; //每次换皮重时赋值
              printf("Record_weight1=%d\n",Record_weight1); 
              weight_current=temp;            
              Is_tare_stamp=1;
              HMI_value_setting("page0.gross.val",weight_current/10); 
              Process_Step=3;             
            }
            else 
            {
              Process_Step=0;
            } 
          break; 
          case 3:  //皮重的基础上加物品
            if(weight_count>third_count)
            {
             if(int_abs(weight_count,third_count)>10)
              {
                weight_count=0;
                Process_Step=4;
              }
              else 
              {       
                weight_count=0;                
                HMI_value_setting("page0.net.val",0);
                Process_Step=3;
              }              
            }
           
          break; 
          case 4:  //测量增加物品的重量
            HAL_Delay(100);  
            second_count=weight_ad7190_ReadAvg(3);
            if((int_abs(second_count,weight_count)<10))
            {
              Process_Step=4;
              weight_count=weight_ad7190_ReadAvg(3);
              weight_first=weight_count;
              data_temp=weight_count-third_count;              
              temp=data_temp*100000/weight_proportion;
              weight_current=temp; 
              HMI_value_setting("page0.net.val",weight_current/10);              
            }
            else
            {
              Process_Step=5;
            } 
          break;  
          case 5:
            if(weight_count>weight_first)
            {              
              Process_Step=4;
            }
            else 
            {
              Process_Step=3;
            }            
          break;         
             
        }
        timecount=0;         
      }         
    }
    HAL_Delay(10);
      
    if(HMI_RX_flag==2)
    {
      HMI_RX_flag=0;    
      switch(HMI_Rx_buf[1])
      {
        case 0x01:
          printf("电子秤校准\n");
        break;
        case 0x02:
          printf("第一步\n");         
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2];
          cali_weight= tmp[0]%10000;
          weight_Zero_Data = weight_ad7190_ReadAvg(4);       
          weight_Zero_IsInit=0;
        break;
        case 0x03:
        {
          int32_t temp;
          int32_t weight_count;
          weight_count=weight_ad7190_ReadAvg(3);   
          temp=(weight_count-weight_Zero_Data)*1000;
          if(temp>0)
          {
            int32_t data;              
            data=temp/cali_weight;  
            weight_proportion=data;
          }   
          Tx_Buffer[0]=weight_proportion/10000;
          Tx_Buffer[1]=weight_proportion/100%100;
          Tx_Buffer[2]=weight_proportion%100;          
          SPI_FLASH_SectorErase(HMI_SECTOR_ADDREE);        
          SPI_FLASH_BufferWrite(Tx_Buffer,HMI_SECTOR_ADDREE,sizeof(Tx_Buffer));       
          printf("第二步\n");         
        }
        break;
        case 0x04:
          printf("校准完成\n");
          Process_Step=0;
          weight_Zero_IsInit=1;   
          HMI_value_setting("page0.gross.val",0);
          HMI_value_setting("page0.net.val",0);           
        break;
        case 0x05:
          printf("清零\n");
          Process_Step=0;
          Is_tare_stamp=0;
          weight_Zero_IsInit=1;
          HMI_value_setting("page0.gross.val",0);
          HMI_value_setting("page0.net.val",0);        
        break;        
        case 0x10:         
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2]; 
          Compa_value=tmp[0]*10;    
          Is_thres_stamp=1; 
          if(Compa_value==0)
          {
            Is_thres_stamp=0;
          }            
          printf("Compa_value=%d\n",Compa_value);
        break;             
      }
    }
    
  }
}

/**
  * 函数功能: 串口屏中断处理函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void HMI_USARTx_IRQHANDLER(void)
{
  uint16_t tmp;
  static uint8_t HUM_Rx_count=0;  
  
  if(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_RXNE) != RESET)
  { 
    tmp=HMI_USARTx->DR;  
    if(tmp==0xF1)
    {
      HMI_RX_flag=1;
      HUM_Rx_count=1;
      HMI_Rx_buf[0]=0xF1;
      usart_rx_flag=200;
    }
    else if((HMI_RX_flag==1)&&(usart_rx_flag))
    {     
      HMI_Rx_buf[HUM_Rx_count]=tmp;
      HUM_Rx_count++;
      if(HUM_Rx_count>=HMI_RX_BUFFER_SIZE)
      {
        HUM_Rx_count=0;
        HMI_RX_flag=0;        
      }
      else if((tmp==0xFF)&&(HMI_Rx_buf[HUM_Rx_count-2]==0xFF)&&(HMI_Rx_buf[HUM_Rx_count-3]==0x00)&&(HMI_Rx_buf[HUM_Rx_count-4]==0xFF)&&(HMI_Rx_buf[HUM_Rx_count-5]==0xFF))
      {
        HMI_RX_flag=2;
        HUM_Rx_count=0;        
      }
    }
    else if(usart_rx_flag==0)
    {
      HMI_RX_flag=0;
      HUM_Rx_count=0;
    }      
  }
}

/**
  * 函数功能: 向串口屏发送数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void HMI_value_setting(const char *val_str,uint32_t value)
{
  uint8_t tmp_str[30]={0};
  uint8_t i;
  
  sprintf((char *)tmp_str,"%s=%d",val_str,value);
  for(i=0;i<strlen((char *)tmp_str);++i)
  {
    HMI_USARTx->DR=tmp_str[i];
    while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
  }
  HMI_USARTx->DR=0xFF;
  while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
  HMI_USARTx->DR=0xFF;
  while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
  HMI_USARTx->DR=0xFF;
  while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
}

/**
  * 函数功能: 向串口屏发送浮点数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void HMI_string_setting(const char *val_str,int32_t value)
{
  uint8_t tmp_str[50]={0};
  uint8_t i;
  float temp=(float)value;
  sprintf((char *)tmp_str,"%s=\"%.1f\"",val_str,temp/100);
  
  for(i=0;i<strlen((char *)tmp_str);++i)
  {
    HMI_USARTx->DR=tmp_str[i];
    while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
  }
  HMI_USARTx->DR=0xFF;
  while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
  HMI_USARTx->DR=0xFF;
  while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
  HMI_USARTx->DR=0xFF;
  while(__HAL_UART_GET_FLAG(&husartx_HMI, UART_FLAG_TXE) == RESET);
}




/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

