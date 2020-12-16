/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: LTC
  * 版    本: V1.0
  * 编写日期: 2020-12-16
  * 功    能: CERS功能实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "string.h"
// #include "usart/bsp_debug_usart.h"
#include "led/bsp_led.h"
#include "HMI/bsp_usartx_HMI.h"
#include "spiflash/bsp_spiweight.h"
#include "spiflash/bsp_spiflash.h"
#include "beep/bsp_beep.h"
#include "usart/bsp_usartx.h"
#include "encoder/bsp_encoder.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define HMI_RX_BUFFER_SIZE       30
#define  FLASH_WriteAddress      0x0000
#define  FLASH_SectorToErase    FLASH_WriteAddress
#define HMI_SECTOR_ADDREE       4096*2
/* 私有变量 ------------------------------------------------------------------*/
/* 串口变量 ------------------------------------------------------------------*/
__IO uint8_t  HMI_Rx_buf[HMI_RX_BUFFER_SIZE]={0};
__IO uint8_t  HMI_RX_flag=0;       //0:未接收到数据头  1：已经接收到数据头  2：一帧接收完毕
__IO uint8_t  usart_rx_flag;
__IO uint16_t timer_count=0;
__IO uint16_t pwm_data=0;
/* 通道选择变量 ------------------------------------------------------------------*/
__IO uint8_t  model_channelx=0;         //模式功能选择
__IO uint8_t  force_channelx=0;         //力传感器选择
__IO uint8_t  encode_channelx=0;        //编码器选择
/* 力检测变量 ------------------------------------------------------------------*/
__IO uint8_t  weight_Zero_IsInit=0;     // 1:零值未获取  2：已获取零值
__IO int32_t  weight_Zero_Data=0;       // 无施加力时零值记录值
__IO uint8_t  Is_thres_stamp=0;         //是否有阀值设定
__IO uint8_t  Is_tare_stamp=0;          //是否记录零值
__IO uint8_t  Is_start_stamp=0;          //开始测试
__IO int32_t  Record_weight;           //预紧力值
__IO int32_t  Record_weight1;           //预紧力记录值（用于每次测力前记录零值） 
__IO int32_t  Record_weight2;           //预紧力记录值（用于每次测力前记录零值 ） 
__IO int32_t  cali_weight;              //校准使用
__IO int32_t  weight_proportion = 1950; //换算值记录 比例系数
__IO int32_t  weight_current;           //换算放大后力数值
__IO int32_t  second_count;             //皮重记录值
__IO int32_t  third_count;              //皮重记录值
__IO uint8_t  Process_Step=0;   // 0未检测到物体 1检测到有物品 2有重物 并且达到皮重要求 3测皮重 4皮重基础上 加了重物 5
/* 插补函数变量 ------------------------------------------------------------------*/
__IO int8_t  in0=0;
__IO int8_t  in1=0;
__IO int8_t  in2=0;
__IO int8_t  in3=0;
/* 角度检测变量 ------------------------------------------------------------------*/
__IO int32_t CaptureNumber = 0;     // 输入捕获数
__IO int32_t LastCapNum = 0;     // 上一次输入捕获数
__IO int32_t Speed = 0;     // 上一次输入捕获数
/* SPI flash变量 ------------------------------------------------------------------*/
uint32_t DeviceID = 0;
uint32_t FlashID = 0;
uint8_t Tx_Buffer[3] = {0};
uint8_t Rx_Buffer[3] = {0};
int32_t Result_data=0;

/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint32_t uwTick;  //时钟计数
extern int32_t OverflowCount ;//定时器溢出次数 

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
  int32_t weight_first;
  uint32_t tmp[2];
  int32_t Compa_value;//阈值预设值

  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  /* 初始化串口并配置串口中断优先级 */
  // MX_DEBUG_USART_Init();
  MX_USARTx_Init();
  HMI_USARTx_Init();
  __HAL_UART_ENABLE_IT(&husartx_HMI, UART_IT_RXNE);
  /* 初始化LED */
  LED_GPIO_Init();
  /* 初始化SPI */
  MX_SPIFlash_Init();
  /* 初始化BEEP */
  BEEP_GPIO_Init();

  /* 编码器初始化及使能编码器模式 */
  ENCODER_TIMx_Init();
  HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
  printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");

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
      weight_Zero_IsInit=1; //开始接收零值
    }      
  }  	  
  if(AD7190_Init())
  {
    printf("检测到  AD7190 !\n");
    // weight_ad7190_conf(channelx);
  }

  /* 无限循环 */
  while(1)
  {
    switch(model_channelx)
    case 0: //未模式选择 可能加复位使能操作
    break;
    case 1:
      if(uwTick % 10 == 0)
      {
        if(weight_Zero_IsInit == 1) //未获取零值
        {
          weight_Zero_Data = weight_ad7190_ReadAvg(4);
          weight_Zero_IsInit = 2;
        }
        else if(weight_Zero_IsInit == 2) //零值已经记录成功
        {
          int64_t data_temp;
          int64_t temp,weight_read;

          weight_read = weight_ad7190_ReadAvg(1);
          date_temp = weight_read - weight_Zero_Data;
          temp = date_temp * 100000 /weight_proportion;
          weight_current = temp;
          printf("weight*10=%d\n",weight_current/1000); //0.1N 

          //插补函数补充

          if(Is_thres_stamp==1)  //如果有超出预设值，那么蜂鸣器响 后期可以加语音模块
          {
            if((weight_current-Record_weight1)>=Compa_value)
            {
              BEEP_ON;
              HAL_Delay(200);
              BEEP_OFF;
            }        
          }

          if(Is_tare_stamp==1)   //如果有清零按钮按下 对应按钮 力通道选择 清零 开始测试 
          {
            Record_weight2=Record_weight1-weight_current;
            if(Record_weight2>15)   
            {
              Record_weight1=0; 
              Record_weight2=0;
              Is_tare_stamp=0;
              Process_Step=0;
              HMI_value_setting("force1.gross.val",0);
              HMI_value_setting("force1.net.val",0); 
            }           
          }

          //
          switch (Process_Step)
          {
            case 0://检测到有物品
              if(weight_read>weight_Zero_Data)
              {
                Process_Step = 1;
                weight_read = 0;
              }
              else
              {
                HMI_value_setting("force1.gross.val",0);
                Process_Step = 0;
                weight_read = 0;
              }
            break;
            case 1:
              if(int_abs(weight_read,weight_Zero_Data)>10)
              {
                Process_Step=2;
              }
              else 
              {
                HMI_value_setting("force1.gross.val",0);
                Process_Step=0;
              }
            break;
            case 2://检测预紧力
              HAL_Delay(100);
              second_count=weight_ad7190_ReadAvg(1);
              if(int_abs(second_count,weight_read)<1000) //根据测试情况更改
              {
                HAL_Delay(10);
                weight_read = weight_ad7190_ReadAvg(4);
                third_count = weight_read;
                date_temp = weight_read - weight_Zero_Data;
                temp=data_temp*100000/weight_proportion;
                Record_weight1=temp; //每次换皮重时赋值
                printf("Record_weight1=%d\n",Record_weight1/10000); 
                weight_read=temp;            
                Is_tare_stamp=1;
                HMI_value_setting("force1.gross.val",weight_read/1000); //0.1N
                Process_Step=3;
              }
              else
              {
                Process_Step = 2;
              }
            break;
            case 3://开始测试 锁定预紧力
              if(Is_start_stamp==1)
              {
                Record_weight=Record_weight1;
                HMI_value_setting("force1.gross.val",weight_read/1000); //0.1N
                printf("Record_weight*10=%d\n",Record_weight/1000); //0.1N
                Process_Step = 4;
                //后续替换成语音
                BEEP_ON;
                HAL_Delay(300);
                BEEP_OFF;
              }
              else
              {
                Process_Step = 2;
              }
            break;
            case 4://力测试 开始判断
              if(weight_read>Record_weight)//施加力大于预紧力
              {
                if(int_abs(weight_read,Record_weight)>5000) //初始施加力大于0.5N
                {
                  weight_read=0;
                  Process_Step=5;
                }

              }

          }




        }
      }


  }





}