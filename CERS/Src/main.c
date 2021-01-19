/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 李天宸
  * 版    本: V1.0
  * 编写日期: 2020-12-17
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

#include "DCMotor/bsp_BDCMotor.h" 
#include "key/bsp_key.h"
#include "encoder/bsp_encoder.h"
#include "usart/bsp_usartx.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define HMI_RX_BUFFER_SIZE       30

#define  FLASH_WriteAddress      0x0000
#define  FLASH_SectorToErase    FLASH_WriteAddress

#define HMI_SECTOR_ADDREE       4096*2

/* 私有变量 ------------------------------------------------------------------*/
/* HMI串口变量 ------------------------------------------------------------------*/
__IO uint8_t  HMI_Rx_buf[HMI_RX_BUFFER_SIZE]={0};
__IO uint8_t  HMI_RX_flag=0;       //0:未接收到数据头  1：已经接收到数据头  2：一帧接收完毕
__IO uint8_t  usart_rx_flag;

__IO uint16_t timer_count=0;
__IO uint16_t pwm_data=0;
/* 通道选择变量 ------------------------------------------------------------------*/
__IO uint8_t  model_channelx=0;         //模式功能选择
__IO uint8_t  force_channelx=0;         //力传感器选择
__IO uint8_t  encoder_channelx=0;        //编码器选择
/* 力检测变量 ------------------------------------------------------------------*/
__IO uint8_t  weight_Zero_IsInit=0;     // 0：停止数据采集 1:零值未获取  2：已获取零值
__IO int32_t  weight_Zero_Data=0;       // 无施加力时零值记录值

__IO uint8_t  Is_thres_stamp=0;         //是否有阀值设定
__IO uint8_t  Is_tare_stamp=0;          //是否记录零值
__IO uint8_t  Is_start_stamp=0;          //开始测试

__IO int32_t  Record_weight = 0;           //预紧力值
__IO int32_t  Record_weight1;           //预紧力记录值（用于每次测力前记录零值） 
__IO int32_t  Record_weight2;           //预紧力记录值（用于每次测力前记录零值 ） 
__IO int32_t  cali_weight;              //校准使用
__IO int32_t  weight_proportion = 1950; //换算值记录 比例系数
__IO int32_t  weight_current;           //换算放大后力数值

__IO int32_t  second_count;             //皮重记录值
__IO int32_t  third_count;              //皮重记录值
__IO uint8_t  Force_Process_Step=0;   // 

__IO uint8_t  Test_Step=0;
/* 插补函数变量 ------------------------------------------------------------------*/
__IO int32_t  in0=0;
__IO int32_t  in1=0;
__IO int32_t  in2=0;
__IO int32_t  in3=0;
__IO int32_t  in4=0;
/* 角度检测变量 ------------------------------------------------------------------*/
__IO int32_t CaptureNumber = 0;     // 输入捕获数
__IO int32_t LastCapNum = 0;     // 上一次输入捕获数
__IO int32_t Speed = 0;     // 上一次输入捕获数
__IO int32_t Angle = 0;      // 角度

__IO uint8_t  encoder_Zero_IsInit=0;     // 0：停止数据采集 1:零值未获取  2：已获取零值
__IO int32_t  encoder_Zero_Data=0;       // 无施加力时零值记录值
__IO uint8_t  Encoder_Process_Step=0;   // 
__IO int32_t   Record_encoder = 0;        //预紧角度值
__IO int32_t   Record_encoder1;           //预紧角度记录值（用于每次测角度前记录零值） 
__IO int32_t   Record_encoder2;           //预紧角度记录值（用于每次测角度前记录零值 ）
__IO int64_t data_temp,temp;
__IO int32_t encoder_read;
/* SPI flash变量 ------------------------------------------------------------------*/
uint32_t DeviceID = 0;
uint32_t FlashID = 0;
uint8_t Tx_Buffer[3] = {0};
uint8_t Rx_Buffer[3] = {0};
int32_t Result_data=0;


__IO uint32_t timecount = 0;
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
  int8_t  Compa_encoder_value;//阈值预设值

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
  //LED_GPIO_Init();
  /* 初始化SPI */
  MX_SPIFlash_Init();
  /* 初始化BEEP */
  //BEEP_GPIO_Init();
  /* 按键初始化 */
  //KEY_GPIO_Init();

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
    //未模式选择 可能加复位使能操作

    //测力模式
    if(model_channelx==1)
    {
      if(uwTick%10==0)
      {
        if(weight_Zero_IsInit == 1) //未获取零值
        {
          weight_Zero_Data = weight_ad7190_ReadAvg(4);
          weight_Zero_IsInit = 2;
        }
        else if(weight_Zero_IsInit == 2) //零值已经记录成功
        {
          //int64_t data_temp,temp;
          int64_t weight_read;

          weight_read = weight_ad7190_ReadAvg(1);
          data_temp = weight_read - weight_Zero_Data;
          temp = data_temp * 100000 /weight_proportion;
          weight_current = temp;
          printf("weight*10=%d\n",weight_current/1000); //0.1N 
          printf("Force_Process_Step=%d\n",Force_Process_Step);

          //插补函数补充

          //超出预警阈值报警
          if(Is_thres_stamp==1)  //如果有超出预设值，那么蜂鸣器响 后期可以加语音模块
          {
            if((weight_current-Record_weight)>=Compa_value)
            {
              HMI_value_setting("force1.ad0.val",3);
              HAL_Delay(3000);
            }        
          }

          //清零按钮判断 用于异常消除 温漂严重时
          if(Is_tare_stamp==1)   //如果有清零按钮按下 对应按钮 力通道选择 清零 开始测试 
          {
            Record_weight2=Record_weight1-weight_current;
            if(Record_weight2>15)   
            {
              Record_weight1=0; 
              Record_weight2=0;
              Is_tare_stamp=0;
              Force_Process_Step=0;
              HMI_value_setting("force1.gross.val",0);
              HMI_value_setting("force1.net.val",0); 
            }           
          }

          ///力检测流程
          switch (Force_Process_Step)   
          {
            case 0://检测到有物品
              if(weight_read>weight_Zero_Data)
              {
                Force_Process_Step = 1;
                weight_read = 0;
              }
              else
              {
                HMI_value_setting("force1.gross.val",0);
                Force_Process_Step = 0;
                weight_read = 0;
              }
            break;
            case 1:
              if(int_abs(weight_read,weight_Zero_Data)>10)
              {
                Force_Process_Step=2;
              }
              else 
              {
                HMI_value_setting("force1.gross.val",0);
                Force_Process_Step=0;
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
                data_temp = weight_read - weight_Zero_Data;
                temp=data_temp*100000/weight_proportion;
                Record_weight1=temp; //每次换皮重时赋值
                printf("Record_weight1*10=%d N\n",Record_weight1/1000); //0.1N
                weight_current=temp;            
                //Is_tare_stamp=1;
                HMI_value_setting("force1.gross.val",weight_current/1000); //0.1N
                Force_Process_Step=3;
              }
            break;
            case 3://开始测试 锁定预紧力
              if(Is_start_stamp==1)   //开始测试按钮
              {
                Record_weight=Record_weight1;
                HMI_value_setting("force1.gross.val",Record_weight/1000); //0.1N
                printf("Record_weight*10=%d N\n",Record_weight/1000); //0.1N
                Force_Process_Step = 4;
                HMI_value_setting("force1.ad0.val",0);
                HAL_Delay(4000);
              }
              else
              {
                Force_Process_Step = 2;
              }
            break;
            case 4://力测试 开始判断
              if (Test_Step>=3)
              {
                Force_Process_Step = 6;
              }
              else if(weight_current>Record_weight)   //施加力大于预紧力
              {
                if(int_abs(weight_current,Record_weight)>50000) //初始施加力大于5N
                {
                  weight_read = 0;
                  Force_Process_Step=5;
                  timecount = 0;
                }
                else
                {
                  weight_read = 0;
                }
              }
            break;
            case 5://力测试 持续输出
              if((timecount<=90) && (int_abs(weight_current,Record_weight)>40000))//施加力大于4N 且 未超过计时
              {
                timecount++;
                //插补函数输入 0.1N
                HMI_value_setting("force1.inx.val",1);
                in1=in0+(weight_current-Record_weight-in0)/5;
                in2=in0+2*(weight_current-Record_weight-in0)/5;
                in3=in0+3*(weight_current-Record_weight-in0)/5;
                in4=in0+4*(weight_current-Record_weight-in0)/5;
                in0=weight_current-Record_weight;
                HMI_value_setting("force1.in1.val",in1/10000);
                HMI_value_setting("force1.in2.val",in2/10000);
                HMI_value_setting("force1.in3.val",in3/10000);
                HMI_value_setting("force1.in4.val",in4/10000);
                HMI_value_setting("force1.in0.val",in0/10000);
                printf("in0*10=%d\n",in0/1000); //0.1N

                Force_Process_Step = 5;
                HMI_value_setting("force1.net.val",in0/1000);//0.1N

              }
              else
              {
                Test_Step++;
                timecount = 0;
                Force_Process_Step = 4;
                in0 = in1 = in2 = in3 = 0;
                HMI_value_setting("force1.inx.val",0);
                HMI_value_setting("force1.ad0.val",1);
                HMI_value_setting("force1.chx.val",Test_Step);
                printf("测试 %d 结束\n",Test_Step);
                HAL_Delay(5000);
              }
            break;
            case 6:  
              Test_Step = 0;
              Force_Process_Step = 0;
              Is_start_stamp = 0;
              weight_Zero_IsInit=0;
              HMI_value_setting("force1.ad0.val",2);
              HAL_Delay(3000);

              HMI_value_setting("force1.gross.val",0);
              HMI_value_setting("force1.net.val",0);
              printf("三次测试结束\n");
            break;
          }

        }
      }
    }
    
    //测角度模式
    if(model_channelx==2)
    {
      if(uwTick%10==0)
      {
        if(encoder_Zero_IsInit==1)//未获取零值
        {
          encoder_Zero_Data = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
          encoder_Zero_IsInit = 2;
        }
        else if(encoder_Zero_IsInit==2)//零值已经记录成功
        {
          

          encoder_read = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
          data_temp = encoder_read -encoder_Zero_Data;
          temp = data_temp *3600 / ENCODER_RESOLUTION;
          Angle = temp;//角度*10
          printf("Angle=%d\n",Angle/10);
          printf("Encoder_Process_Step=%d\n",Encoder_Process_Step);

          //插补函数补充

          //超出预警阈值报警
          if(Is_thres_stamp==1)  //如果有超出预设值，那么蜂鸣器响 后期可以加语音模块
          {
            if((Angle-Record_encoder) >= Compa_encoder_value)
            {
              HMI_value_setting("encode1.ad0.val",3);
              HAL_Delay(3000);
            }        
          }

           //清零按钮判断 用于异常消除 
          if(Is_tare_stamp==1)   //如果有清零按钮按下 对应按钮 力通道选择 清零 开始测试 
          {
            Record_encoder2=Record_encoder1-Angle;
            if(Record_encoder2>10)   
            {
              Record_encoder1=0; 
              Record_encoder2=0;
              Is_tare_stamp=0;
              Encoder_Process_Step=0;
              HMI_value_setting("encode1.gross.val",0);
              HMI_value_setting("encode1.net.val",0); 
            }           
          }

          //角度检测流程
          switch (Encoder_Process_Step)
          {
            case 0:
             if(encoder_read>encoder_Zero_Data)
             {
               Encoder_Process_Step = 1;
               encoder_read = 0;
             }
             else
             {
              HMI_value_setting("encode1.gross.val",0);
              Encoder_Process_Step=0;
             }
            break;
            case 1:
              if(int_abs(encoder_read,encoder_Zero_Data)>5)
              {
                Encoder_Process_Step=2;
              }
              else 
              {
                HMI_value_setting("encode1.gross.val",0);
                Encoder_Process_Step=0;
              }
            break;
            case 2://检测初始角度
              HAL_Delay(100);
              second_count = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
              if(int_abs(second_count,encoder_read)<1000) //根据测试情况更改
              {
                HAL_Delay(10);
                encoder_read = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
                third_count = encoder_read;
                data_temp = encoder_read -encoder_Zero_Data;
                temp = data_temp *3600 / ENCODER_RESOLUTION;
                Record_encoder1=temp; //每次换皮重时赋值
                printf("Record_encoder1=%d 度\n",Record_encoder1/10); 
                Angle=temp;            
                //Is_tare_stamp=1;
                HMI_value_setting("encode1.gross.val",Angle); 
                Encoder_Process_Step=3;
              }
            break;
            case 3://开始测试 锁定预紧角度
              if(Is_start_stamp==1)   //开始测试按钮
              {
                Record_encoder=Record_encoder1;
                HMI_value_setting("encode1.gross.val",Record_encoder1); 
                printf("Record_encoder1=%d 度\n",Record_encoder1); 
                Encoder_Process_Step = 4;
                HMI_value_setting("encode1.ad0.val",0);
                HAL_Delay(4000);
              }
              else
              {
                Encoder_Process_Step = 2;
              }
            break;
            case 4://角度测试 开始判断
              if (Test_Step>=3)
              {
                Encoder_Process_Step = 6;
              }
              else if(Angle>Record_encoder)   //角度大于预紧角度
              {
                if(int_abs(Angle,Record_encoder)>1) //角度大于1°
                {
                  encoder_read = 0;
                  Encoder_Process_Step =5;
                  timecount = 0;
                }
                else
                {
                  encoder_read = 0;
                }
              }
            break;
            case 5://力测试 持续输出
              if((timecount<=90) && (int_abs(Angle,Record_encoder)>1))//施加力大于1° 且 未超过计时
              {
                timecount++;
                //插补函数输入 
                HMI_value_setting("encode1.inx.val",1);
                in1=in0+(Angle-Record_encoder-in0)/5;
                in2=in0+2*(Angle-Record_encoder-in0)/5;
                in3=in0+3*(Angle-Record_encoder-in0)/5;
                in4=in0+4*(Angle-Record_encoder-in0)/5;
                in0=Angle-Record_encoder;
                HMI_value_setting("encode1.in1.val",in1/10);
                HMI_value_setting("encode1.in2.val",in2/10);
                HMI_value_setting("encode1.in3.val",in3/10);
                HMI_value_setting("encode1.in4.val",in4/10);
                HMI_value_setting("encode1.in0.val",in0/10);
                printf("in0=%d 度\n",in0/100); 

                Encoder_Process_Step = 5;
                HMI_value_setting("encode1.net.val",in0);

              }
              else
              {
                Test_Step++;
                timecount = 0;
                Encoder_Process_Step = 4;
                in0 = in1 = in2 = in3 = 0;
                HMI_value_setting("encode1.inx.val",0);
                HMI_value_setting("encode1.ad0.val",1);
                HMI_value_setting("encode1.chx.val",Test_Step);
                printf("测试 %d 结束\n",Test_Step);
                HAL_Delay(5000);
              }
            break;
            case 6:  
              Test_Step = 0;
              Encoder_Process_Step = 0;
              Is_start_stamp = 0;
              encoder_Zero_IsInit=0;
              HMI_value_setting("encode1.ad0.val",2);
              HAL_Delay(3000);

              HMI_value_setting("encode1.gross.val",0);
              HMI_value_setting("encode1.net.val",0);
              printf("三次测试结束\n");
            break;


          }

        }


      }
    }

    if(HMI_RX_flag==2)
    {
      HMI_RX_flag=0;    
      switch(HMI_Rx_buf[1])
      {
        //功能选择界面 home
        case 0x01:
          printf("等长肌力测试界面\n"); 
          model_channelx=1;       
        break;
        case 0x02:
          printf("活动范围功能选择界面\n"); 
          model_channelx=2;       
        break;
        
        //等长肌力测试界面 force0
        case 0x10:
          printf("功能选择界面\n");
          model_channelx=0;        
        break;
        case 0x11:
          printf("前屈1选择\n");
          force_channelx=1;
          weight_proportion = 2050; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSx_ENABLE();
          weight_ad7190_conf(force_channelx);       
        break;
        case 0x12:
          printf("后伸2选择\n");
          force_channelx=2;
          weight_proportion = 1760;
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSx_ENABLE();  
          weight_ad7190_conf(force_channelx);      
        break;
        case 0x13:
          printf("左侧屈3选择\n");
          force_channelx=1; 
          weight_proportion = 1700; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSy_ENABLE(); 
          weight_ad7190_conf(force_channelx);     
        break;
        case 0x14:
          printf("右侧屈4选择\n");
          force_channelx=2;
          weight_proportion = 1744; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSy_ENABLE();
          weight_ad7190_conf(force_channelx);       
        break;

        //活动范围检测 encode0
        case 0x20:
          printf("功能选择界面\n");
          model_channelx=0;
          /* 编码器初始化及使能编码器模式 */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");   

        break;
        case 0x21:
          printf("前屈1选择\n");
          encoder_channelx=1;
          /* 编码器初始化及使能编码器模式 */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");

        break;
        case 0x22:
          printf("后伸2选择\n");
          encoder_channelx=1;
          /* 编码器初始化及使能编码器模式 */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");
           
        break;
        case 0x23:
          printf("左侧屈3选择\n");
          encoder_channelx=1;
          /* 编码器初始化及使能编码器模式 */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");
               
        break;
        case 0x24:
          printf("右侧屈4选择\n");
          encoder_channelx=1;
          /* 编码器初始化及使能编码器模式 */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");
                
        break;
        case 0x25:
          printf("左旋5选择\n");
          encoder_channelx=2;
          /* 编码器初始化及使能编码器模式 */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");
             
        break;
        case 0x26:
          printf("右旋6选择\n");
          encoder_channelx=2;
          /* 编码器初始化及使能编码器模式 */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");
               
        break;

        //力传感器检测界面 force1
        case 0x30:
          printf("肌力测试功能选择界面");
          force_channelx = 0;
          weight_Zero_IsInit = 0;//停止传输数据
          Is_start_stamp = 0;
          Test_Step = 0;
          Force_Process_Step = 0;
        break;
        case 0x31:
          printf("力传感器 %d 校准\n",force_channelx);
        break;
        case 0x32:
          printf("清零\n");
          Force_Process_Step=0;
          Is_tare_stamp=0;
          weight_Zero_IsInit=1;
          HMI_value_setting("force1.gross.val",0);
          HMI_value_setting("force1.net.val",0);        
        break;
        case 0x33:
          printf("开始测试\n");
          Is_start_stamp = 1;
          Force_Process_Step = 3;       
        break;
        case 0x39:         
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2]; 
          Compa_value=tmp[0]*1000;   //0.1N 
          Is_thres_stamp=1; 
          if(Compa_value==0)
          {
            Is_thres_stamp=0;
          }            
          printf("Compa_value*10=%d\n N",Compa_value/1000);
        break;

        //活动范围检测界面 encode1
        case 0x40:
          printf("活动范围检测功能选择界面");
          encoder_channelx = 0;
          encoder_Zero_IsInit = 0;//停止传输数据
          Is_start_stamp = 0;
          Test_Step = 0;
          Encoder_Process_Step = 0;
        break;
        case 0x42:
          printf("清零\n");
          Encoder_Process_Step=0;
          Is_tare_stamp=0;
          encoder_Zero_IsInit=1;
          HMI_value_setting("encode1.gross.val",0);
          HMI_value_setting("encode1.net.val",0);        
        break;
        case 0x43:
          printf("开始测试\n");
          Is_start_stamp = 1;
          Encoder_Process_Step = 3;       
        break;
        case 0x49:         
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2]; 
          Compa_encoder_value=tmp[0]*10;   
          Is_thres_stamp=1; 
          if(Compa_encoder_value==0)
          {
            Is_thres_stamp=0;
          }            
          printf("Compa_value*10=%d\n N",Compa_encoder_value/1000);
        break;

        //力传感器校准界面
        case 0x51:
          printf("第一步\n");         
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2];
          cali_weight= tmp[0]%10000;
          weight_Zero_Data = weight_ad7190_ReadAvg(4);       
          weight_Zero_IsInit=0;
        break;
        case 0x52:
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
          printf("weight_proportion=%d\n",weight_proportion);         
        }
        break;
        case 0x53:
          printf("校准完成\n");
          Force_Process_Step=0;
          weight_Zero_IsInit=1;   
          HMI_value_setting("page0.gross.val",0);
          HMI_value_setting("page0.net.val",0);           
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




/*****END OF FILE****/

