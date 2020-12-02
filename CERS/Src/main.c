/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2018-04-21
  * 功    能: 编码器接口例程
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "DCMotor/bsp_BDCMotor.h" 
#include "key/bsp_key.h"
#include "encoder/bsp_encoder.h"
#include "usart/bsp_usartx.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
__IO int32_t XCaptureNumber = 0;     // 输入捕获数
__IO int32_t XLastCapNum = 0;     // 上一次输入捕获数
__IO int32_t XSpeed = 0;     // 上一次输入捕获数

__IO int32_t YCaptureNumber = 0;     // 输入捕获数
__IO int32_t YLastCapNum = 0;     // 上一次输入捕获数
__IO int32_t YSpeed = 0;     // 上一次输入捕获数

/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint32_t uwTick;
extern int32_t XOverflowCount ;//定时器溢出次数 

extern int32_t YOverflowCount ;//定时器溢出次数 
/* 私有函数原形 --------------------------------------------------------------*/
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
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     // 使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // 设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // 打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // 打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/随机数产生器等的主PLL分频系数
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{ 
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  /* 串口初始化 */
  MX_USARTx_Init();
  /* 按键初始化 */
  KEY_GPIO_Init();
  /* 编码器初始化及使能编码器模式 */
  XENCODER_TIMx_Init();

  YENCODER_TIMx_Init();

  HAL_TIM_Encoder_Start(&xhtimx_Encoder, TIM_CHANNEL_ALL);

  HAL_TIM_Encoder_Start(&yhtimx_Encoder, TIM_CHANNEL_ALL);

  printf("--> 编码器速度位置读取例程 <-- \n");
  printf("--> 编码器接口4倍频,上下边沿都计数<-- \n");
  /* 无限循环 */
  while (1)
  {
    if(uwTick % 100 ==0)  // 100ms
    {
      /* 读取编码器计数值 */
      XCaptureNumber = (XOverflowCount*XCNT_MAX) + __HAL_TIM_GET_COUNTER(&xhtimx_Encoder);
      XSpeed = XCaptureNumber - XLastCapNum;   //得到100ms内的捕获值
      XLastCapNum = XCaptureNumber;

      YCaptureNumber = (YOverflowCount*YCNT_MAX) + __HAL_TIM_GET_COUNTER(&yhtimx_Encoder);
      YSpeed = YCaptureNumber - YLastCapNum;   //得到100ms内的捕获值
      YLastCapNum = YCaptureNumber;
      
      /* 计算速度位置(圈数r) */
      printf("X轴");
      printf("输入捕获值：%d \n",XCaptureNumber);
      printf("行程：%.3f r \n",(float)((float)XCaptureNumber/XENCODER_RESOLUTION));
      /* 速度单位是r/s, */
      printf("速度：%.3f r/s \n",(float)((float)XSpeed/XENCODER_RESOLUTION *10));

      printf("Y轴");
      printf("输入捕获值：%d \n",YCaptureNumber);
      printf("行程：%.3f r \n",(float)((float)YCaptureNumber/YENCODER_RESOLUTION));
      /* 速度单位是r/s, */
      printf("速度：%.3f r/s \n",(float)((float)YSpeed/YENCODER_RESOLUTION *10));
    }
		//HAL_Delay(10);
  }
	
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
