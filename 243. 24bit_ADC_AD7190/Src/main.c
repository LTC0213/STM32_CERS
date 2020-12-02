#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart/bsp_debug_usart.h"
#include "led/bsp_led.h"
#include "weight/bsp_weight.h"
#include "beep/bsp_beep.h"
#include "key/bsp_key.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
__IO float weight;
__IO int32_t weight_proportion=2000;  // 电压值与重量变换比例，这个需要实际测试计算才能得到
__IO int32_t weight_Zero_Data=0;   // 零值
__IO float  weight_k=500;

/* 扩展变量 ------------------------------------------------------------------*/
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

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  float data_temp;      
  int32_t weight_count;  
  uint8_t cali_flag=0;
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  /* 初始化LED */
  LED_GPIO_Init(); 
  
  KEY_GPIO_Init();
  /* 初始化BEEP */
  BEEP_GPIO_Init();
  
  if(AD7190_Init()==0)
  {
    printf("获取不到 AD7190 !\n");
    while(1)
    {
      HAL_Delay(1000);
      if(AD7190_Init())
        break;
    }
  }
  printf("检测到  AD7190 !\n");
  weight_ad7190_conf();
  
  HAL_Delay(500);
  weight_Zero_Data = weight_ad7190_ReadAvg(6);
  printf("zero:%d\n",weight_Zero_Data);
  while(1)
  {
    weight_count=weight_ad7190_ReadAvg(3);
    data_temp=weight_count-weight_Zero_Data;
    weight=data_temp*1000/weight_proportion;
    printf("重量：%d->%.2f\n",weight_count,weight);
    HAL_Delay(50);
    if(KEY1_StateRead()==KEY_DOWN)  // 清零
    {      
      weight_Zero_Data = weight_ad7190_ReadAvg(6);
      printf("zero:%d\n",weight_Zero_Data);
      cali_flag=1;
    }
    if(KEY2_StateRead()==KEY_DOWN) // 校准：必须先按“清零”键，然后把500g砝码放在称上，按下校准键
    {
      if(cali_flag)
      {
        weight_count = weight_ad7190_ReadAvg(6);
        weight_proportion=(weight_count-weight_Zero_Data)*1000/weight_k;
        printf("weight_proportion:%d\n",weight_proportion);
      }
      cali_flag=0;
    }
  }
}







