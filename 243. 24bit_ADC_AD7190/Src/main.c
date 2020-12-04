#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart/bsp_debug_usart.h"
#include "led/bsp_led.h"
#include "weight/bsp_weight.h"
#include "beep/bsp_beep.h"
#include "key/bsp_key.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO float weight;
__IO int32_t weight_proportion=2000;  // ��ѹֵ�������任�����������Ҫʵ�ʲ��Լ�����ܵõ�
__IO int32_t weight_Zero_Data=0;   // ��ֵ
__IO float  weight_k=500;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  float data_temp;      
  int32_t weight_count;  
  uint8_t cali_flag=0;
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  /* ��ʼ��LED */
  LED_GPIO_Init(); 
  
  KEY_GPIO_Init();
  /* ��ʼ��BEEP */
  BEEP_GPIO_Init();
  
  if(AD7190_Init()==0)
  {
    printf("��ȡ���� AD7190 !\n");
    while(1)
    {
      HAL_Delay(1000);
      if(AD7190_Init())
        break;
    }
  }
  printf("��⵽  AD7190 !\n");
  weight_ad7190_conf();
  
  HAL_Delay(500);
  weight_Zero_Data = weight_ad7190_ReadAvg(6);
  printf("zero:%d\n",weight_Zero_Data);
  while(1)
  {
    weight_count=weight_ad7190_ReadAvg(3);
    data_temp=weight_count-weight_Zero_Data;
    weight=data_temp*1000/weight_proportion;
    printf("������%d->%.2f\n",weight_count,weight);
    HAL_Delay(50);
    if(KEY1_StateRead()==KEY_DOWN)  // ����
    {      
      weight_Zero_Data = weight_ad7190_ReadAvg(6);
      printf("zero:%d\n",weight_Zero_Data);
      cali_flag=1;
    }
    if(KEY2_StateRead()==KEY_DOWN) // У׼�������Ȱ������㡱����Ȼ���500g������ڳ��ϣ�����У׼��
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






