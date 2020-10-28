/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2018-04-21
  * ��    ��: �������ӿ�����
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "DCMotor/bsp_BDCMotor.h" 
#include "key/bsp_key.h"
#include "encoder/bsp_encoder.h"
#include "usart/bsp_usartx.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO int32_t XCaptureNumber = 0;     // ���벶����
__IO int32_t XLastCapNum = 0;     // ��һ�����벶����
__IO int32_t XSpeed = 0;     // ��һ�����벶����

__IO int32_t YCaptureNumber = 0;     // ���벶����
__IO int32_t YLastCapNum = 0;     // ��һ�����벶����
__IO int32_t YSpeed = 0;     // ��һ�����벶����

/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint32_t uwTick;
extern int32_t XOverflowCount ;//��ʱ��������� 

extern int32_t YOverflowCount ;//��ʱ��������� 
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
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     // ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // ���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // ��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // ��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/������������ȵ���PLL��Ƶϵ��
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // ���ò�����ϵͳ�δ�ʱ��
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
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  /* ���ڳ�ʼ�� */
  MX_USARTx_Init();
  /* ������ʼ�� */
  KEY_GPIO_Init();
  /* ��������ʼ����ʹ�ܱ�����ģʽ */
  XENCODER_TIMx_Init();

  YENCODER_TIMx_Init();

  HAL_TIM_Encoder_Start(&xhtimx_Encoder, TIM_CHANNEL_ALL);

  HAL_TIM_Encoder_Start(&yhtimx_Encoder, TIM_CHANNEL_ALL);

  printf("--> �������ٶ�λ�ö�ȡ���� <-- \n");
  printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");
  /* ����ѭ�� */
  while (1)
  {
    if(uwTick % 100 ==0)  // 100ms
    {
      /* ��ȡ����������ֵ */
      XCaptureNumber = (XOverflowCount*XCNT_MAX) + __HAL_TIM_GET_COUNTER(&xhtimx_Encoder);
      XSpeed = XCaptureNumber - XLastCapNum;   //�õ�100ms�ڵĲ���ֵ
      XLastCapNum = XCaptureNumber;

      YCaptureNumber = (YOverflowCount*YCNT_MAX) + __HAL_TIM_GET_COUNTER(&yhtimx_Encoder);
      YSpeed = YCaptureNumber - YLastCapNum;   //�õ�100ms�ڵĲ���ֵ
      YLastCapNum = YCaptureNumber;
      
      /* �����ٶ�λ��(Ȧ��r) */
      printf("X��");
      printf("���벶��ֵ��%d \n",XCaptureNumber);
      printf("�г̣�%.3f r \n",(float)((float)XCaptureNumber/XENCODER_RESOLUTION));
      /* �ٶȵ�λ��r/s, */
      printf("�ٶȣ�%.3f r/s \n",(float)((float)XSpeed/XENCODER_RESOLUTION *10));

      printf("Y��");
      printf("���벶��ֵ��%d \n",YCaptureNumber);
      printf("�г̣�%.3f r \n",(float)((float)YCaptureNumber/YENCODER_RESOLUTION));
      /* �ٶȵ�λ��r/s, */
      printf("�ٶȣ�%.3f r/s \n",(float)((float)YSpeed/YENCODER_RESOLUTION *10));
    }
		//HAL_Delay(10);
  }
	
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
