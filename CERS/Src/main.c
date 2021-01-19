/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: �����
  * ��    ��: V1.0
  * ��д����: 2020-12-17
  * ��    ��: CERS����ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
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

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define HMI_RX_BUFFER_SIZE       30

#define  FLASH_WriteAddress      0x0000
#define  FLASH_SectorToErase    FLASH_WriteAddress

#define HMI_SECTOR_ADDREE       4096*2

/* ˽�б��� ------------------------------------------------------------------*/
/* HMI���ڱ��� ------------------------------------------------------------------*/
__IO uint8_t  HMI_Rx_buf[HMI_RX_BUFFER_SIZE]={0};
__IO uint8_t  HMI_RX_flag=0;       //0:δ���յ�����ͷ  1���Ѿ����յ�����ͷ  2��һ֡�������
__IO uint8_t  usart_rx_flag;

__IO uint16_t timer_count=0;
__IO uint16_t pwm_data=0;
/* ͨ��ѡ����� ------------------------------------------------------------------*/
__IO uint8_t  model_channelx=0;         //ģʽ����ѡ��
__IO uint8_t  force_channelx=0;         //��������ѡ��
__IO uint8_t  encoder_channelx=0;        //������ѡ��
/* �������� ------------------------------------------------------------------*/
__IO uint8_t  weight_Zero_IsInit=0;     // 0��ֹͣ���ݲɼ� 1:��ֵδ��ȡ  2���ѻ�ȡ��ֵ
__IO int32_t  weight_Zero_Data=0;       // ��ʩ����ʱ��ֵ��¼ֵ

__IO uint8_t  Is_thres_stamp=0;         //�Ƿ��з�ֵ�趨
__IO uint8_t  Is_tare_stamp=0;          //�Ƿ��¼��ֵ
__IO uint8_t  Is_start_stamp=0;          //��ʼ����

__IO int32_t  Record_weight = 0;           //Ԥ����ֵ
__IO int32_t  Record_weight1;           //Ԥ������¼ֵ������ÿ�β���ǰ��¼��ֵ�� 
__IO int32_t  Record_weight2;           //Ԥ������¼ֵ������ÿ�β���ǰ��¼��ֵ �� 
__IO int32_t  cali_weight;              //У׼ʹ��
__IO int32_t  weight_proportion = 1950; //����ֵ��¼ ����ϵ��
__IO int32_t  weight_current;           //����Ŵ������ֵ

__IO int32_t  second_count;             //Ƥ�ؼ�¼ֵ
__IO int32_t  third_count;              //Ƥ�ؼ�¼ֵ
__IO uint8_t  Force_Process_Step=0;   // 

__IO uint8_t  Test_Step=0;
/* �岹�������� ------------------------------------------------------------------*/
__IO int32_t  in0=0;
__IO int32_t  in1=0;
__IO int32_t  in2=0;
__IO int32_t  in3=0;
__IO int32_t  in4=0;
/* �Ƕȼ����� ------------------------------------------------------------------*/
__IO int32_t CaptureNumber = 0;     // ���벶����
__IO int32_t LastCapNum = 0;     // ��һ�����벶����
__IO int32_t Speed = 0;     // ��һ�����벶����
__IO int32_t Angle = 0;      // �Ƕ�

__IO uint8_t  encoder_Zero_IsInit=0;     // 0��ֹͣ���ݲɼ� 1:��ֵδ��ȡ  2���ѻ�ȡ��ֵ
__IO int32_t  encoder_Zero_Data=0;       // ��ʩ����ʱ��ֵ��¼ֵ
__IO uint8_t  Encoder_Process_Step=0;   // 
__IO int32_t   Record_encoder = 0;        //Ԥ���Ƕ�ֵ
__IO int32_t   Record_encoder1;           //Ԥ���Ƕȼ�¼ֵ������ÿ�β�Ƕ�ǰ��¼��ֵ�� 
__IO int32_t   Record_encoder2;           //Ԥ���Ƕȼ�¼ֵ������ÿ�β�Ƕ�ǰ��¼��ֵ ��
__IO int64_t data_temp,temp;
__IO int32_t encoder_read;
/* SPI flash���� ------------------------------------------------------------------*/
uint32_t DeviceID = 0;
uint32_t FlashID = 0;
uint8_t Tx_Buffer[3] = {0};
uint8_t Rx_Buffer[3] = {0};
int32_t Result_data=0;


__IO uint32_t timecount = 0;
/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint32_t uwTick;  //ʱ�Ӽ���
extern int32_t OverflowCount ;//��ʱ��������� 

/* ˽�к���ԭ�� --------------------------------------------------------------*/
void HMI_value_setting(const char *val_str,uint32_t value);
void HMI_string_setting(const char *val_str,int32_t value);

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

int32_t int_abs(int32_t value1,int32_t value2)
{
  if(value1>value2)
    return (value1-value2);
  else
    return (value2-value1);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{

  int32_t weight_first;
  uint32_t tmp[2];
  int32_t Compa_value;//��ֵԤ��ֵ
  int8_t  Compa_encoder_value;//��ֵԤ��ֵ

  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  // MX_DEBUG_USART_Init();
  MX_USARTx_Init();
  HMI_USARTx_Init();
  __HAL_UART_ENABLE_IT(&husartx_HMI, UART_IT_RXNE);
  /* ��ʼ��LED */
  //LED_GPIO_Init();
  /* ��ʼ��SPI */
  MX_SPIFlash_Init();
  /* ��ʼ��BEEP */
  //BEEP_GPIO_Init();
  /* ������ʼ�� */
  //KEY_GPIO_Init();

  /* Get SPI Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
  HAL_Delay(100);
  FlashID = SPI_FLASH_ReadID(); 
  printf("FlashID is 0x%X,  Manufacturer Device ID is 0x%X\n", FlashID, DeviceID);
  
  if (FlashID == SPI_FLASH_ID)  /* #define  sFLASH_ID  0XEF4018 */
	{	
		printf("��⵽�����flash W25Q128 !\n");
    SPI_FLASH_BufferRead(Rx_Buffer,HMI_SECTOR_ADDREE,sizeof(Rx_Buffer));
    Result_data=Rx_Buffer[0]*10000+Rx_Buffer[1]*100+Rx_Buffer[2];
    printf("Result_data=%d\n",Result_data);		
    
    if((Result_data>30550)&&(Result_data<30950)) 
    {
      weight_proportion=Result_data;
      weight_Zero_IsInit=1; //��ʼ������ֵ
    }      
  }  	  
  if(AD7190_Init())
  {
    printf("��⵽  AD7190 !\n");
    // weight_ad7190_conf(channelx);
  }

  /* ����ѭ�� */
  while(1)
  {
    //δģʽѡ�� ���ܼӸ�λʹ�ܲ���

    //����ģʽ
    if(model_channelx==1)
    {
      if(uwTick%10==0)
      {
        if(weight_Zero_IsInit == 1) //δ��ȡ��ֵ
        {
          weight_Zero_Data = weight_ad7190_ReadAvg(4);
          weight_Zero_IsInit = 2;
        }
        else if(weight_Zero_IsInit == 2) //��ֵ�Ѿ���¼�ɹ�
        {
          //int64_t data_temp,temp;
          int64_t weight_read;

          weight_read = weight_ad7190_ReadAvg(1);
          data_temp = weight_read - weight_Zero_Data;
          temp = data_temp * 100000 /weight_proportion;
          weight_current = temp;
          printf("weight*10=%d\n",weight_current/1000); //0.1N 
          printf("Force_Process_Step=%d\n",Force_Process_Step);

          //�岹��������

          //����Ԥ����ֵ����
          if(Is_thres_stamp==1)  //����г���Ԥ��ֵ����ô�������� ���ڿ��Լ�����ģ��
          {
            if((weight_current-Record_weight)>=Compa_value)
            {
              HMI_value_setting("force1.ad0.val",3);
              HAL_Delay(3000);
            }        
          }

          //���㰴ť�ж� �����쳣���� ��Ư����ʱ
          if(Is_tare_stamp==1)   //��������㰴ť���� ��Ӧ��ť ��ͨ��ѡ�� ���� ��ʼ���� 
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

          ///���������
          switch (Force_Process_Step)   
          {
            case 0://��⵽����Ʒ
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
            case 2://���Ԥ����
              HAL_Delay(100);
              second_count=weight_ad7190_ReadAvg(1);
              if(int_abs(second_count,weight_read)<1000) //���ݲ����������
              {
                HAL_Delay(10);
                weight_read = weight_ad7190_ReadAvg(4);
                third_count = weight_read;
                data_temp = weight_read - weight_Zero_Data;
                temp=data_temp*100000/weight_proportion;
                Record_weight1=temp; //ÿ�λ�Ƥ��ʱ��ֵ
                printf("Record_weight1*10=%d N\n",Record_weight1/1000); //0.1N
                weight_current=temp;            
                //Is_tare_stamp=1;
                HMI_value_setting("force1.gross.val",weight_current/1000); //0.1N
                Force_Process_Step=3;
              }
            break;
            case 3://��ʼ���� ����Ԥ����
              if(Is_start_stamp==1)   //��ʼ���԰�ť
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
            case 4://������ ��ʼ�ж�
              if (Test_Step>=3)
              {
                Force_Process_Step = 6;
              }
              else if(weight_current>Record_weight)   //ʩ��������Ԥ����
              {
                if(int_abs(weight_current,Record_weight)>50000) //��ʼʩ��������5N
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
            case 5://������ �������
              if((timecount<=90) && (int_abs(weight_current,Record_weight)>40000))//ʩ��������4N �� δ������ʱ
              {
                timecount++;
                //�岹�������� 0.1N
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
                printf("���� %d ����\n",Test_Step);
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
              printf("���β��Խ���\n");
            break;
          }

        }
      }
    }
    
    //��Ƕ�ģʽ
    if(model_channelx==2)
    {
      if(uwTick%10==0)
      {
        if(encoder_Zero_IsInit==1)//δ��ȡ��ֵ
        {
          encoder_Zero_Data = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
          encoder_Zero_IsInit = 2;
        }
        else if(encoder_Zero_IsInit==2)//��ֵ�Ѿ���¼�ɹ�
        {
          

          encoder_read = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
          data_temp = encoder_read -encoder_Zero_Data;
          temp = data_temp *3600 / ENCODER_RESOLUTION;
          Angle = temp;//�Ƕ�*10
          printf("Angle=%d\n",Angle/10);
          printf("Encoder_Process_Step=%d\n",Encoder_Process_Step);

          //�岹��������

          //����Ԥ����ֵ����
          if(Is_thres_stamp==1)  //����г���Ԥ��ֵ����ô�������� ���ڿ��Լ�����ģ��
          {
            if((Angle-Record_encoder) >= Compa_encoder_value)
            {
              HMI_value_setting("encode1.ad0.val",3);
              HAL_Delay(3000);
            }        
          }

           //���㰴ť�ж� �����쳣���� 
          if(Is_tare_stamp==1)   //��������㰴ť���� ��Ӧ��ť ��ͨ��ѡ�� ���� ��ʼ���� 
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

          //�Ƕȼ������
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
            case 2://����ʼ�Ƕ�
              HAL_Delay(100);
              second_count = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
              if(int_abs(second_count,encoder_read)<1000) //���ݲ����������
              {
                HAL_Delay(10);
                encoder_read = (OverflowCount*CNT_MAX) + __HAL_TIM_GET_COUNTER(&htimx_Encoder);
                third_count = encoder_read;
                data_temp = encoder_read -encoder_Zero_Data;
                temp = data_temp *3600 / ENCODER_RESOLUTION;
                Record_encoder1=temp; //ÿ�λ�Ƥ��ʱ��ֵ
                printf("Record_encoder1=%d ��\n",Record_encoder1/10); 
                Angle=temp;            
                //Is_tare_stamp=1;
                HMI_value_setting("encode1.gross.val",Angle); 
                Encoder_Process_Step=3;
              }
            break;
            case 3://��ʼ���� ����Ԥ���Ƕ�
              if(Is_start_stamp==1)   //��ʼ���԰�ť
              {
                Record_encoder=Record_encoder1;
                HMI_value_setting("encode1.gross.val",Record_encoder1); 
                printf("Record_encoder1=%d ��\n",Record_encoder1); 
                Encoder_Process_Step = 4;
                HMI_value_setting("encode1.ad0.val",0);
                HAL_Delay(4000);
              }
              else
              {
                Encoder_Process_Step = 2;
              }
            break;
            case 4://�ǶȲ��� ��ʼ�ж�
              if (Test_Step>=3)
              {
                Encoder_Process_Step = 6;
              }
              else if(Angle>Record_encoder)   //�Ƕȴ���Ԥ���Ƕ�
              {
                if(int_abs(Angle,Record_encoder)>1) //�Ƕȴ���1��
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
            case 5://������ �������
              if((timecount<=90) && (int_abs(Angle,Record_encoder)>1))//ʩ��������1�� �� δ������ʱ
              {
                timecount++;
                //�岹�������� 
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
                printf("in0=%d ��\n",in0/100); 

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
                printf("���� %d ����\n",Test_Step);
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
              printf("���β��Խ���\n");
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
        //����ѡ����� home
        case 0x01:
          printf("�ȳ��������Խ���\n"); 
          model_channelx=1;       
        break;
        case 0x02:
          printf("���Χ����ѡ�����\n"); 
          model_channelx=2;       
        break;
        
        //�ȳ��������Խ��� force0
        case 0x10:
          printf("����ѡ�����\n");
          model_channelx=0;        
        break;
        case 0x11:
          printf("ǰ��1ѡ��\n");
          force_channelx=1;
          weight_proportion = 2050; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSx_ENABLE();
          weight_ad7190_conf(force_channelx);       
        break;
        case 0x12:
          printf("����2ѡ��\n");
          force_channelx=2;
          weight_proportion = 1760;
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSx_ENABLE();  
          weight_ad7190_conf(force_channelx);      
        break;
        case 0x13:
          printf("�����3ѡ��\n");
          force_channelx=1; 
          weight_proportion = 1700; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSy_ENABLE(); 
          weight_ad7190_conf(force_channelx);     
        break;
        case 0x14:
          printf("�Ҳ���4ѡ��\n");
          force_channelx=2;
          weight_proportion = 1744; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSy_ENABLE();
          weight_ad7190_conf(force_channelx);       
        break;

        //���Χ��� encode0
        case 0x20:
          printf("����ѡ�����\n");
          model_channelx=0;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");   

        break;
        case 0x21:
          printf("ǰ��1ѡ��\n");
          encoder_channelx=1;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");

        break;
        case 0x22:
          printf("����2ѡ��\n");
          encoder_channelx=1;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");
           
        break;
        case 0x23:
          printf("�����3ѡ��\n");
          encoder_channelx=1;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");
               
        break;
        case 0x24:
          printf("�Ҳ���4ѡ��\n");
          encoder_channelx=1;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");
                
        break;
        case 0x25:
          printf("����5ѡ��\n");
          encoder_channelx=2;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");
             
        break;
        case 0x26:
          printf("����6ѡ��\n");
          encoder_channelx=2;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          ENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&htimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");
               
        break;

        //�������������� force1
        case 0x30:
          printf("�������Թ���ѡ�����");
          force_channelx = 0;
          weight_Zero_IsInit = 0;//ֹͣ��������
          Is_start_stamp = 0;
          Test_Step = 0;
          Force_Process_Step = 0;
        break;
        case 0x31:
          printf("�������� %d У׼\n",force_channelx);
        break;
        case 0x32:
          printf("����\n");
          Force_Process_Step=0;
          Is_tare_stamp=0;
          weight_Zero_IsInit=1;
          HMI_value_setting("force1.gross.val",0);
          HMI_value_setting("force1.net.val",0);        
        break;
        case 0x33:
          printf("��ʼ����\n");
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

        //���Χ������ encode1
        case 0x40:
          printf("���Χ��⹦��ѡ�����");
          encoder_channelx = 0;
          encoder_Zero_IsInit = 0;//ֹͣ��������
          Is_start_stamp = 0;
          Test_Step = 0;
          Encoder_Process_Step = 0;
        break;
        case 0x42:
          printf("����\n");
          Encoder_Process_Step=0;
          Is_tare_stamp=0;
          encoder_Zero_IsInit=1;
          HMI_value_setting("encode1.gross.val",0);
          HMI_value_setting("encode1.net.val",0);        
        break;
        case 0x43:
          printf("��ʼ����\n");
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

        //��������У׼����
        case 0x51:
          printf("��һ��\n");         
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
          printf("�ڶ���\n"); 
          printf("weight_proportion=%d\n",weight_proportion);         
        }
        break;
        case 0x53:
          printf("У׼���\n");
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
  * ��������: �������жϴ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * ��������: �򴮿�����������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * ��������: �򴮿������͸�������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
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

