/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���������ӳ�
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart/bsp_debug_usart.h"
#include "led/bsp_led.h"
#include "HMI/bsp_usartx_HMI.h"
#include "spiflash/bsp_spiweight.h"
#include "spiflash/bsp_spiflash.h"
#include "beep/bsp_beep.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define HMI_RX_BUFFER_SIZE       30

#define  FLASH_WriteAddress      0x0000
#define  FLASH_SectorToErase    FLASH_WriteAddress

#define HMI_SECTOR_ADDREE       4096*2

/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t  HMI_Rx_buf[HMI_RX_BUFFER_SIZE]={0};
__IO uint8_t  HMI_RX_flag=0;       //0:δ���յ�����ͷ  1���Ѿ����յ�����ͷ  2��һ֡�������
__IO uint8_t  usart_rx_flag;

__IO uint16_t timer_count=0;
__IO uint16_t pwm_data=0;

__IO uint8_t  weight_Zero_IsInit=0; //0: ��ֵδ��ȡ  1���ѻ�ȡ��ֵ
__IO int32_t  weight_Zero_Data=0;   // [0]:��������ֵ  [1]:����Ƥ������ֵ

__IO uint8_t Is_thres_stamp=0;  //�Ƿ��з�ֵ�趨
__IO uint8_t Is_tare_stamp=0;   //�Ƿ���Ƥ�ر�־
__IO int32_t Record_weight1;    //Ƥ�ؼ�¼ֵ������ÿ�γ���ǰ��¼Ƥ�أ� 
__IO int32_t Record_weight2;    //Ƥ�ؼ�¼ֵ������ÿ�γ���ǰ��¼Ƥ�أ� 
__IO int32_t cali_weight;       //У׼ʹ��
__IO int32_t weight_Zero_Data;  //Ƥ�س�ʼֵ
__IO int32_t weight_proportion; //����ֵ��¼
__IO int32_t weight_current;    //Ƥ�ؼ�¼ֵ
__IO int32_t second_count;      
__IO int32_t third_count;
__IO uint8_t Process_Step=0;

uint32_t DeviceID = 0;
uint32_t FlashID = 0;

uint8_t Tx_Buffer[3] = {0};
uint8_t Rx_Buffer[3] = {0};

int32_t Result_data=0;

/* ��չ���� ------------------------------------------------------------------*/


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
  static int16_t timecount=0;  
  int32_t weight_first;
  uint32_t tmp[2];
  int32_t Compa_value;
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  HMI_USARTx_Init();
  /* ��ʼ��LED */
  LED_GPIO_Init(); 
  
  __HAL_UART_ENABLE_IT(&husartx_HMI, UART_IT_RXNE);
  /* ��ʼ��SPI */
  MX_SPIFlash_Init();
  /* ��ʼ��BEEP */
  BEEP_GPIO_Init();
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
      weight_Zero_IsInit=1;
    }      
  }  	  
  if(AD7190_Init())
  {
    printf("��⵽  AD7190 !\n");
    weight_ad7190_conf();
  }
  /* ����ѭ�� */
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
        if(Is_thres_stamp==1)  //����г���Ԥ��ֵ����ô��������
        {
          if((weight_current-Record_weight1)>=Compa_value)
          {
            BEEP_ON;
            HAL_Delay(200);
            BEEP_OFF;
          }        
        }        
        if(Is_tare_stamp==1)   //��������㰴ť����
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
        //���г��أ������ǣ���һ���������Ʒ���ᱻ����Ƥ�أ��ڶ��η���Ĳ���ʵ������
        //��������������������Ҫ�������ܣ������ڴ˻����Ͻ����޸�
        switch(Process_Step)  
        {
          case 0:  //��⵽����Ʒ
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
          case 1:  //��⵽����Ʒ
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
          case 2:  //����Ƥ��
            HAL_Delay(100);
            second_count=weight_ad7190_ReadAvg(3);
            if(int_abs(second_count,weight_count)<10)
            {
              HAL_Delay(10);
              weight_count=weight_ad7190_ReadAvg(3);
              third_count=weight_count;
              data_temp=weight_count-weight_Zero_Data;
              temp=data_temp*100000/weight_proportion;
              Record_weight1=temp; //ÿ�λ�Ƥ��ʱ��ֵ
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
          case 3:  //Ƥ�صĻ����ϼ���Ʒ
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
          case 4:  //����������Ʒ������
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
          printf("���ӳ�У׼\n");
        break;
        case 0x02:
          printf("��һ��\n");         
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
          printf("�ڶ���\n");         
        }
        break;
        case 0x04:
          printf("У׼���\n");
          Process_Step=0;
          weight_Zero_IsInit=1;   
          HMI_value_setting("page0.gross.val",0);
          HMI_value_setting("page0.net.val",0);           
        break;
        case 0x05:
          printf("����\n");
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




/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

