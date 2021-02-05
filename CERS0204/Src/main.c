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
#include "math.h"
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

#include "usbh_core.h"
#include "usbh_msc.h" 
#include "ff.h"
#include "ff_gen_drv.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_READY,    
  APPLICATION_DISCONNECT,
}MSC_ApplicationTypeDef;

/* ˽�к궨�� ----------------------------------------------------------------*/
#define HMI_RX_BUFFER_SIZE       30

#define  FLASH_WriteAddress      0x0000
#define  FLASH_SectorToErase    FLASH_WriteAddress

#define HMI_SECTOR_ADDREE       4096*2

/* ˽�б��� ------------------------------------------------------------------*/
/* USB���� ------------------------------------------------------------------*/
USBH_HandleTypeDef hUSBHost;
MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;

char USBDISKPath[4];             /* ����Flash�߼��豸·�� */
FATFS fs;													/* FatFs�ļ�ϵͳ���� */
FIL file;													/* �ļ����� */
FRESULT f_res;                    /* �ļ�������� */
UINT fnum;            					  /* �ļ��ɹ���д���� */

char scan_path[255] = "0:";      /* �ݹ�ɨ���ļ�ʱʹ�õ�·�� */

BYTE ReadBuffer[1024]={0};       /* �������� */
BYTE WriteBuffer[]= "��U�����½��ļ�ϵͳ�����ļ�\n";/* д������*/  
BYTE WriteBuffer_test[1024];
BYTE WriteBuffer_name[100];
BYTE WriteBuffer_number[100];
BYTE WriteBuffer_high[100];
BYTE WriteBuffer_weight[100];
BYTE WriteBuffer_age[100];
BYTE WriteBuffer_male[100];
BYTE WriteBuffer_sick[100];
BYTE WriteBuffer_title[1024];
BYTE WriteBuffer_date[1024];
BYTE WriteBuffer_dir[100];
BYTE WriteBuffer_csv[100];

__IO uint8_t key1_temp=0;
/* HMI���ڱ��� ------------------------------------------------------------------*/
__IO uint8_t  HMI_Rx_buf[HMI_RX_BUFFER_SIZE]={0};
__IO uint8_t  HMI_RX_flag=0;       //0:δ���յ�����ͷ  1���Ѿ����յ�����ͷ  2��һ֡�������
__IO uint8_t  usart_rx_flag;
__IO uint16_t HUM_Rx_count=0;
__IO char     HMI_str[HMI_RX_BUFFER_SIZE + 1];
__IO char     HMI_dir[100];

__IO char     receiveBuffer[HMI_RX_BUFFER_SIZE];
__IO uint16_t receiveLength;
__IO uint16_t receiveLength_str;

__IO uint16_t timer_count=0;
__IO uint16_t pwm_data=0;
/* ͨ��ѡ����� ------------------------------------------------------------------*/
__IO uint8_t  model_channelx=0;         //ģʽ����ѡ��
__IO uint8_t  force_channelx=0;         //��������ѡ��
__IO uint8_t  encoder_channelx=0;        //������ѡ��
__IO uint8_t  force_modelx=0;         //����ѡ��
__IO uint8_t  encoder_modelx=0;        //����ѡ��
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

/* �岹�������� ------------------------------------------------------------------*/
__IO int32_t  average=0;
__IO int32_t  maximum=0;
__IO int32_t  standard_deviation=0;//��׼��
__IO int16_t  coefficient_variation=0;
__IO int32_t  maximum_record1=0;
__IO int32_t  maximum_record2=0;
__IO int32_t  maximum_record3=0;
__IO int16_t  record_count=0;

/* �Ƕȼ����� ------------------------------------------------------------------*/
// __IO int32_t CaptureNumber = 0;     // ���벶����
// __IO int32_t LastCapNum = 0;     // ��һ�����벶����
// __IO int32_t Speed = 0;     // ��һ�����벶����
__IO int32_t Angle = 0;      // �Ƕ�
__IO int32_t ENCODER_RESOLUTION = 0;

__IO uint8_t  encoder_Zero_IsInit=0;     // 0��ֹͣ���ݲɼ� 1:��ֵδ��ȡ  2���ѻ�ȡ��ֵ
__IO int32_t  encoder_Zero_Data=0;       // ��ʩ����ʱ��ֵ��¼ֵ
__IO uint8_t  Encoder_Process_Step=0;   // 
__IO int32_t  Record_encoder = 0;        //Ԥ���Ƕ�ֵ
__IO int32_t  Record_encoder1;           //Ԥ���Ƕȼ�¼ֵ������ÿ�β�Ƕ�ǰ��¼��ֵ�� 
__IO int32_t  Record_encoder2;           //Ԥ���Ƕȼ�¼ֵ������ÿ�β�Ƕ�ǰ��¼��ֵ ��
__IO int64_t  data_temp,temp;
__IO int32_t  encoder_read;
__IO int32_t  encoder_read_before;
/* SPI flash���� ------------------------------------------------------------------*/
uint32_t DeviceID = 0;
uint32_t FlashID = 0;
uint8_t Tx_Buffer[3] = {0};
uint8_t Rx_Buffer[3] = {0};
int32_t Result_data=0;


__IO int32_t timecount = 0;
/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint32_t uwTick;  //ʱ�Ӽ���
extern int32_t XOverflowCount ;//��ʱ��������� 
extern int32_t YOverflowCount ;//��ʱ��������� 

extern Diskio_drvTypeDef  USBH_Driver;

/* ˽�к���ԭ�� --------------------------------------------------------------*/
void HMI_value_setting(const char *val_str,uint32_t value);
void HMI_string_setting(const char *val_str,int32_t value);
unsigned int ENCODER_read_channelx(int8_t channelx);
char* HMI_USARTx_GetString(void);

uint8_t exf_getfree(uint8_t *drv,uint32_t *total,uint32_t *free);
static void read_write_flie(void);
unsigned int write_file(int32_t date);
static FRESULT scan_files (char* path) ;
static void printf_fatfs_error(FRESULT fresult);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);


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
  int8_t Pain_value=0;
  int8_t Pain_stamp= 0;
  int8_t  Compa_encoder_value;//��ֵԤ��ֵ
  char* HMI_string_temp=NULL;

  uint32_t total,free;
	uint8_t res=0,counts=0;

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
  KEY_GPIO_Init();

  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
  {
    /* ��ʼ���û����� */
    if(USBH_Init(&hUSBHost, USBH_UserProcess, HOST_FS)==USBH_OK)
    {
      printf("USB INIT  OK!\n");
    }
    
    /* ���ϵͳMSC������ */
    USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);
    
    /* ������������ */
    USBH_Start(&hUSBHost);      
  }

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
    USBH_Process(&hUSBHost);

    /* ����key1��ɨ��U���ڵ��ļ� */
    if(KEY1_StateRead()==KEY_DOWN)
    {
      key1_temp=1;
    }
    /* ����key2����ѯU��������ʣ������ */ 
    if(KEY2_StateRead()==KEY_DOWN)
    {
      key1_temp=2;
    }  
    switch (key1_temp)
    {
      case 1:/* ����key1��ɨ��U���ڵ��ļ� */
        LED1_ON;
        f_res = f_mount(&fs,"0:",1);	/* �ڴ���FLASH�����ļ�ϵͳ���ļ�ϵͳ����ʱ��Դ���FLASH��ʼ�� */
        printf_fatfs_error(f_res);
        if(f_res != FR_OK)
        {
          printf("\r\n�ļ�ϵͳ����ʧ�ܣ�\r\n");
          continue; 
        }
        printf("\r\nɨ�赽��U���ļ���\r\n");
        scan_files(scan_path); 
        key1_temp=0;
      break;
      case 2:/* ����key2����ѯU��������ʣ������ */
        res=exf_getfree("0:",&total,&free);
        if(res==0)
        {
          printf("total=%dMB free=%dMB\n",total>>10,free>>10);
          key1_temp=0;
        }
      break;  
    } 

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
                f_res = f_mount(&fs,"0:",1);
                printf_fatfs_error(f_res);
                f_res = f_open(&file, "�����ļ�1.txt",FA_CREATE_ALWAYS | FA_WRITE );
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
                write_file(in0/1000);

                Force_Process_Step = 5;
                HMI_value_setting("force1.net.val",in0/1000);//0.1N
                if(in0/1000>maximum)//0.1N
                {
                  maximum=in0/1000;
                }

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
                switch(Test_Step)
                {
                  case 1: 
                    HMI_value_setting("force1.max1.val",maximum);//0.1N
                    maximum_record1=maximum;
                    maximum =0;
                  break;
                  case 2:
                    HMI_value_setting("force1.max2.val",maximum);
                    maximum_record2=maximum;
                    maximum =0;
                  break;
                  case 3:
                    HMI_value_setting("force1.max3.val",maximum);
                    maximum_record3=maximum;
                    maximum =0;
                    average = (maximum_record1+maximum_record2+maximum_record3)/3;
                    printf("average=%d N\n",average); //0.1N
                    
                    
                    standard_deviation=sqrt((pow(maximum_record1-average,2)+pow(maximum_record2-average,2)+pow(maximum_record3-average,2))/3);
                    // printf("standard_deviation=%d N\n",standard_deviation);
                    coefficient_variation = standard_deviation*100/average;
                    printf("coefficient_variation=%d \n",coefficient_variation);

                  break;
                }
                HAL_Delay(5000);
              }
            break;
            case 6:  
              if(Pain_stamp==1)
              {
                switch(force_modelx)
                {
                  case 1://ǰ��
                    HMI_value_setting("report5.x0.val",average);//ƽ��ֵ
                    HMI_value_setting("report5.x1.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report5.x2.val",average*100/113);//����ֵ��
                    HMI_value_setting("report5.x3.val",Pain_value*10);//��ʹֵ
                    if(average<3000)//300
                    {
                      HMI_value_setting("report5.j1.val",average/30);
                    }
                    else
                    {
                      HMI_value_setting("report5.j1.val",100);
                    } 
                    if(Pain_value>0)
                    {
                      HMI_value_setting("report0.va0.val",1);
                    }
                  break;
                  case 2://����
                    HMI_value_setting("report5.x5.val",average);//ƽ��ֵ
                    HMI_value_setting("report5.x6.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report5.x7.val",average*100/158);//����ֵ��
                    HMI_value_setting("report5.x8.val",Pain_value*10);//��ʹֵ
                    if(average<3000)//300
                    {
                      HMI_value_setting("report5.j0.val",average/30);
                    }
                    else
                    {
                      HMI_value_setting("report5.j0.val",100);
                    } 
                    if(Pain_value>0)
                    {
                      HMI_value_setting("report0.va1.val",1);
                    }
                  break;
                  case 3://�����
                    HMI_value_setting("report4.x0.val",average);//ƽ��ֵ
                    HMI_value_setting("report4.x1.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report4.x2.val",average*100/113);//����ֵ��
                    HMI_value_setting("report4.x3.val",Pain_value*10);//��ʹֵ
                    if(average<2000)//200
                    {
                      HMI_value_setting("report4.j1.val",average/20);
                    }
                    else
                    {
                      HMI_value_setting("report4.j1.val",100);
                    } 
                    if(Pain_value>0)
                    {
                      HMI_value_setting("report0.va2.val",1);
                    }
                  break;
                  case 4://�Ҳ���
                    HMI_value_setting("report4.x5.val",average);//ƽ��ֵ
                    HMI_value_setting("report4.x6.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report4.x7.val",average*100/113);//����ֵ��
                    HMI_value_setting("report4.x8.val",Pain_value*10);//��ʹֵ
                    if(average<2000)//200
                    {
                      HMI_value_setting("report4.j0.val",average/20);
                    }
                    else
                    {
                      HMI_value_setting("report4.j0.val",100);
                    } 
                    if(Pain_value>0)
                    {
                      HMI_value_setting("report0.va2.val",1);
                    }
                  break;
                }
                // WriteBuffer_test[]="���ֵ1=";
                write_file(maximum_record1);
                
                // WriteBuffer_test[]="���ֵ2=";
                write_file(maximum_record2);
                
                // WriteBuffer_test[]="���ֵ3=";
                write_file(maximum_record3);
                // WriteBuffer_test[]="ƽ��ֵ=";
                write_file(average);

                // WriteBuffer_test[]="����ϵ��=";
                write_file(coefficient_variation);
                
                write_file(Pain_value);

                f_close(&file);


                Test_Step = 0;
                Force_Process_Step = 0;
                Is_start_stamp = 0;
                weight_Zero_IsInit=0;
                Pain_stamp = 0;
                HMI_value_setting("force1.ad0.val",2);//����
                HAL_Delay(3000);

                HMI_value_setting("force1.gross.val",0);
                HMI_value_setting("force1.net.val",0);
                printf("���β��Խ���\n");
              }
              else
              {
                Force_Process_Step=6;
              }
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
          encoder_Zero_Data = ENCODER_read_channelx(encoder_channelx);
          encoder_Zero_IsInit = 2;
        }
        else if(encoder_Zero_IsInit==2)//��ֵ�Ѿ���¼�ɹ�
        {
          // encoder_read_before=ENCODER_read_channelx(encoder_channelx);
          // encoder_read = encoder_read_before%65535;
          encoder_read=ENCODER_read_channelx(encoder_channelx);
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
              second_count = ENCODER_read_channelx(encoder_channelx);
              if(int_abs(second_count,encoder_read)<1000) //���ݲ����������
								
              {
                HAL_Delay(10);
                encoder_read = ENCODER_read_channelx(encoder_channelx);
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
                f_res = f_mount(&fs,"0:",1);
                printf_fatfs_error(f_res);
                f_res = f_open(&file, "�����ļ�2.txt",FA_CREATE_ALWAYS | FA_WRITE );
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
                if(int_abs(Angle,Record_encoder)>10) //�Ƕȴ���1��
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
            case 5://�ǶȲ��� �������
              if((timecount<=90) && (int_abs(Angle,Record_encoder)>10) && (int_abs(Angle,Record_encoder))<1000)//ʩ�Ӵ���1�� �� δ������ʱ
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
                printf("in0=%d ��\n",in0/10); 
                write_file(in0/10);


                Encoder_Process_Step = 5;
                HMI_value_setting("encode1.net.val",in0);//0.1��
                if(in0>maximum)//0.1��
                {
                  maximum=in0;
                }
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
                switch(Test_Step)
                {
                  case 1: 
                    HMI_value_setting("encode1.max1.val",maximum);
                    maximum_record1=maximum;
                    maximum =0;
                  break;
                  case 2:
                    HMI_value_setting("encode1.max2.val",maximum);
                    maximum_record2=maximum;
                    maximum =0;
                  break;
                  case 3:
                    HMI_value_setting("encode1.max3.val",maximum);
                    maximum_record3=maximum;
                    maximum =0;
                    average = (maximum_record1+maximum_record2+maximum_record3)/3;
                    printf("average=%d ��\n",in0/10); 
                    standard_deviation=sqrt((pow(maximum_record1-average,2)+pow(maximum_record2-average,2)+pow(maximum_record3-average,2))/3);
                    // printf("standard_deviation=%d N\n",standard_deviation);
                    coefficient_variation = standard_deviation*100/average;
                    printf("coefficient_variation=%d \n",coefficient_variation);
                  break;
                }
                HAL_Delay(5000);
              }
            break;
            case 6:
              if(Pain_stamp==1)
              {
                switch(encoder_modelx)
                {
                  case 1://ǰ��
                    HMI_value_setting("report2.x0.val",average);//ƽ��ֵ
                    HMI_value_setting("report2.x1.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report2.x2.val",average*100/60);//����ֵ��
                    HMI_value_setting("report2.x3.val",Pain_value*10);//��ʹֵ
                    if(average<900)//90
                    {
                      HMI_value_setting("report2.j1.val",average/9);
                    }
                    else
                    {
                      HMI_value_setting("report2.j1.val",100);
                    } 
                  break;
                  case 2://����
                    HMI_value_setting("report2.x5.val",average);//ƽ��ֵ
                    HMI_value_setting("report2.x6.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report2.x7.val",average*100/60);//����ֵ��
                    HMI_value_setting("report2.x8.val",Pain_value*10);//��ʹֵ
                    if(average<900)//90
                    {
                      HMI_value_setting("report2.j0.val",average/9);
                    }
                    else
                    {
                      HMI_value_setting("report2.j0.val",100);
                    } 
                  break;
                  case 3://�����
                    HMI_value_setting("report1.x0.val",average);//ƽ��ֵ
                    HMI_value_setting("report1.x1.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report1.x2.val",average*100/45);//����ֵ��
                    HMI_value_setting("report1.x3.val",Pain_value*10);//��ʹֵ
                    if(average<900)//90
                    {
                      HMI_value_setting("report1.j1.val",average/9);
                    }
                    else
                    {
                      HMI_value_setting("report1.j1.val",100);
                    } 
                  break;
                  case 4://�Ҳ���
                    HMI_value_setting("report1.x5.val",average);//ƽ��ֵ
                    HMI_value_setting("report1.x6.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report1.x7.val",average*100/45);//����ֵ��
                    HMI_value_setting("report1.x8.val",Pain_value*10);//��ʹֵ
                    if(average<900)//90
                    {
                      HMI_value_setting("report1.j0.val",average/9);
                    }
                    else
                    {
                      HMI_value_setting("report1.j0.val",100);
                    } 
                  break;
                  case 5://����
                    HMI_value_setting("report3.x0.val",average);//ƽ��ֵ
                    HMI_value_setting("report3.x1.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report3.x2.val",average*100/80);//����ֵ��
                    HMI_value_setting("report3.x3.val",Pain_value*10);//��ʹֵ
                    if(average<1000)//100
                    {
                      HMI_value_setting("report3.j1.val",average/10);
                    }
                    else
                    {
                      HMI_value_setting("report3.j1.val",100);
                    } 
                  break;
                  case 6://����
                    HMI_value_setting("report3.x5.val",average);//ƽ��ֵ
                    HMI_value_setting("report3.x6.val",coefficient_variation*10);//����ϵ��
                    HMI_value_setting("report3.x7.val",average*100/80);//����ֵ��
                    HMI_value_setting("report3.x8.val",Pain_value*10);//��ʹֵ
                    if(average<1000)//100
                    {
                      HMI_value_setting("report3.j0.val",average/10);
                    }
                    else
                    {
                      HMI_value_setting("report3.j0.val",100);
                    }
                  break;
                }
                // WriteBuffer_test[]="���ֵ1=";
                write_file(maximum_record1);
                
                // WriteBuffer_test[]="���ֵ2=";
                write_file(maximum_record2);
                
                // WriteBuffer_test[]="���ֵ3=";
                write_file(maximum_record3);
                // WriteBuffer_test[]="ƽ��ֵ=";
                write_file(average);

                // WriteBuffer_test[]="����ϵ��=";
                write_file(coefficient_variation);
                
                write_file(Pain_value);
                
                f_close(&file);
                
                Test_Step = 0;
                Encoder_Process_Step = 0;
                Is_start_stamp = 0;
                encoder_Zero_IsInit=0;
                Pain_stamp =0;
                HMI_value_setting("encode1.ad0.val",2);
                HAL_Delay(3000);

                HMI_value_setting("encode1.gross.val",0);
                HMI_value_setting("encode1.net.val",0);
                printf("���β��Խ���\n");
              }
              else
              {
                Encoder_Process_Step=6;
              }
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
          force_modelx=1;
          weight_proportion = 2050; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSx_ENABLE();
          weight_ad7190_conf(force_channelx); 
          HMI_value_setting("force1.file.val",0);      
        break;
        case 0x12:
          printf("����2ѡ��\n");
          force_channelx=2;
          force_modelx=2;
          weight_proportion = 1760;
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSx_ENABLE();  
          weight_ad7190_conf(force_channelx);
          HMI_value_setting("force1.file.val",0);      
        break;
        case 0x13:
          printf("�����3ѡ��\n");
          force_channelx=1; 
          force_modelx=3;
          weight_proportion = 1700; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSy_ENABLE(); 
          weight_ad7190_conf(force_channelx); 
          HMI_value_setting("force1.file.val",0);    
        break;
        case 0x14:
          printf("�Ҳ���4ѡ��\n");
          force_channelx=2;
          force_modelx=4;
          weight_proportion = 1744; 
          WEIGHT_CSx_DISABLE();
          WEIGHT_CSy_DISABLE();
          WEIGHT_CSy_ENABLE();
          weight_ad7190_conf(force_channelx);
          HMI_value_setting("force1.file.val",0);       
        break;

        //���Χ��� encode0
        case 0x20:
          printf("����ѡ�����\n");
          model_channelx=0;
          encoder_channelx=0;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          printf("--> �������ӿ�4��Ƶ,���±��ض�����<-- \n");   

        break;
        case 0x21:
          printf("ǰ��1ѡ��\n");
          encoder_channelx=1;
          encoder_modelx=1;
          ENCODER_RESOLUTION = XENCODER_RESOLUTION;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          YENCODER_TIM_RCC_CLK_DISABLE();
          XENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&xhtimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�X<-- \n");
          HMI_value_setting("encode1.file.val",0); 

        break;
        case 0x22:
          printf("����2ѡ��\n");
          encoder_channelx=1;
          encoder_modelx=2;
          ENCODER_RESOLUTION = XENCODER_RESOLUTION;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          YENCODER_TIM_RCC_CLK_DISABLE();
          XENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&xhtimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�X<-- \n");
          HMI_value_setting("encode1.file.val",0); 
           
        break;
        case 0x23:
          printf("�����3ѡ��\n");
          encoder_channelx=1;
          encoder_modelx=3;
          ENCODER_RESOLUTION = XENCODER_RESOLUTION;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          YENCODER_TIM_RCC_CLK_DISABLE();
          XENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&xhtimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�X<-- \n");
          HMI_value_setting("encode1.file.val",0); 
               
        break;
        case 0x24:
          printf("�Ҳ���4ѡ��\n");
          encoder_channelx=1;
          encoder_modelx=4;
          ENCODER_RESOLUTION = XENCODER_RESOLUTION;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          YENCODER_TIM_RCC_CLK_DISABLE();
          XENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&xhtimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�X<-- \n");
          HMI_value_setting("encode1.file.val",0); 
                
        break;
        case 0x25:
          printf("����5ѡ��\n");
          encoder_channelx=2;
          encoder_modelx=5;
          ENCODER_RESOLUTION = YENCODER_RESOLUTION;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          XENCODER_TIM_RCC_CLK_DISABLE();
          YENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&yhtimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�Y<-- \n");
          HMI_value_setting("encode1.file.val",0); 
             
        break;
        case 0x26:
          printf("����6ѡ��\n");
          encoder_channelx=2;
          encoder_modelx=6;
          ENCODER_RESOLUTION = YENCODER_RESOLUTION;
          /* ��������ʼ����ʹ�ܱ�����ģʽ */
          XENCODER_TIM_RCC_CLK_DISABLE();
          YENCODER_TIMx_Init();
          HAL_TIM_Encoder_Start(&yhtimx_Encoder, TIM_CHANNEL_ALL);
          printf("--> �������ӿ�Y<-- \n");
          HMI_value_setting("encode1.file.val",0); 
               
        break;

        //�������������� force1
        case 0x30:
          printf("�������Թ���ѡ�����");
          force_channelx = 0;
          force_modelx=0;
          weight_Zero_IsInit = 0;//ֹͣ��������
          Is_start_stamp = 0;
          Test_Step = 0;
          Force_Process_Step = 0;      
          HMI_value_setting("force1.max1.val",0);
          HMI_value_setting("force1.max2.val",0);
          HMI_value_setting("force1.max3.val",0);
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
          HMI_value_setting("force1.max1.val",0);
          HMI_value_setting("force1.max2.val",0);
          HMI_value_setting("force1.max3.val",0);       
        break;
        case 0x33:
          printf("��ʼ����\n");
          Is_start_stamp = 1;
          Force_Process_Step = 3;       
        break;
        case 0x34: //��ȫֵ����        
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2]; 
          Compa_value=tmp[0]*1000;   //0.1N 
          Is_thres_stamp=1; 
          if(Compa_value==0)
          {
            Is_thres_stamp=0;
          }            
          printf("Compa_value*10=%d\n N",Compa_value/1000);
        break;
        case 0x35: //��ʹָ������        
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2]; 
          Pain_value=tmp[0];
          Pain_stamp=1;
          printf("Pain_value=%d\n",Pain_value);
        break;

        //���Χ������ encode1
        case 0x40:
          printf("���Χ��⹦��ѡ�����");
          encoder_channelx = 0;
          encoder_modelx=0;
          encoder_Zero_IsInit = 0;//ֹͣ��������
          Is_start_stamp = 0;
          Test_Step = 0;
          Encoder_Process_Step = 0;     
          HMI_value_setting("encode1.max1.val",0);
          HMI_value_setting("encode1.max2.val",0);
          HMI_value_setting("encode1.max3.val",0);
        break;
        case 0x42:
          printf("����\n");
          Encoder_Process_Step=0;
          Is_tare_stamp=0;
          encoder_Zero_IsInit=1;
          HMI_value_setting("encode1.gross.val",0);
          HMI_value_setting("encode1.net.val",0);     
          HMI_value_setting("encode1.max1.val",0);
          HMI_value_setting("encode1.max2.val",0);
          HMI_value_setting("encode1.max3.val",0);
        break;
        case 0x43:
          printf("��ʼ����\n");
          Is_start_stamp = 1;
          Encoder_Process_Step = 3;       
        break;
        case 0x44: //��ȫ������        
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2]; 
          Compa_encoder_value=tmp[0]*10;   
          Is_thres_stamp=1; 
          if(Compa_encoder_value==0)
          {
            Is_thres_stamp=0;
          }            
          printf("Compa_encoder_value*10=%d\n N",Compa_encoder_value);
        break;
        case 0x45: //��ʹָ������        
          tmp[0]=(HMI_Rx_buf[4]<<16)|(HMI_Rx_buf[3]<<8)|HMI_Rx_buf[2]; 
          Pain_value=tmp[0];
          Pain_stamp=1;
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

        case 0x60:
          HMI_USARTx_GetString();
          printf("������ %s\n",HMI_str);
          sprintf(WriteBuffer_dir,"%s",HMI_str);
          sprintf(WriteBuffer_name,"%s",HMI_str);
        break;
        case 0x61: 
          HMI_USARTx_GetString();
          printf("ʵ������ %s\n",HMI_str);
          sprintf(WriteBuffer_dir,"%s_%s",WriteBuffer_dir,HMI_str);
          sprintf(WriteBuffer_number,"%s",HMI_str);
        break;
        case 0x62: 
          HMI_USARTx_GetString();
          printf("����� %s\n",HMI_str);
          sprintf(WriteBuffer_high,"%s",HMI_str);
        break;
        case 0x63: 
          HMI_USARTx_GetString();
          printf("������ %s\n",HMI_str);
          sprintf(WriteBuffer_weight,"%s",HMI_str);
        break;
        case 0x64: 
          HMI_USARTx_GetString();
          printf("������ %s\n",HMI_str);
          sprintf(WriteBuffer_age,"%s",HMI_str);
        break;
        case 0x65: 
          HMI_USARTx_GetString();
          printf("�Ա��� %s\n",HMI_str);
          sprintf(WriteBuffer_male,"%s",HMI_str);
        break;
        case 0x66: 
          HMI_USARTx_GetString();
          printf("���޻���ʷ�� %s\n",HMI_str);
          sprintf(WriteBuffer_sick,"%s",HMI_str);
        break;
        case 0x69:
          printf("�������� %s\n",WriteBuffer_dir);
          f_res = f_mount(&fs,"0:",1);	/* �ڴ���FLASH�����ļ�ϵͳ���ļ�ϵͳ����ʱ��Դ���FLASH��ʼ�� */
          printf_fatfs_error(f_res);

          f_res=f_mkdir(WriteBuffer_dir);
          printf_fatfs_error(f_res);

          sprintf( WriteBuffer_csv,"/%s/%s.csv",WriteBuffer_dir,"�û�������Ϣ");
          f_res =f_open(&file, WriteBuffer_csv,FA_CREATE_ALWAYS | FA_WRITE );
          printf_fatfs_error(f_res);

          sprintf(WriteBuffer_title, "%s\r\n", "����,ʵ����,���,����,����,�Ա�,���޻���ʷ");
          f_write(&file,WriteBuffer_title,sizeof(WriteBuffer_title),&fnum);
          f_sync(&file);
          printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
          printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer_title);

          sprintf(WriteBuffer_test, "%s,%s,%s,%s,%s,%s,%s\r\n", WriteBuffer_name,WriteBuffer_number,WriteBuffer_high,WriteBuffer_weight,WriteBuffer_age,WriteBuffer_male,WriteBuffer_sick);
          f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
          f_sync(&file);
          printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
          printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer_test);
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
        receiveLength=0;       
      }
      if((HMI_RX_flag!=2)&&(HUM_Rx_count>2)&&(tmp!=0xFF))
      {
        receiveBuffer[receiveLength++] = tmp;
        receiveLength_str=receiveLength;
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
  * ��������: ���մ������ַ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
char* HMI_USARTx_GetString(void)
{
	uint16_t HMI_temp;

	for(HMI_temp = 0; HMI_temp < receiveLength_str; HMI_temp++)
	{
		HMI_str[HMI_temp] = receiveBuffer[HMI_temp];
	}

  receiveLength_str=0;
	// HMI_str[HMI_temp] = '\0';
	return HMI_str;
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

/**
  * ��������: ����������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
unsigned int ENCODER_read_channelx(int8_t channelx)
{
  int32_t  encoder_read_channelx=0;
  int32_t  encoder_read_motion=0;
  switch(encoder_channelx)
  {
    case 0:
      encoder_Zero_IsInit = 1;
    break;
    case 1:
      encoder_read_channelx = (XOverflowCount*XCNT_MAX) + __HAL_TIM_GET_COUNTER(&xhtimx_Encoder);
    break;
    case 2:
      encoder_read_channelx = (YOverflowCount*YCNT_MAX) + __HAL_TIM_GET_COUNTER(&yhtimx_Encoder);
    break;
  }
  if(encoder_modelx==2||encoder_modelx==3||encoder_modelx==5)
  {
    encoder_read_motion=(-1)*encoder_read_channelx;
  }
  else
  {
    encoder_read_motion=encoder_read_channelx;
  }
  return encoder_read_motion;
}

/**
  * ��������: �õ����������Լ�ʣ������
  * �������: drv:���̱��"0:" total:������ ��λKB  free:ʣ������ ��λKB
  * �� �� ֵ: ����ֵ:0,����.����,�������
  * ˵    ��: ��
  */
uint8_t exf_getfree(uint8_t *drv,uint32_t *total,uint32_t *free) 
{
	FATFS *fs1;
	uint8_t res;
  uint32_t fre_clust=0, fre_sect=0, tot_sect=0;
  /* �õ�������Ϣ�����д����� */
  res =(uint32_t)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
  if(res==0)
	{											   
	  tot_sect=(fs1->n_fatent-2)*fs1->csize;	/* �õ��������� */
	  fre_sect=fre_clust*fs1->csize;			/* �õ�����������	*/   
#if _MAX_SS!=512				  				/* ������С����512�ֽ�,��ת��Ϊ512�ֽ� */
		tot_sect*=fs1->ssize/512;
		fre_sect*=fs1->ssize/512;
#endif	  
		*total=tot_sect>>1;	/* ��λΪKB */
		*free=fre_sect>>1;	/* ��λΪKB */ 
 	}
	return res;
}		

/**
  * ��������: �ݹ�ɨ��FatFs�ڵ��ļ�
  * �������: ��ʼɨ��·��
  * �� �� ֵ: res:�ļ�ϵͳ�ķ���ֵ
  * ˵    ��: ��
  */
static FRESULT scan_files (char* path) 
{ 
  FRESULT res; 		/* �����ڵݹ���̱��޸ĵı���������ȫ�ֱ��� */	
  FILINFO fno; 
  DIR dir; 
  int i;            
  char *fn;       /* �ļ��� */
	
#if _USE_LFN 
  /* ���ļ���֧�� */
  /* ����������Ҫ2���ֽڱ���һ�����֡�*/
  static char lfn[_MAX_LFN*2 + 1]; 	
  fno.lfname = lfn; 
  fno.lfsize = sizeof(lfn); 
#endif 
  /* ��Ŀ¼ */
  res = f_opendir(&dir, path); 
  if (res == FR_OK) 
	{ 
    i = strlen(path); 
    for (;;) 
		{ 
      /* ��ȡĿ¼�µ����ݣ��ٶ����Զ�����һ���ļ� */
      res = f_readdir(&dir, &fno); 								
      /* Ϊ��ʱ��ʾ������Ŀ��ȡ��ϣ����� */
      if (res != FR_OK || fno.fname[0] == 0) break; 	
#if _USE_LFN 
      fn = *fno.lfname ? fno.lfname : fno.fname; 
#else 
      fn = fno.fname; 
#endif 
      /* ���ʾ��ǰĿ¼������ */			
      if (*fn == '.') continue; 	
      /* Ŀ¼���ݹ��ȡ */      
      if (fno.fattrib & AM_DIR)         
			{ 			
        /* �ϳ�����Ŀ¼�� */        
        sprintf(&path[i], "/%s", fn); 		
        /* �ݹ���� */         
        res = scan_files(path);	
        path[i] = 0;         
        /* ��ʧ�ܣ�����ѭ�� */        
        if (res != FR_OK) 
	break; 
      } 
      else 
      { 
              printf("%s/%s\r\n", path, fn);								/* ����ļ���	*/
        /* ������������ȡ�ض���ʽ���ļ�·�� */        
      }//else
    } //for
  } 
  return res; 
}


/**
  * ��������: �ļ�ϵͳд�ļ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
unsigned int write_file(int32_t date)
{
  sprintf(WriteBuffer_test, "%d", date);
  f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
  f_sync(&file);
  sprintf(WriteBuffer_test, "%d", 0x0000);
  
}

/**
  * ��������: �û�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
  switch(id)
  { 
    case HOST_USER_SELECT_CONFIGURATION:
      break;
      
    case HOST_USER_DISCONNECTION:
      Appli_state = APPLICATION_DISCONNECT;
      
      break;
      
    case HOST_USER_CLASS_ACTIVE:
      Appli_state = APPLICATION_READY;
      break;
      
    case HOST_USER_CONNECTION:
      break;

    default:
      break; 
  }
}


/**
  * ��������: FatFS�ļ�ϵͳ���������Ϣ����.
  * �������: FatFS�ļ�ϵͳ���������FRESULT
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void printf_fatfs_error(FRESULT fresult)
{
  switch(fresult)
  {
    case FR_OK:                   //(0)
      printf("�������ɹ���\n");
    break;
    case FR_DISK_ERR:             //(1)
      printf("����Ӳ�����������������\n");
    break;
    case FR_INT_ERR:              //(2)
      printf("�������Դ���\n");
    break;
    case FR_NOT_READY:            //(3)
      printf("���������豸�޷�������\n");
    break;
    case FR_NO_FILE:              //(4)
      printf("�����޷��ҵ��ļ���\n");
    break;
    case FR_NO_PATH:              //(5)
      printf("�����޷��ҵ�·����\n");
    break;
    case FR_INVALID_NAME:         //(6)
      printf("������Ч��·������\n");
    break;
    case FR_DENIED:               //(7)
    case FR_EXIST:                //(8)
      printf("�����ܾ����ʡ�\n");
    break;
    case FR_INVALID_OBJECT:       //(9)
      printf("������Ч���ļ���·����\n");
    break;
    case FR_WRITE_PROTECTED:      //(10)
      printf("�����߼��豸д������\n");
    break;
    case FR_INVALID_DRIVE:        //(11)
      printf("������Ч���߼��豸��\n");
    break;
    case FR_NOT_ENABLED:          //(12)
      printf("������Ч�Ĺ�������\n");
    break;
    case FR_NO_FILESYSTEM:        //(13)
      printf("������Ч���ļ�ϵͳ��\n");
    break;
    case FR_MKFS_ABORTED:         //(14)
      printf("���������������⵼��f_mkfs��������ʧ�ܡ�\n");
    break;
    case FR_TIMEOUT:              //(15)
      printf("����������ʱ��\n");
    break;
    case FR_LOCKED:               //(16)
      printf("�����ļ���������\n");
    break;
    case FR_NOT_ENOUGH_CORE:      //(17)
      printf("�������ļ���֧�ֻ�ȡ�ѿռ�ʧ�ܡ�\n");
    break;
    case FR_TOO_MANY_OPEN_FILES:  //(18)
      printf("������̫���ļ���\n");
    break;
    case FR_INVALID_PARAMETER:    // 19)
      printf("����������Ч��\n");
    break;
  }
}

/*****END OF FILE****/  * �������: FatFS�ļ�ϵͳ���������FRESULT
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void printf_fatfs_error(FRESULT fresult)
{
  switch(fresult)
  {
    case FR_OK:                   //(0)
      printf("�������ɹ���\n");
    break;
    case FR_DISK_ERR:             //(1)
      printf("����Ӳ�����������������\n");
    break;
    case FR_INT_ERR:              //(2)
      printf("�������Դ���\n");
    break;
    case FR_NOT_READY:            //(3)
      printf("���������豸�޷�������\n");
    break;
    case FR_NO_FILE:              //(4)
      printf("�����޷��ҵ��ļ���\n");
    break;
    case FR_NO_PATH:              //(5)
      printf("�����޷��ҵ�·����\n");
    break;
    case FR_INVALID_NAME:         //(6)
      printf("������Ч��·������\n");
    break;
    case FR_DENIED:               //(7)
    case FR_EXIST:                //(8)
      printf("�����ܾ����ʡ�\n");
    break;
    case FR_INVALID_OBJECT:       //(9)
      printf("������Ч���ļ���·����\n");
    break;
    case FR_WRITE_PROTECTED:      //(10)
      printf("�����߼��豸д������\n");
    break;
    case FR_INVALID_DRIVE:        //(11)
      printf("������Ч���߼��豸��\n");
    break;
    case FR_NOT_ENABLED:          //(12)
      printf("������Ч�Ĺ�������\n");
    break;
    case FR_NO_FILESYSTEM:        //(13)
      printf("������Ч���ļ�ϵͳ��\n");
    break;
    case FR_MKFS_ABORTED:         //(14)
      printf("���������������⵼��f_mkfs��������ʧ�ܡ�\n");
    break;
    case FR_TIMEOUT:              //(15)
      printf("����������ʱ��\n");
    break;
    case FR_LOCKED:               //(16)
      printf("�����ļ���������\n");
    break;
    case FR_NOT_ENOUGH_CORE:      //(17)
      printf("�������ļ���֧�ֻ�ȡ�ѿռ�ʧ�ܡ�\n");
    break;
    case FR_TOO_MANY_OPEN_FILES:  //(18)
      printf("������̫���ļ���\n");
    break;
    case FR_INVALID_PARAMETER:    // 19)
      printf("����������Ч��\n");
    break;
  }
}

/*****END OF FILE****/