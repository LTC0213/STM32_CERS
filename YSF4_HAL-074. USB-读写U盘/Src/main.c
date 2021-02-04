/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: USB-��дU�̼�����
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
#include "usart/bsp_debug_usart.h"
#include "spiflash/bsp_spiflash.h"
#include "led/bsp_led.h"
#include "usbh_core.h"
#include "usbh_msc.h" 
#include "ff.h"
#include "ff_gen_drv.h"
#include "key/bsp_key.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_READY,    
  APPLICATION_DISCONNECT,
}MSC_ApplicationTypeDef;

/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
USBH_HandleTypeDef hUSBHost;
MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;

char USBDISKPath[4];             /* ����Flash�߼��豸·�� */
FATFS fs;													/* FatFs�ļ�ϵͳ���� */
FIL file;													/* �ļ����� */
FRESULT f_res;                    /* �ļ�������� */
UINT fnum;            					  /* �ļ��ɹ���д���� */
DIR fdir;

char scan_path[255] = "0:";      /* �ݹ�ɨ���ļ�ʱʹ�õ�·�� */

BYTE ReadBuffer[1024]={0};       /* �������� */
BYTE WriteBuffer[]= "ӲʯSTM32F4��������ԣ���U�����½��ļ�ϵͳ�����ļ�\n";/* д������*/  
BYTE WriteBuffer_test[100]; 
BYTE WriteBuffer_dir[1024];
BYTE WriteBuffer_dir_next[1024];
BYTE WriteBuffer_csv[1024];

__IO uint8_t key1_temp=0;

/* ��չ���� ------------------------------------------------------------------*/
extern Diskio_drvTypeDef  USBH_Driver;

/* ˽�к���ԭ�� --------------------------------------------------------------*/

uint8_t exf_getfree(uint8_t *drv,uint32_t *total,uint32_t *free);
static void read_write_flie(void);
static void write_flie(void);
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

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  uint32_t total,free;
	uint8_t res=0,counts=0;
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  /* LED�Ƴ�ʼ�� */
  LED_GPIO_Init();
  /* KEY������ʼ�� */
  KEY_GPIO_Init();
  /* ��ʼ��LED�� */
  LED_GPIO_Init();
  
  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  printf("USB  U�̲���\n");
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
  printf("\r\n****** ���������ļ�д�����... ******\r\n");	
  f_res = f_mount(&fs,"0:",1);
	//printf_fatfs_error(f_res);
  f_res = f_open(&file, "FatFs��д�����ļ�.txt",FA_CREATE_ALWAYS | FA_WRITE );
  
  /* ����ѭ�� */
  while (1)
  {
    USBH_Process(&hUSBHost);
    HAL_Delay(1);
		counts++;
		if(counts==200)
		{
			LED1_TOGGLE;
			counts=0;
		}     
    /* ����key1��ɨ��U���ڵ��ļ� */
    if(KEY1_StateRead()==KEY_DOWN)
    {
      key1_temp=1;
    }
    /* ����key2����/�����ļ����ж�д���� */
    if(KEY2_StateRead()==KEY_DOWN)
    {
      key1_temp=2;
    }  
    /* ����key3����ѯU��������ʣ������ */    
    if(KEY3_StateRead()==KEY_DOWN)
    {
      key1_temp=3;
    }
    if(KEY4_StateRead()==KEY_DOWN)
    {
      key1_temp=4;
    }
    if(KEY5_StateRead()==KEY_DOWN)
    {
      key1_temp=5;
    }      
    if(key1_temp==1)
    {
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
      f_res = f_open(&file, "FatFs��д�����ļ�.txt",FA_CREATE_ALWAYS | FA_WRITE );
    }
    if(key1_temp==2)
    {
      // f_res = f_mount(&fs,"0:",1);	/* �ڴ���FLASH�����ļ�ϵͳ���ļ�ϵͳ����ʱ��Դ���FLASH��ʼ�� */
      // // printf_fatfs_error(f_res);
      // if(f_res != FR_OK)
      // {
      //   printf("\r\n�ļ�ϵͳ����ʧ�ܣ�\r\n");
      //   continue; 
      // }
      sprintf(WriteBuffer_test, "%d", counts);
      f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
      f_sync(&file);
      printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
      printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer_test);
      sprintf(WriteBuffer_test, "%d", 0000);
      // write_flie();	
      // read_write_flie();
      // read_write_flie();
      key1_temp=0;
    } 
    if(key1_temp==5)
    {
      // f_close(&file);
      // f_res=exf_getfree("0:",&total,&free);
      // if(f_res==0)
      // {
      //   printf("total=%dMB free=%dMB\n",total>>10,free>>10);
      //   key1_temp=0;
      // }   
      sprintf(WriteBuffer_test, "\r\n"); 
      f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
      f_sync(&file);
      printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
      printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer_test); 
      key1_temp=0;
    }
    if(key1_temp==3)
    {
      sprintf(WriteBuffer_dir,"%s","�ļ��в����ļ���");
      f_res=f_mkdir(WriteBuffer_dir);
      printf_fatfs_error(f_res);

      // sprintf(WriteBuffer_dir_next,"/�ļ��в����ļ���");
      // f_res=f_chdir(WriteBuffer_dir_next);
      // printf_fatfs_error(f_res);

      sprintf( WriteBuffer_csv,"/%s/%s.csv",WriteBuffer_dir,"�ļ��в����ļ�");
      // f_res = f_open(&file, "FatFs��д�����ļ�.txt",FA_CREATE_ALWAYS | FA_WRITE );
      f_res =f_open(&file, WriteBuffer_csv,FA_CREATE_ALWAYS | FA_WRITE );
      printf_fatfs_error(f_res);

      sprintf(WriteBuffer_test, "%d,", counts);
      f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
      f_sync(&file);
      printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
      printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer_test);
      sprintf(WriteBuffer_test, "%d", 0000);
      key1_temp=0;
    }
    if(key1_temp==4)
    {
      sprintf(WriteBuffer_test, "%d,", counts);
      f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
      f_sync(&file);
      printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
      printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer_test);
      key1_temp=0;
    //  f_res =f_mkdir("0:/2017110223");
    //  printf_fatfs_error(f_res);

    //  f_res =f_open(&file, "/2017110223/201711011.txt", FA_CREATE_ALWAYS | FA_WRITE);
    //  printf_fatfs_error(f_res);

    //  f_res = f_close(&file);
    //  printf_fatfs_error(f_res);
    //  key1_temp=0;

    }      
  }
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
  * ��������: �ļ�ϵͳ��д����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void read_write_flie(void)
{	

/*----------------------- �ļ�ϵͳ���ԣ�д���� -----------------------------*/
  /* ���ļ�������ļ��������򴴽��� */
  printf("\r\n****** ���������ļ�д�����... ******\r\n");	
  f_res = f_open(&file, "FatFs��д�����ļ�.txt",FA_CREATE_ALWAYS | FA_WRITE );
  printf_fatfs_error(f_res);
  if ( f_res == FR_OK )
  {
    printf("����/����FatFs��д�����ļ�.txt�ļ��ɹ������ļ�д�����ݡ�\r\n");
    /* ��ָ���洢������д�뵽�ļ��� */
		f_res=f_write(&file,WriteBuffer,sizeof(WriteBuffer),&fnum);
    // f_res=f_write(&file,WriteBuffer,sizeof(WriteBuffer),&fnum);
    if(f_res==FR_OK)
    {
      printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
      printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer);
    }
    else
    {
      printf("�����ļ�д��ʧ�ܣ�(%d)\n",f_res);
    }    
		/* ���ٶ�д���ر��ļ� */
    f_close(&file);
  }
  else
  {	
          printf("������/�����ļ�ʧ�ܡ�\r\n");
  }
	
/*------------------- �ļ�ϵͳ���ԣ������� ------------------------------------*/
  printf("****** ���������ļ���ȡ����... ******\r\n");
  f_res = f_open(&file, "FatFs��д�����ļ�.txt", FA_OPEN_EXISTING | FA_READ); 	 
  if(f_res == FR_OK)
  {
    printf("�����ļ��ɹ���\r\n");
    f_res = f_read(&file, ReadBuffer, sizeof(ReadBuffer), &fnum); 
    if(f_res==FR_OK)
    {
      printf("���ļ���ȡ�ɹ�,�����ֽ����ݣ�%d\r\n",fnum);
      printf("����ȡ�õ��ļ�����Ϊ��\r\n%s \r\n", ReadBuffer);	
    }
    else
    {
      printf("�����ļ���ȡʧ�ܣ�(%d)\n",f_res);
    }		
  }
  else
  {
          printf("�������ļ�ʧ�ܡ�\r\n");
  }
  /* ���ٶ�д���ر��ļ� */
  f_close(&file);	
}

/**
  * ��������: �ļ�ϵͳд�ļ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void write_flie(void)
{	

  
  /* ���ļ�������ļ��������򴴽��� */
  //printf("\r\n****** ���������ļ�д��... ******\r\n");	
  f_res = f_mount(&fs,"0:",1);	/* �ڴ���FLASH�����ļ�ϵͳ���ļ�ϵͳ����ʱ��Դ���FLASH��ʼ�� */
  // printf_fatfs_error(f_res);
  if(f_res != FR_OK)
  {
    printf("\r\n�ļ�ϵͳ����ʧ�ܣ�\r\n");
  }
  f_res = f_open(&file, "�������ݼ�¼�ļ�.txt",FA_CREATE_ALWAYS | FA_WRITE );
  // printf_fatfs_error(f_res);
  if ( f_res == FR_OK )
  {
    //printf("����/����FatFs��д�����ļ�.txt�ļ��ɹ������ļ�д�����ݡ�\r\n");
    /* ��ָ���洢������д�뵽�ļ��� */
		f_res=f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
    if(f_res==FR_OK)
    {
      printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
      printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer_test);
    }
    else
    {
      printf("�����ļ�д��ʧ�ܣ�(%d)\n",f_res);
    }    
		/* ���ٶ�д���ر��ļ� */
    //f_close(&file);
  }
  else
  {	
          printf("������/�����ļ�ʧ�ܡ�\r\n");
  }
  /* ���ٶ�д���ر��ļ� */
  f_close(&file);	
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


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
