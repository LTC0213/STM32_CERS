/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���ڴ���FLASH��FatFS�ļ�ϵͳʵ����������ܲ���
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
#include "ff.h"
#include "ff_gen_drv.h"
#include "drivers\spiflash_diskio.h"
#include "string.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
char SPIFLASH_Path[4];            /* ����Flash�߼��豸·�� */
FATFS fs;													/* FatFs�ļ�ϵͳ���� */
FIL file;													/* �ļ����� */
FRESULT f_res;                    /* �ļ�������� */
UINT fnum;            					  /* �ļ��ɹ���д���� */
char fpath[100];                  /* ���浱ǰɨ��·�� */
char readbuffer[512];
DIR dir;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
static void printf_fatfs_error(FRESULT fresult);
static FRESULT miscellaneous(void);
static FRESULT file_check(void);
static FRESULT scan_files (char* path);

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
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  printf("****** ����һ�����ڴ���Flash��FatFS�ļ�ϵͳ����ʹ�� ******\n");
  
  /* ע��һ��FatFS�豸������Flash */
  if(FATFS_LinkDriver(&SPIFLASH_Driver, SPIFLASH_Path) == 0)
  {
    //�ڴ���Flash�����ļ�ϵͳ���ļ�ϵͳ����ʱ��Դ���Flash��ʼ��
    f_res = f_mount(&fs,(TCHAR const*)SPIFLASH_Path,1);
    printf_fatfs_error(f_res);
    if(f_res!=FR_OK)
    {
      printf("��������Flash�����ļ�ϵͳʧ�ܡ�\n");
      while(1);
    }
    else
    {
      printf("������Flash�ļ�ϵͳ���سɹ������Խ��в��ԡ�\n");    
    }
    
    /* FatFs����ܲ��� */
    f_res = miscellaneous();
    
    printf("\n*************** �ļ���Ϣ��ȡ���� **************\r\n");
    f_res = file_check();

    printf("***************** �ļ�ɨ����� ****************\r\n");
    strcpy(fpath,SPIFLASH_Path);
    scan_files(fpath);
    
    /* ����ʹ�ã�ȡ������ */
    f_res = f_mount(NULL,(TCHAR const*)SPIFLASH_Path,1);	
  }  
  
  /* ע��һ��FatFS�豸������Flash */
  FATFS_UnLinkDriver(SPIFLASH_Path);
  
  /* ����ѭ�� */
  while (1)
  {
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
    case FR_INVALID_PARAMETER:    // (19)
      printf("����������Ч��\n");
    break;
  }
}


/* FatFs����ܲ��� */
static FRESULT miscellaneous(void)
{
  FATFS *pfs;
  DWORD fre_clust, fre_sect, tot_sect;
  
  printf("\n*************** �豸��Ϣ��ȡ ***************\r\n");
  /* ��ȡ�豸��Ϣ�Ϳմش�С */
  f_res = f_getfree((TCHAR const*)SPIFLASH_Path, &fre_clust, &pfs);

  /* ����õ��ܵ����������Ϳ��������� */
  tot_sect = (pfs->n_fatent - 2) * pfs->csize;
  fre_sect = fre_clust * pfs->csize;

  /* ��ӡ��Ϣ(4096 �ֽ�/����) */
  printf("���豸�ܿռ䣺%10lu KB��\n�����ÿռ䣺  %10lu KB��\n", tot_sect *4, fre_sect *4);
  
  printf("\n******** �ļ���λ�͸�ʽ��д�빦�ܲ��� ********\r\n");
  f_res = f_open(&file, "FatFs��д�����ļ�.txt",
                            FA_OPEN_EXISTING|FA_WRITE|FA_READ );
  printf("f_res=%d",f_res);
	if ( f_res == FR_OK )
	{
    /*  �ļ���λ */
    f_res = f_lseek(&file,f_size(&file)-1);
    if (f_res == FR_OK)
    {
      /* ��ʽ��д�룬������ʽ����printf���� */
      f_printf(&file,"\n��ԭ���ļ������һ������\n");
      f_printf(&file,"���豸�ܿռ䣺%10lu KB��\n�����ÿռ䣺  %10lu KB��\n", tot_sect *4, fre_sect *4);
      /*  �ļ���λ���ļ���ʼλ�� */
      f_res = f_lseek(&file,0);
      /* ��ȡ�ļ��������ݵ������� */
      f_res = f_read(&file,readbuffer,f_size(&file),&fnum);
      if(f_res == FR_OK)
      {
        printf("���ļ����ݣ�\n%s\n",readbuffer);
      }
    }
    f_close(&file);    
    
    printf("\n********** Ŀ¼���������������ܲ��� **********\r\n");
    /* ���Դ�Ŀ¼ */
    f_res=f_opendir(&dir,"TestDir");
    if(f_res!=FR_OK)
    {
      /* ��Ŀ¼ʧ�ܣ��ʹ���Ŀ¼ */
      f_res=f_mkdir("TestDir");
			printf("����Ŀ¼TestDir\n");
    }
    else
    {
      /* ���Ŀ¼�Ѿ����ڣ��ر��� */
      f_res=f_closedir(&dir);
      /* ɾ���ļ� */
      f_unlink("TestDir/testdir.txt");
    }
    if(f_res==FR_OK)
    {
      /* ���������ƶ��ļ� */
      f_res=f_rename("FatFs��д�����ļ�.txt","TestDir/testdir.txt");      
    } 
	}
  else
  {
    printf("!! ���ļ�ʧ�ܣ�%d\n",f_res);
    printf("!! ������Ҫ�ٴ����С�FatFs��ֲ���д���ԡ�����\n");
  }
  return f_res;
}


/**
  * �ļ���Ϣ��ȡ
  */
static FRESULT file_check(void)
{  
  static FILINFO finfo;
  /* ��ȡ�ļ���Ϣ */
  f_res=f_stat("TestDir/testdir.txt",&finfo);
  if(f_res==FR_OK)
  {
    printf("��testdir.txt���ļ���Ϣ��\n");
    printf("���ļ���С: %ld(�ֽ�)\n", finfo.fsize);
    printf("��ʱ���: %u/%02u/%02u, %02u:%02u\n",
           (finfo.fdate >> 9) + 1980, finfo.fdate >> 5 & 15, finfo.fdate & 31,finfo.ftime >> 11, finfo.ftime >> 5 & 63);
    printf("������: %c%c%c%c%c\n\n",
           (finfo.fattrib & AM_DIR) ? 'D' : '-',      // ��һ��Ŀ¼
           (finfo.fattrib & AM_RDO) ? 'R' : '-',      // ֻ���ļ�
           (finfo.fattrib & AM_HID) ? 'H' : '-',      // �����ļ�
           (finfo.fattrib & AM_SYS) ? 'S' : '-',      // ϵͳ�ļ�
           (finfo.fattrib & AM_ARC) ? 'A' : '-');     // �����ļ�
  }
  return f_res;
}

/**
  * @brief  scan_files �ݹ�ɨ��FatFs�ڵ��ļ�
  * @param  path:��ʼɨ��·��
  * @retval result:�ļ�ϵͳ�ķ���ֵ
  */
static FRESULT scan_files (char* path) 
{ 
  FRESULT res; 		//�����ڵݹ���̱��޸ĵı���������ȫ�ֱ���	
  FILINFO fno;  
  int i;            
  char *fn;        // �ļ���	
	
#if _USE_LFN 
  /* ���ļ���֧�� */
  /* ����������Ҫ2���ֽڱ���һ�����֡�*/
  static char lfn[_MAX_LFN*2 + 1]; 	
  fno.lfname = lfn; 
  fno.lfsize = sizeof(lfn); 
#endif 
  //��Ŀ¼
  res = f_opendir(&dir, path); 
  if (res == FR_OK) 
	{ 
    i = strlen(path); 
    for (;;) 
		{ 
      //��ȡĿ¼�µ����ݣ��ٶ����Զ�����һ���ļ�
      res = f_readdir(&dir, &fno); 								
      //Ϊ��ʱ��ʾ������Ŀ��ȡ��ϣ�����
      if (res != FR_OK || fno.fname[0] == 0) break; 	
#if _USE_LFN 
      fn = *fno.lfname ? fno.lfname : fno.fname; 
#else 
      fn = fno.fname; 
#endif 
      //���ʾ��ǰĿ¼������			
      if (*fn == '.') continue; 	
      //Ŀ¼���ݹ��ȡ      
      if (fno.fattrib & AM_DIR)         
			{ 			
        //�ϳ�����Ŀ¼��        
        sprintf(&path[i], "/%s", fn); 		
        //�ݹ����         
        res = scan_files(path);	
        path[i] = 0;         
        //��ʧ�ܣ�����ѭ��        
        if (res != FR_OK) 
					break; 
      } 
			else 
			{ 
				printf("%s/%s\n", path, fn);								//����ļ���	
        /* ������������ȡ�ض���ʽ���ļ�·�� */        
      }//else
    } //for
  } 
  return res; 
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
