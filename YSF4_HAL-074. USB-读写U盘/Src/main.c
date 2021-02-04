/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: USB-读写U盘及测试
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart/bsp_debug_usart.h"
#include "spiflash/bsp_spiflash.h"
#include "led/bsp_led.h"
#include "usbh_core.h"
#include "usbh_msc.h" 
#include "ff.h"
#include "ff_gen_drv.h"
#include "key/bsp_key.h"

/* 私有类型定义 --------------------------------------------------------------*/
typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_READY,    
  APPLICATION_DISCONNECT,
}MSC_ApplicationTypeDef;

/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
USBH_HandleTypeDef hUSBHost;
MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;

char USBDISKPath[4];             /* 串行Flash逻辑设备路径 */
FATFS fs;													/* FatFs文件系统对象 */
FIL file;													/* 文件对象 */
FRESULT f_res;                    /* 文件操作结果 */
UINT fnum;            					  /* 文件成功读写数量 */
DIR fdir;

char scan_path[255] = "0:";      /* 递归扫描文件时使用的路径 */

BYTE ReadBuffer[1024]={0};       /* 读缓冲区 */
BYTE WriteBuffer[]= "硬石STM32F4开发板测试，在U盘内新建文件系统测试文件\n";/* 写缓冲区*/  
BYTE WriteBuffer_test[100]; 
BYTE WriteBuffer_dir[1024];
BYTE WriteBuffer_dir_next[1024];
BYTE WriteBuffer_csv[1024];

__IO uint8_t key1_temp=0;

/* 扩展变量 ------------------------------------------------------------------*/
extern Diskio_drvTypeDef  USBH_Driver;

/* 私有函数原形 --------------------------------------------------------------*/

uint8_t exf_getfree(uint8_t *drv,uint32_t *total,uint32_t *free);
static void read_write_flie(void);
static void write_flie(void);
static FRESULT scan_files (char* path) ;
static void printf_fatfs_error(FRESULT fresult);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

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
  uint32_t total,free;
	uint8_t res=0,counts=0;
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  /* LED灯初始化 */
  LED_GPIO_Init();
  /* KEY按键初始化 */
  KEY_GPIO_Init();
  /* 初始化LED灯 */
  LED_GPIO_Init();
  
  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  printf("USB  U盘测试\n");
  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
  {
    /* 初始化用户进程 */
    if(USBH_Init(&hUSBHost, USBH_UserProcess, HOST_FS)==USBH_OK)
    {
      printf("USB INIT  OK!\n");
    }
    
    /* 添加系统MSC进程类 */
    USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);
    
    /* 启动主机进程 */
    USBH_Start(&hUSBHost);      
  }
  printf("\r\n****** 即将进行文件写入测试... ******\r\n");	
  f_res = f_mount(&fs,"0:",1);
	//printf_fatfs_error(f_res);
  f_res = f_open(&file, "FatFs读写测试文件.txt",FA_CREATE_ALWAYS | FA_WRITE );
  
  /* 无限循环 */
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
    /* 按下key1，扫描U盘内的文件 */
    if(KEY1_StateRead()==KEY_DOWN)
    {
      key1_temp=1;
    }
    /* 按下key2，打开/创建文件进行读写测试 */
    if(KEY2_StateRead()==KEY_DOWN)
    {
      key1_temp=2;
    }  
    /* 按下key3，查询U盘容量和剩余容量 */    
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
      f_res = f_mount(&fs,"0:",1);	/* 在串行FLASH挂载文件系统，文件系统挂载时会对串行FLASH初始化 */
      printf_fatfs_error(f_res);
      if(f_res != FR_OK)
      {
        printf("\r\n文件系统挂载失败！\r\n");
        continue; 
      }
      printf("\r\n扫描到的U盘文件：\r\n");
      scan_files(scan_path); 
      key1_temp=0;
      f_res = f_open(&file, "FatFs读写测试文件.txt",FA_CREATE_ALWAYS | FA_WRITE );
    }
    if(key1_temp==2)
    {
      // f_res = f_mount(&fs,"0:",1);	/* 在串行FLASH挂载文件系统，文件系统挂载时会对串行FLASH初始化 */
      // // printf_fatfs_error(f_res);
      // if(f_res != FR_OK)
      // {
      //   printf("\r\n文件系统挂载失败！\r\n");
      //   continue; 
      // }
      sprintf(WriteBuffer_test, "%d", counts);
      f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
      f_sync(&file);
      printf("》文件写入成功，写入字节数据：%d\n",fnum);
      printf("》向文件写入的数据为：\r\n%s\r\n",WriteBuffer_test);
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
      printf("》文件写入成功，写入字节数据：%d\n",fnum);
      printf("》向文件写入的数据为：\r\n%s\r\n",WriteBuffer_test); 
      key1_temp=0;
    }
    if(key1_temp==3)
    {
      sprintf(WriteBuffer_dir,"%s","文件夹测试文件夹");
      f_res=f_mkdir(WriteBuffer_dir);
      printf_fatfs_error(f_res);

      // sprintf(WriteBuffer_dir_next,"/文件夹测试文件夹");
      // f_res=f_chdir(WriteBuffer_dir_next);
      // printf_fatfs_error(f_res);

      sprintf( WriteBuffer_csv,"/%s/%s.csv",WriteBuffer_dir,"文件夹测试文件");
      // f_res = f_open(&file, "FatFs读写测试文件.txt",FA_CREATE_ALWAYS | FA_WRITE );
      f_res =f_open(&file, WriteBuffer_csv,FA_CREATE_ALWAYS | FA_WRITE );
      printf_fatfs_error(f_res);

      sprintf(WriteBuffer_test, "%d,", counts);
      f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
      f_sync(&file);
      printf("》文件写入成功，写入字节数据：%d\n",fnum);
      printf("》向文件写入的数据为：\r\n%s\r\n",WriteBuffer_test);
      sprintf(WriteBuffer_test, "%d", 0000);
      key1_temp=0;
    }
    if(key1_temp==4)
    {
      sprintf(WriteBuffer_test, "%d,", counts);
      f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
      f_sync(&file);
      printf("》文件写入成功，写入字节数据：%d\n",fnum);
      printf("》向文件写入的数据为：\r\n%s\r\n",WriteBuffer_test);
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
  * 函数功能: 得到磁盘容量以及剩余容量
  * 输入参数: drv:磁盘编号"0:" total:总容量 单位KB  free:剩余容量 单位KB
  * 返 回 值: 返回值:0,正常.其他,错误代码
  * 说    明: 无
  */
uint8_t exf_getfree(uint8_t *drv,uint32_t *total,uint32_t *free) 
{
	FATFS *fs1;
	uint8_t res;
  uint32_t fre_clust=0, fre_sect=0, tot_sect=0;
  /* 得到磁盘信息及空闲簇数量 */
  res =(uint32_t)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
  if(res==0)
	{											   
	  tot_sect=(fs1->n_fatent-2)*fs1->csize;	/* 得到总扇区数 */
	  fre_sect=fre_clust*fs1->csize;			/* 得到空闲扇区数	*/   
#if _MAX_SS!=512				  				/* 扇区大小不是512字节,则转换为512字节 */
		tot_sect*=fs1->ssize/512;
		fre_sect*=fs1->ssize/512;
#endif	  
		*total=tot_sect>>1;	/* 单位为KB */
		*free=fre_sect>>1;	/* 单位为KB */ 
 	}
	return res;
}		

/**
  * 函数功能: 递归扫描FatFs内的文件
  * 输入参数: 初始扫描路径
  * 返 回 值: res:文件系统的返回值
  * 说    明: 无
  */
static FRESULT scan_files (char* path) 
{ 
  FRESULT res; 		/* 部分在递归过程被修改的变量，不用全局变量 */	
  FILINFO fno; 
  DIR dir; 
  int i;            
  char *fn;       /* 文件名 */
	
#if _USE_LFN 
  /* 长文件名支持 */
  /* 简体中文需要2个字节保存一个“字”*/
  static char lfn[_MAX_LFN*2 + 1]; 	
  fno.lfname = lfn; 
  fno.lfsize = sizeof(lfn); 
#endif 
  /* 打开目录 */
  res = f_opendir(&dir, path); 
  if (res == FR_OK) 
	{ 
    i = strlen(path); 
    for (;;) 
		{ 
      /* 读取目录下的内容，再读会自动读下一个文件 */
      res = f_readdir(&dir, &fno); 								
      /* 为空时表示所有项目读取完毕，跳出 */
      if (res != FR_OK || fno.fname[0] == 0) break; 	
#if _USE_LFN 
      fn = *fno.lfname ? fno.lfname : fno.fname; 
#else 
      fn = fno.fname; 
#endif 
      /* 点表示当前目录，跳过 */			
      if (*fn == '.') continue; 	
      /* 目录，递归读取 */      
      if (fno.fattrib & AM_DIR)         
			{ 			
        /* 合成完整目录名 */        
        sprintf(&path[i], "/%s", fn); 		
        /* 递归遍历 */         
        res = scan_files(path);	
        path[i] = 0;         
        /* 打开失败，跳出循环 */        
        if (res != FR_OK) 
	break; 
      } 
      else 
      { 
              printf("%s/%s\r\n", path, fn);								/* 输出文件名	*/
        /* 可以在这里提取特定格式的文件路径 */        
      }//else
    } //for
  } 
  return res; 
}

/**
  * 函数功能: 文件系统读写测试
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void read_write_flie(void)
{	

/*----------------------- 文件系统测试：写测试 -----------------------------*/
  /* 打开文件，如果文件不存在则创建它 */
  printf("\r\n****** 即将进行文件写入测试... ******\r\n");	
  f_res = f_open(&file, "FatFs读写测试文件.txt",FA_CREATE_ALWAYS | FA_WRITE );
  printf_fatfs_error(f_res);
  if ( f_res == FR_OK )
  {
    printf("》打开/创建FatFs读写测试文件.txt文件成功，向文件写入数据。\r\n");
    /* 将指定存储区内容写入到文件内 */
		f_res=f_write(&file,WriteBuffer,sizeof(WriteBuffer),&fnum);
    // f_res=f_write(&file,WriteBuffer,sizeof(WriteBuffer),&fnum);
    if(f_res==FR_OK)
    {
      printf("》文件写入成功，写入字节数据：%d\n",fnum);
      printf("》向文件写入的数据为：\r\n%s\r\n",WriteBuffer);
    }
    else
    {
      printf("！！文件写入失败：(%d)\n",f_res);
    }    
		/* 不再读写，关闭文件 */
    f_close(&file);
  }
  else
  {	
          printf("！！打开/创建文件失败。\r\n");
  }
	
/*------------------- 文件系统测试：读测试 ------------------------------------*/
  printf("****** 即将进行文件读取测试... ******\r\n");
  f_res = f_open(&file, "FatFs读写测试文件.txt", FA_OPEN_EXISTING | FA_READ); 	 
  if(f_res == FR_OK)
  {
    printf("》打开文件成功。\r\n");
    f_res = f_read(&file, ReadBuffer, sizeof(ReadBuffer), &fnum); 
    if(f_res==FR_OK)
    {
      printf("》文件读取成功,读到字节数据：%d\r\n",fnum);
      printf("》读取得的文件数据为：\r\n%s \r\n", ReadBuffer);	
    }
    else
    {
      printf("！！文件读取失败：(%d)\n",f_res);
    }		
  }
  else
  {
          printf("！！打开文件失败。\r\n");
  }
  /* 不再读写，关闭文件 */
  f_close(&file);	
}

/**
  * 函数功能: 文件系统写文件
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void write_flie(void)
{	

  
  /* 打开文件，如果文件不存在则创建它 */
  //printf("\r\n****** 即将进行文件写入... ******\r\n");	
  f_res = f_mount(&fs,"0:",1);	/* 在串行FLASH挂载文件系统，文件系统挂载时会对串行FLASH初始化 */
  // printf_fatfs_error(f_res);
  if(f_res != FR_OK)
  {
    printf("\r\n文件系统挂载失败！\r\n");
  }
  f_res = f_open(&file, "测试数据记录文件.txt",FA_CREATE_ALWAYS | FA_WRITE );
  // printf_fatfs_error(f_res);
  if ( f_res == FR_OK )
  {
    //printf("》打开/创建FatFs读写测试文件.txt文件成功，向文件写入数据。\r\n");
    /* 将指定存储区内容写入到文件内 */
		f_res=f_write(&file,WriteBuffer_test,sizeof(WriteBuffer_test),&fnum);
    if(f_res==FR_OK)
    {
      printf("》文件写入成功，写入字节数据：%d\n",fnum);
      printf("》向文件写入的数据为：\r\n%s\r\n",WriteBuffer_test);
    }
    else
    {
      printf("！！文件写入失败：(%d)\n",f_res);
    }    
		/* 不再读写，关闭文件 */
    //f_close(&file);
  }
  else
  {	
          printf("！！打开/创建文件失败。\r\n");
  }
  /* 不再读写，关闭文件 */
  f_close(&file);	
}

/**
  * 函数功能: 用户进程
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
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
  * 函数功能: FatFS文件系统操作结果信息处理.
  * 输入参数: FatFS文件系统操作结果：FRESULT
  * 返 回 值: 无
  * 说    明: 无
  */
static void printf_fatfs_error(FRESULT fresult)
{
  switch(fresult)
  {
    case FR_OK:                   //(0)
      printf("》操作成功。\n");
    break;
    case FR_DISK_ERR:             //(1)
      printf("！！硬件输入输出驱动出错。\n");
    break;
    case FR_INT_ERR:              //(2)
      printf("！！断言错误。\n");
    break;
    case FR_NOT_READY:            //(3)
      printf("！！物理设备无法工作。\n");
    break;
    case FR_NO_FILE:              //(4)
      printf("！！无法找到文件。\n");
    break;
    case FR_NO_PATH:              //(5)
      printf("！！无法找到路径。\n");
    break;
    case FR_INVALID_NAME:         //(6)
      printf("！！无效的路径名。\n");
    break;
    case FR_DENIED:               //(7)
    case FR_EXIST:                //(8)
      printf("！！拒绝访问。\n");
    break;
    case FR_INVALID_OBJECT:       //(9)
      printf("！！无效的文件或路径。\n");
    break;
    case FR_WRITE_PROTECTED:      //(10)
      printf("！！逻辑设备写保护。\n");
    break;
    case FR_INVALID_DRIVE:        //(11)
      printf("！！无效的逻辑设备。\n");
    break;
    case FR_NOT_ENABLED:          //(12)
      printf("！！无效的工作区。\n");
    break;
    case FR_NO_FILESYSTEM:        //(13)
      printf("！！无效的文件系统。\n");
    break;
    case FR_MKFS_ABORTED:         //(14)
      printf("！！因函数参数问题导致f_mkfs函数操作失败。\n");
    break;
    case FR_TIMEOUT:              //(15)
      printf("！！操作超时。\n");
    break;
    case FR_LOCKED:               //(16)
      printf("！！文件被保护。\n");
    break;
    case FR_NOT_ENOUGH_CORE:      //(17)
      printf("！！长文件名支持获取堆空间失败。\n");
    break;
    case FR_TOO_MANY_OPEN_FILES:  //(18)
      printf("！！打开太多文件。\n");
    break;
    case FR_INVALID_PARAMETER:    // 19)
      printf("！！参数无效。\n");
    break;
  }
}


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
