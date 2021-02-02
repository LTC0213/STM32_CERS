/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 基于串行FLASH的FatFS文件系统实现与基本功能测试
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
#include "ff.h"
#include "ff_gen_drv.h"
#include "drivers\spiflash_diskio.h"
#include "string.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
char SPIFLASH_Path[4];            /* 串行Flash逻辑设备路径 */
FATFS fs;													/* FatFs文件系统对象 */
FIL file;													/* 文件对象 */
FRESULT f_res;                    /* 文件操作结果 */
UINT fnum;            					  /* 文件成功读写数量 */
char fpath[100];                  /* 保存当前扫描路径 */
char readbuffer[512];
DIR dir;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
static void printf_fatfs_error(FRESULT fresult);
static FRESULT miscellaneous(void);
static FRESULT file_check(void);
static FRESULT scan_files (char* path);

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
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  printf("****** 这是一个基于串行Flash的FatFS文件系统功能使用 ******\n");
  
  /* 注册一个FatFS设备：串行Flash */
  if(FATFS_LinkDriver(&SPIFLASH_Driver, SPIFLASH_Path) == 0)
  {
    //在串行Flash挂载文件系统，文件系统挂载时会对串行Flash初始化
    f_res = f_mount(&fs,(TCHAR const*)SPIFLASH_Path,1);
    printf_fatfs_error(f_res);
    if(f_res!=FR_OK)
    {
      printf("！！串行Flash挂载文件系统失败。\n");
      while(1);
    }
    else
    {
      printf("》串行Flash文件系统挂载成功，可以进行测试。\n");    
    }
    
    /* FatFs多项功能测试 */
    f_res = miscellaneous();
    
    printf("\n*************** 文件信息获取测试 **************\r\n");
    f_res = file_check();

    printf("***************** 文件扫描测试 ****************\r\n");
    strcpy(fpath,SPIFLASH_Path);
    scan_files(fpath);
    
    /* 不再使用，取消挂载 */
    f_res = f_mount(NULL,(TCHAR const*)SPIFLASH_Path,1);	
  }  
  
  /* 注销一个FatFS设备：串行Flash */
  FATFS_UnLinkDriver(SPIFLASH_Path);
  
  /* 无限循环 */
  while (1)
  {
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
    case FR_INVALID_PARAMETER:    // (19)
      printf("！！参数无效。\n");
    break;
  }
}


/* FatFs多项功能测试 */
static FRESULT miscellaneous(void)
{
  FATFS *pfs;
  DWORD fre_clust, fre_sect, tot_sect;
  
  printf("\n*************** 设备信息获取 ***************\r\n");
  /* 获取设备信息和空簇大小 */
  f_res = f_getfree((TCHAR const*)SPIFLASH_Path, &fre_clust, &pfs);

  /* 计算得到总的扇区个数和空扇区个数 */
  tot_sect = (pfs->n_fatent - 2) * pfs->csize;
  fre_sect = fre_clust * pfs->csize;

  /* 打印信息(4096 字节/扇区) */
  printf("》设备总空间：%10lu KB。\n》可用空间：  %10lu KB。\n", tot_sect *4, fre_sect *4);
  
  printf("\n******** 文件定位和格式化写入功能测试 ********\r\n");
  f_res = f_open(&file, "FatFs读写测试文件.txt",
                            FA_OPEN_EXISTING|FA_WRITE|FA_READ );
  printf("f_res=%d",f_res);
	if ( f_res == FR_OK )
	{
    /*  文件定位 */
    f_res = f_lseek(&file,f_size(&file)-1);
    if (f_res == FR_OK)
    {
      /* 格式化写入，参数格式类似printf函数 */
      f_printf(&file,"\n在原来文件新添加一行内容\n");
      f_printf(&file,"》设备总空间：%10lu KB。\n》可用空间：  %10lu KB。\n", tot_sect *4, fre_sect *4);
      /*  文件定位到文件起始位置 */
      f_res = f_lseek(&file,0);
      /* 读取文件所有内容到缓存区 */
      f_res = f_read(&file,readbuffer,f_size(&file),&fnum);
      if(f_res == FR_OK)
      {
        printf("》文件内容：\n%s\n",readbuffer);
      }
    }
    f_close(&file);    
    
    printf("\n********** 目录创建和重命名功能测试 **********\r\n");
    /* 尝试打开目录 */
    f_res=f_opendir(&dir,"TestDir");
    if(f_res!=FR_OK)
    {
      /* 打开目录失败，就创建目录 */
      f_res=f_mkdir("TestDir");
			printf("创建目录TestDir\n");
    }
    else
    {
      /* 如果目录已经存在，关闭它 */
      f_res=f_closedir(&dir);
      /* 删除文件 */
      f_unlink("TestDir/testdir.txt");
    }
    if(f_res==FR_OK)
    {
      /* 重命名并移动文件 */
      f_res=f_rename("FatFs读写测试文件.txt","TestDir/testdir.txt");      
    } 
	}
  else
  {
    printf("!! 打开文件失败：%d\n",f_res);
    printf("!! 或许需要再次运行“FatFs移植与读写测试”工程\n");
  }
  return f_res;
}


/**
  * 文件信息获取
  */
static FRESULT file_check(void)
{  
  static FILINFO finfo;
  /* 获取文件信息 */
  f_res=f_stat("TestDir/testdir.txt",&finfo);
  if(f_res==FR_OK)
  {
    printf("“testdir.txt”文件信息：\n");
    printf("》文件大小: %ld(字节)\n", finfo.fsize);
    printf("》时间戳: %u/%02u/%02u, %02u:%02u\n",
           (finfo.fdate >> 9) + 1980, finfo.fdate >> 5 & 15, finfo.fdate & 31,finfo.ftime >> 11, finfo.ftime >> 5 & 63);
    printf("》属性: %c%c%c%c%c\n\n",
           (finfo.fattrib & AM_DIR) ? 'D' : '-',      // 是一个目录
           (finfo.fattrib & AM_RDO) ? 'R' : '-',      // 只读文件
           (finfo.fattrib & AM_HID) ? 'H' : '-',      // 隐藏文件
           (finfo.fattrib & AM_SYS) ? 'S' : '-',      // 系统文件
           (finfo.fattrib & AM_ARC) ? 'A' : '-');     // 档案文件
  }
  return f_res;
}

/**
  * @brief  scan_files 递归扫描FatFs内的文件
  * @param  path:初始扫描路径
  * @retval result:文件系统的返回值
  */
static FRESULT scan_files (char* path) 
{ 
  FRESULT res; 		//部分在递归过程被修改的变量，不用全局变量	
  FILINFO fno;  
  int i;            
  char *fn;        // 文件名	
	
#if _USE_LFN 
  /* 长文件名支持 */
  /* 简体中文需要2个字节保存一个“字”*/
  static char lfn[_MAX_LFN*2 + 1]; 	
  fno.lfname = lfn; 
  fno.lfsize = sizeof(lfn); 
#endif 
  //打开目录
  res = f_opendir(&dir, path); 
  if (res == FR_OK) 
	{ 
    i = strlen(path); 
    for (;;) 
		{ 
      //读取目录下的内容，再读会自动读下一个文件
      res = f_readdir(&dir, &fno); 								
      //为空时表示所有项目读取完毕，跳出
      if (res != FR_OK || fno.fname[0] == 0) break; 	
#if _USE_LFN 
      fn = *fno.lfname ? fno.lfname : fno.fname; 
#else 
      fn = fno.fname; 
#endif 
      //点表示当前目录，跳过			
      if (*fn == '.') continue; 	
      //目录，递归读取      
      if (fno.fattrib & AM_DIR)         
			{ 			
        //合成完整目录名        
        sprintf(&path[i], "/%s", fn); 		
        //递归遍历         
        res = scan_files(path);	
        path[i] = 0;         
        //打开失败，跳出循环        
        if (res != FR_OK) 
					break; 
      } 
			else 
			{ 
				printf("%s/%s\n", path, fn);								//输出文件名	
        /* 可以在这里提取特定格式的文件路径 */        
      }//else
    } //for
  } 
  return res; 
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
