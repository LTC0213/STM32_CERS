/**
  ******************************************************************************
  * 文件名程: diskio.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: FatFS文件系统存储设备输入输出接口实现
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
#include "diskio.h"
#include "ff_gen_drv.h"
#include "ff.h"

#if _USE_LFN != 0   // 如果使能长文件名，添加相关解码文件

#if _CODE_PAGE == 936	/* 简体中文：GBK */
#include "option\cc936.c"
#elif _CODE_PAGE == 950	/* 繁体中文：Big5 */
#include "option\cc950.c"
#else					/* Single Byte Character-Set */
#include "option\ccsbcs.c"
#endif

#endif

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
extern Disk_drvTypeDef  disk;

/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 获取物理设备状态 
  * 输入参数: pdrv：物理设备编号
  * 返 回 值: DSTATUS：操作结果
  * 说    明: 无
  */
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
  DSTATUS stat;
  
  stat = disk.drv[pdrv]->disk_status(disk.lun[pdrv]);
  return stat;
}

/**
  * 函数功能: 初始化物理设备
  * 输入参数: pdrv：物理设备编号
  * 返 回 值: DSTATUS：操作结果
  * 说    明: 无
  */
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
  DSTATUS stat = RES_OK;
  
  if(disk.is_initialized[pdrv] == 0)
  { 
    disk.is_initialized[pdrv] = 1;
    stat = disk.drv[pdrv]->disk_initialize(disk.lun[pdrv]);
  }
  return stat;
}

/**
  * 函数功能: 从物理设备读取数据到缓冲区
  * 输入参数: pdrv：物理设备编号
  *           buff：存放待写入数据的缓冲区指针
  *           sector：扇区地址(LBA)
  *           count：扇区数目(1..128)
  * 返 回 值: DSTATUS：操作结果
  * 说    明: 无
  */
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	        /* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
  DRESULT res;
 
  res = disk.drv[pdrv]->disk_read(disk.lun[pdrv], buff, sector, count);
  return res;
}

/**
  * 函数功能: 将缓冲区数据写入到物理设备内
  * 输入参数: pdrv：物理设备编号
  *           buff：存放待写入数据的缓冲区指针
  *           sector：扇区地址(LBA)
  *           count：扇区数目
  * 返 回 值: DSTATUS：操作结果
  * 说    明: SD卡写操作没有使用DMA传输
  */
#if _USE_WRITE == 1
DRESULT disk_write (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count        	/* Number of sectors to write */
)
{
  DRESULT res;
  
  res = disk.drv[pdrv]->disk_write(disk.lun[pdrv], buff, sector, count);
  return res;
}
#endif /* _USE_WRITE == 1 */

/**
  * 函数功能: 输入输出控制操作(I/O control operation)
  * 输入参数: pdrv：物理设备编号
  *           cmd：控制命令
  *           buff：存放待写入或者读取数据的缓冲区指针
  * 返 回 值: DSTATUS：操作结果
  * 说    明: 无
  */
#if _USE_IOCTL == 1
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
  DRESULT res;

  res = disk.drv[pdrv]->disk_ioctl(disk.lun[pdrv], cmd, buff);
  return res;
}
#endif /* _USE_IOCTL == 1 */

/**
  * 函数功能: 获取实时时钟
  * 输入参数: 无
  * 返 回 值: 实时时钟(DWORD)
  * 说    明: 无
  */
__weak DWORD get_fattime (void)
{
  return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

