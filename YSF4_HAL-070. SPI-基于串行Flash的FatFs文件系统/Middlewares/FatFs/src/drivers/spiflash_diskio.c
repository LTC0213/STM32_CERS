/**
  ******************************************************************************
  * 文件名程: spiflash_diskio.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 串行FLASH与FatFS文件系统桥接函数实现
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
#include <string.h>
#include "ff_gen_drv.h"
#include "spiflash/bsp_spiflash.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define SPI_FLASH_REBUILD           1    /* 1:使能格式化串行Flash，0：禁用格式化串行Flash */
#define SPI_FLASH_SECTOR_SIZE    4096    /* 串行Flash扇区大小 */
#define SPI_FLASH_START_SECTOR   1792    /* 串行Flash文件系统FatFS偏移量 */
#define SPI_FLASH_SECTOR_COUNT   2304    /* 串行Flash文件系统FatFS占用扇区个数 */

/* 私有变量 ------------------------------------------------------------------*/
static __IO DSTATUS Stat = STA_NOINIT;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
DSTATUS SPIFLASH_initialize(BYTE);
DSTATUS SPIFLASH_status(BYTE);
DRESULT SPIFLASH_read(BYTE, BYTE*, DWORD, UINT);

#if _USE_WRITE == 1   /*如果允许写操作*/
  DRESULT SPIFLASH_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1   /* 如果输入输出操作控制 */
  DRESULT SPIFLASH_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

/* 定义串行FLASH接口函数 */
const Diskio_drvTypeDef  SPIFLASH_Driver =
{
  SPIFLASH_initialize,                   /* 串行FLASH初始化 */
  SPIFLASH_status,                       /* 串行FLASH状态获取 */
  SPIFLASH_read,                         /* 串行FLASH读数据 */
#if  _USE_WRITE == 1
  SPIFLASH_write,                        /* 串行FLASH写数据 */
#endif /* _USE_WRITE == 1 */
  
#if  _USE_IOCTL == 1
  SPIFLASH_ioctl,                         /* 获取串行FLASH信息 */
#endif /* _USE_IOCTL == 1 */
};

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 串行FLASH初始化配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
DSTATUS SPIFLASH_initialize(BYTE lun)
{
  uint32_t Flash_Id=0;
#if SPI_FLASH_REBUILD == 1
  static uint8_t startflag=1;
#endif
  Stat = STA_NOINIT;
  /* 初始化SPIFLASHIO外设 */
  MX_SPIFlash_Init();
  /* 获取串行FLASH状态 */

  Flash_Id=SPI_FLASH_ReadID();

  if(Flash_Id==SPI_FLASH_ID)
  {
#if SPI_FLASH_REBUILD == 1
      if(startflag)
      {
        SPI_FLASH_SectorErase(SPI_FLASH_START_SECTOR*SPI_FLASH_SECTOR_SIZE);
        startflag=0;
      }
#endif
    Stat &=~STA_NOINIT;
  }
  return Stat;
}

/**
  * 函数功能: 串行FLASH状态获取
  * 输入参数: lun : 不用
  * 返 回 值: DSTATUS：串行FLASH状态返回值
  * 说    明: 无
  */
DSTATUS SPIFLASH_status(BYTE lun)
{
  Stat = STA_NOINIT;

  if(SPI_FLASH_ReadID()==SPI_FLASH_ID)
  {
    Stat &= ~STA_NOINIT;
  }
  return Stat;
}

/**
  * 函数功能: 从串行FLASH读取数据到缓冲区
  * 输入参数: lun : 不用
  *           buff：存放读取到数据缓冲区指针
  *           sector：扇区地址(LBA)
  *           count：扇区数目
  * 返 回 值: DSTATUS：操作结果
  * 说    明: 无
  */
DRESULT SPIFLASH_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{ 
  sector+=SPI_FLASH_START_SECTOR;      
  SPI_FLASH_BufferRead(buff, sector*SPI_FLASH_SECTOR_SIZE, count*SPI_FLASH_SECTOR_SIZE);
  return RES_OK;
}

/**
  * 函数功能: 将缓冲区数据写入到串行FLASH内
  * 输入参数: lun : 不用
  *           buff：存放待写入数据的缓冲区指针
  *           sector：扇区地址(LBA)
  *           count：扇区数目
  * 返 回 值: DSTATUS：操作结果
  * 说    明: 无
  */
#if _USE_WRITE == 1
DRESULT SPIFLASH_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  uint32_t write_addr; 
  /* 扇区偏移7MB，外部Flash文件系统空间放在SPI Flash后面9MB空间 */
  sector+=SPI_FLASH_START_SECTOR;
  write_addr = sector*SPI_FLASH_SECTOR_SIZE;    
  SPI_FLASH_SectorErase(write_addr);
  SPI_FLASH_BufferWrite((uint8_t *)buff,write_addr,count*SPI_FLASH_SECTOR_SIZE);
  return RES_OK;
}
#endif /* _USE_WRITE == 1 */

/**
  * 函数功能: 输入输出控制操作(I/O control operation)
  * 输入参数: lun : 不用
  *           cmd：控制命令
  *           buff：存放待写入或者读取数据的缓冲区指针
  * 返 回 值: DSTATUS：操作结果
  * 说    明: 无
  */
#if _USE_IOCTL == 1
DRESULT SPIFLASH_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  
  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;
  
  /* 获取串行FLASH总扇区数目(DWORD) */
  case GET_SECTOR_COUNT :
    *(DWORD * )buff = SPI_FLASH_SECTOR_COUNT;	
    res = RES_OK;
    break;
  
  /* 获取读写扇区大小(WORD) */
  case GET_SECTOR_SIZE :
    *(WORD * )buff = SPI_FLASH_SECTOR_SIZE;
    res = RES_OK;
    break;
  
  /* 获取擦除块大小(DWORD) */
  case GET_BLOCK_SIZE :
    *(DWORD * )buff = 1;
    res = RES_OK;
    break;
  
  default:
    res = RES_PARERR;
  }
  
  return res;
}
#endif /* _USE_IOCTL == 1 */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
