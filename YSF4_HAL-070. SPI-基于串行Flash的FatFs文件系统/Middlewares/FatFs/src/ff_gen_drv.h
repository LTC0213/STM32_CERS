#ifndef __FF_GEN_DRV_H__
#define __FF_GEN_DRV_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "diskio.h"
#include "ff.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 存储设备输入输出操作函数类型定义 */
typedef struct
{
  DSTATUS (*disk_initialize) (BYTE);                           /* 初始化存储设备函数指针         */
  DSTATUS (*disk_status)     (BYTE);                           /* 获取存储设备状态函数指针       */
  DRESULT (*disk_read)       (BYTE, BYTE*, DWORD, UINT);       /* 从存储设备读取扇区数据函数指针 */
#if _USE_WRITE == 1                                            /* 当_USE_WRITE=1时才有效         */
  DRESULT (*disk_write)      (BYTE, const BYTE*, DWORD, UINT); /* 写数据到存储设备扇区内函数指针 */
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1                                            /* 当_USE_IOCTL=1时才有效         */
  DRESULT (*disk_ioctl)      (BYTE, BYTE, void*);              /* 输入输出控制操作函数指针       */
#endif /* _USE_IOCTL == 1 */
}Diskio_drvTypeDef;

/* 存储设备类型定义 */
typedef struct
{ 
  uint8_t                 is_initialized[_VOLUMES];            /* 存储设备初始化状态       */ 
  Diskio_drvTypeDef       *drv[_VOLUMES];                      /* 存储设备输入输出操作函数 */
  uint8_t                 lun[_VOLUMES];                       /* USB设备逻辑分区编号 */
  __IO uint8_t            nbr;                                 /* 存储设备在FatFS中设备编号 */
}Disk_drvTypeDef;

/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint8_t FATFS_LinkDriverEx(Diskio_drvTypeDef *drv, char *path, uint8_t lun);
uint8_t FATFS_LinkDriver(Diskio_drvTypeDef *drv, char *path);
uint8_t FATFS_UnLinkDriver(char *path);
uint8_t FATFS_LinkDriverEx(Diskio_drvTypeDef *drv, char *path, BYTE lun);
uint8_t FATFS_UnLinkDriverEx(char *path, BYTE lun);
uint8_t FATFS_GetAttachedDriversNbr(void);

#endif  /* __FF_GEN_DRV_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
