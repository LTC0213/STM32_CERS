#ifndef _LORA_APP_H_
#define _LORA_APP_H_

/************************************************
 ALIENTEK 阿波罗STM32F429开发板
 ATK-LORA-01模块功能驱动
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com  
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

#include "sys.h"
#include "lora_cfg.h"

#define LORA_AUX  PIin(11)  //LORA模块状态引脚
#define LORA_MD0  PAout(4)  //LORA模块控制引脚

extern _LoRa_CFG LoRa_CFG;
extern u8 Lora_mode;

u8 LoRa_Init(void);
void Aux_Int(u8 mode);
void LoRa_Set(void);
void LoRa_SendData(void);
void LoRa_ReceData(void);
void LoRa_Process(void);
void Lora_Test(void);


#endif

