#ifndef _LORA_APP_H_
#define _LORA_APP_H_

/************************************************
 ALIENTEK ������STM32F429������
 ATK-LORA-01ģ�鹦������
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com  
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

#include "sys.h"
#include "lora_cfg.h"

#define LORA_AUX  PIin(11)  //LORAģ��״̬����
#define LORA_MD0  PAout(4)  //LORAģ���������

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

