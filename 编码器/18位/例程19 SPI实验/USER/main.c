#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "spi.h"
#include "w25qxx.h"
#include "key.h"  



//SPIͨ�Žӿ�ʵ��-�⺯���汾
//STM32F4����ģ��-�⺯���汾
//�Ա����̣�http://mcudev.taobao.com			
 
 
 
	
int main(void)
{ 
	u8 t;
	u8 len;	
	u16 times=0;  
	int H03=0;
	int H04=0;
	int H05=0;
	int angle=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	uart_init(115200);	//���ڳ�ʼ��������Ϊ115200
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�  
	W25QXX_CS=1;			//SPI FLASH
	SPI1_Init();		   			//SPI
	SPI1_SetSpeed(SPI_BaudRatePrescaler_128);	
	while(1)
	{
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			printf("\r\n��������:\r\n");
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);         //�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			printf("\r\n\r\n");//���뻻��
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%5000==0)
			{
				printf("\r\n 6825 ����ʵ��\r\n");
				printf("��ݸ�¿��ص������޹�˾\r\n\r\n\r\n");
			}
			  W25QXX_CS=0;			//SPI
	      SPI1_ReadWriteByte(0x83);	//6825 Angle  command 
	      H03=SPI1_ReadWriteByte(0x00);	//6825???  H  
	      H04=SPI1_ReadWriteByte(0x00); //M
	      H05=SPI1_ReadWriteByte(0x00);	//L
        W25QXX_CS=1;	
			   
			  angle=(((H03&0x00ff)<<10)|((H04&0x00fc)<<2)|((H05&0x00f0)>>4))&0x3ffff;
         			
			//angle = SPI_RW();  //5040���Ƕ�
			if(times%200==0)printf("�Ƕ�ֵH:%x\n",H03); //����Ƕ� 
			if(times%200==0)printf("�Ƕ�ֵM:%x\n",H04); //����Ƕ� 
			if(times%200==0)printf("�Ƕ�ֵL:%x\n",H05); //����Ƕ� 
			if(times%200==0)printf("�Ƕ�ֵ:%d\n",angle); //����Ƕ� 
			if(times%30==0)LED0=!LED0;//��˸LED,��ʾϵͳ��������.
			delay_ms(10);   
		}
	}
}

