#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "spi.h"
#include "w25qxx.h"
#include "key.h"  



//SPI通信接口实验-库函数版本
//STM32F4工程模板-库函数版本
//淘宝店铺：http://mcudev.taobao.com			
 
 
 
	
int main(void)
{ 
	u8 t;
	u8 len;	
	u16 times=0;  
	int H03=0;
	int H04=0;
	int H05=0;
	int angle=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200
	LED_Init();		  		//初始化与LED连接的硬件接口  
	W25QXX_CS=1;			//SPI FLASH
	SPI1_Init();		   			//SPI
	SPI1_SetSpeed(SPI_BaudRatePrescaler_128);	
	while(1)
	{
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			printf("\r\n发送数据:\r\n");
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);         //向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			}
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%5000==0)
			{
				printf("\r\n 6825 串口实验\r\n");
				printf("东莞奥凯特电子有限公司\r\n\r\n\r\n");
			}
			  W25QXX_CS=0;			//SPI
	      SPI1_ReadWriteByte(0x83);	//6825 Angle  command 
	      H03=SPI1_ReadWriteByte(0x00);	//6825???  H  
	      H04=SPI1_ReadWriteByte(0x00); //M
	      H05=SPI1_ReadWriteByte(0x00);	//L
        W25QXX_CS=1;	
			   
			  angle=(((H03&0x00ff)<<10)|((H04&0x00fc)<<2)|((H05&0x00f0)>>4))&0x3ffff;
         			
			//angle = SPI_RW();  //5040读角度
			if(times%200==0)printf("角度值H:%x\n",H03); //输出角度 
			if(times%200==0)printf("角度值M:%x\n",H04); //输出角度 
			if(times%200==0)printf("角度值L:%x\n",H05); //输出角度 
			if(times%200==0)printf("角度值:%d\n",angle); //输出角度 
			if(times%30==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
			delay_ms(10);   
		}
	}
}

