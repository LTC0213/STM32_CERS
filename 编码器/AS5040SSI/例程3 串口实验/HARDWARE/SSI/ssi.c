#include "ssi.h"

#define CS_H  GPIOA->ODR|=(1<<2);
#define CS_L  GPIOA->ODR&=~(1<<2);
#define CLK_H  GPIOA->ODR|=(1<<1);
#define CLK_L  GPIOA->ODR&=~(1<<1);



void Delay_SPI(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

int SPI_RW()
{
	int i=0;
	int	wdata=0;
	CS_H;
	Delay_SPI(0xF);//72M 
	
	CS_L;
	Delay_SPI(0xF);
	for(i=0; i<16; i++)
	{
		CLK_H;
		Delay_SPI(0xF);
		if((GPIOA->IDR&0x00000001)>0)wdata|=1;  //DO
		wdata<<=1;
		CLK_L;
		Delay_SPI(0xF);//
	}
	
	return(wdata>>7);           	//Angle
}
