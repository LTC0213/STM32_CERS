#include "timer.h"
#include "usmart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//定时器中断驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

extern vu16 USART3_RX_STA;

TIM_HandleTypeDef TIM7_Handler; //定时器句柄 

//定时器7配置预装载周期值
void TIM7_SetARR(u16 period)
{
	 TIM7->CNT = 0;     //计数器清空
	 TIM7->ARR&=0x00;   //先清预装载周期值为0
	 TIM7->ARR|= period;//更新预装载周期值 
}

//通用定时器7中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器7!(定时器7挂在APB1上，时钟为HCLK/2)
void TIM7_Init(u16 arr,u16 psc)
{  
	__HAL_RCC_TIM7_CLK_ENABLE();                           //使能TIM7时钟
    TIM7_Handler.Instance=TIM7;                            //基本定时器7
    TIM7_Handler.Init.Prescaler=psc;                       //分频系数
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;      //向上计数器
    TIM7_Handler.Init.Period=arr;                          //自动装载值
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM7_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM7_Handler);                  //使能定时器7和定时器7更新中断：TIM_IT_UPDATE  

	HAL_NVIC_SetPriority(TIM7_IRQn,0,1);                   //设置中断优先级，抢占优先级0，子优先级1
	HAL_NVIC_EnableIRQ(TIM7_IRQn);                         //开启ITM7中断  	
}

//定时器7中断服务函数
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM7_Handler);
}

//回调函数，定时器中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 

    if(htim==(&TIM7_Handler))//定时器7中断服务函数调用(GSM调用)
    {
	   USART3_RX_STA|=1<<15;	           //标记接收完成
	   HAL_TIM_Base_Stop(&TIM7_Handler);//关闭定时器7
		
    }
	if(htim==(&TIM4_Handler))//定时器4中断服务函数调用(UAMART调用)
    {
        usmart_dev.scan();	                        //执行usmart扫描
        __HAL_TIM_SET_COUNTER(&TIM4_Handler,0);;    //清空定时器的CNT
        __HAL_TIM_SET_AUTORELOAD(&TIM4_Handler,100);//恢复原来的设置
    }
}

