
#include "delay.h"

static u8  fac_us = 0;//us延时倍乘数			   
static u16 fac_ms = 0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数

// 初始化时间。
void Systick_Init(void)
{
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
//	SysTick_Config(168);
	fac_us = 168/8;		//不论是否使用ucos,fac_us都需要使用
	fac_ms = (u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数  
}


void delay_us(u32 nus)
{		
	u32 temp;	 
  //SysTick_Config(168000);//1ms	
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}

void delay_xms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 

void delay_ms(u16 nms)
{	 	 
	u8 repeat=nms/540;
						
	u16 remain=nms%540;
	//SysTick_Config(168000);//1ms
	while(repeat)
	{
		delay_xms(540);
		repeat--;
	}
	if(remain)delay_xms(remain);
} 


void my_delay_us(u16 time)//此延时要根据cpu主频来定量
{    
  u16 i=0;  
  while(time--)
  {
    i=23;  //
    while(i--);    
  }
}
void my_delay_ms(u16 time)
{    
  u16 i=0;  
  while(time--)
  {
    i=28000;  
    while(i--) ;    
  }
}
//unsigned int nTime;
//void delay_1ms(unsigned int time)
//{
//	SysTick_Config(168000);//1ms
//	nTime = time;
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;
//	while(nTime)
//	{
//		
//	}
//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk ;
//}
//void delay_us(unsigned int time)
//{
//	SysTick_Config(168);//1us
//	nTime = time;
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;
//	while(nTime)
//	{
//		
//	}
//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk ;
//}
//void delay_ms(u16 time)
//{
//	SysTick_Config(168000);//1ms
//	nTime = time;
//	//SysTick->LOAD=nus*fac_us;
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;
//	while(nTime)
//	{
//		
//	}
//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk ;
//}
