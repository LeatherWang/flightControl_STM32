
#include "delay.h"

static u8  fac_us = 0;//us��ʱ������			   
static u16 fac_ms = 0;//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

// ��ʼ��ʱ�䡣
void Systick_Init(void)
{
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
//	SysTick_Config(168);
	fac_us = 168/8;		//�����Ƿ�ʹ��ucos,fac_us����Ҫʹ��
	fac_ms = (u16)fac_us*1000;//��ucos��,����ÿ��ms��Ҫ��systickʱ����  
}


void delay_us(u32 nus)
{		
	u32 temp;	 
  //SysTick_Config(168000);//1ms	
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ���� 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}

void delay_xms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
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


void my_delay_us(u16 time)//����ʱҪ����cpu��Ƶ������
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
