#include "timer.h"
#include "led.h"
#include "stm32f4xx_it.h"
#include "uart.h"
#include "stdio.h"
#include "delay.h"
#include "MS5611.h"
#include "intc.h"
//初始化TIM5 32位定时器，用于做系统的时钟。 
void Initial_System_Timer(void)
{
//  RCC->APB1ENR |= 0x0008;	//使能TIM5时钟
//	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
//	TIM5->CR2 = 0x0000;
//	TIM5->CNT = 0x0000;
//	TIM5->ARR = 0xFFFFFFFF;
//	TIM5->PSC = 84 - 1;	//分出 1M 的时钟 保证每个周期为1us
//	TIM5->EGR = 0x0001;
//	TIM5->CR1 |= 0x0001; //启动定时器
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  
  /**********************************************************
	168 000 000/168=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
  TIM_TimeBaseInitStructure.TIM_Period = 0xffffffff; //重载值，4ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84-1;  //分频系数
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分频，0
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); 
	TIM_Cmd(TIM5, ENABLE);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//DISABLE
//	NVIC_Init(&NVIC_InitStructure);
}
void TIM2_Init(void)//主定时器->姿态解算
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  
  /**********************************************************
	168 000 000/168=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
  TIM_TimeBaseInitStructure.TIM_Period = 4000-1; //重载值，4ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84-1;  //分频系数
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分频，0
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
	TIM_Cmd(TIM2, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//DISABLE
	NVIC_Init(&NVIC_InitStructure);
}
unsigned char Ctrol_flag=0,Ultrasonic_flag=0,MS5611BA_flag =0;
unsigned char ms1=0,ms2=0;
//如果中断子函数大于定时器时间，如果中断子程序在开头清中断标志，则在推出后继续进入中断，
//在中断子程序末尾清除，则下次中断丢失！！
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		
		//TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		ms1++;
		ms2++;
		if(ms2 == 8)
		{
			//Ultrasonic_Routine();//开启一次超声波测距，此代码一定要放在姿态解算的前面！！！！
			Ultrasonic_flag = 1;
			ms2 = 0;
		}
		if(ms1 == 3)
		{
		  //MS5611BA_Routing();
			MS5611BA_flag = 1;
			ms1=0;
		}
		Ctrol_flag = 1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
//如果中断子函数大于定时器时间，如果中断子程序在开头清中断标志，则在推出后继续进入中断，
//在中断子程序末尾清除，则下次中断丢失！！
//unsigned char chaoTrig_flag;
//void TIM3_IRQHandler(void)//超声波trig
//{
//	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
//	{
//		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//		chaoTrig_flag = 1;
//	}
//	
//}
//void TIM5_IRQHandler(void)
//{
//	;
//}


