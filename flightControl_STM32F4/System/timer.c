#include "timer.h"
#include "led.h"
#include "stm32f4xx_it.h"
#include "uart.h"
#include "stdio.h"
#include "delay.h"
#include "MS5611.h"
#include "intc.h"
//��ʼ��TIM5 32λ��ʱ����������ϵͳ��ʱ�ӡ� 
void Initial_System_Timer(void)
{
//  RCC->APB1ENR |= 0x0008;	//ʹ��TIM5ʱ��
//	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
//	TIM5->CR2 = 0x0000;
//	TIM5->CNT = 0x0000;
//	TIM5->ARR = 0xFFFFFFFF;
//	TIM5->PSC = 84 - 1;	//�ֳ� 1M ��ʱ�� ��֤ÿ������Ϊ1us
//	TIM5->EGR = 0x0001;
//	TIM5->CR1 |= 0x0001; //������ʱ��
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  
  /**********************************************************
	168 000 000/168=1M
	1000 000/2500=400Hz
	���Բ�����PWMΪ400Hz
	����Ϊ2.5ms����Ӧ2500�ļ���ֵ��1ms~2ms��Ӧ�ļ���ֵΪ1000~2000��
	**********************************************************/
  TIM_TimeBaseInitStructure.TIM_Period = 0xffffffff; //����ֵ��4ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84-1;  //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷ�Ƶ��0
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
void TIM2_Init(void)//����ʱ��->��̬����
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  
  /**********************************************************
	168 000 000/168=1M
	1000 000/2500=400Hz
	���Բ�����PWMΪ400Hz
	����Ϊ2.5ms����Ӧ2500�ļ���ֵ��1ms~2ms��Ӧ�ļ���ֵΪ1000~2000��
	**********************************************************/
  TIM_TimeBaseInitStructure.TIM_Period = 4000-1; //����ֵ��4ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84-1;  //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷ�Ƶ��0
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
//����ж��Ӻ������ڶ�ʱ��ʱ�䣬����ж��ӳ����ڿ�ͷ���жϱ�־�������Ƴ�����������жϣ�
//���ж��ӳ���ĩβ��������´��ж϶�ʧ����
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		
		//TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		ms1++;
		ms2++;
		if(ms2 == 8)
		{
			//Ultrasonic_Routine();//����һ�γ�������࣬�˴���һ��Ҫ������̬�����ǰ�棡������
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
//����ж��Ӻ������ڶ�ʱ��ʱ�䣬����ж��ӳ����ڿ�ͷ���жϱ�־�������Ƴ�����������жϣ�
//���ж��ӳ���ĩβ��������´��ж϶�ʧ����
//unsigned char chaoTrig_flag;
//void TIM3_IRQHandler(void)//������trig
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


