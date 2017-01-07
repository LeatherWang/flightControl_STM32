
#include "moto.h"
#include "stdio.h"
#include "stm32f4xx_tim.h"
int aa=2000;
void Moto_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	//uint16_t PrescalerValue = 0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	/* Compute the prescaler value */
  //PrescalerValue = (uint16_t) (SystemCoreClock / (40000*50*2)) - 1;//计算分频系数//SystemCoreClock=168 000 000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 40000-1;									//重装值     40000->20ms？
  TIM_TimeBaseStructure.TIM_Prescaler = 42-1;		//分频系数  ，84M
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
  TIM_OCInitStructure.TIM_Pulse = aa;//CCR1_Val;预装值
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	//输出极性:TIM输出比较极性高
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能TIMx在CCRx上的预装载寄存器
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = aa;//CCR2_Val;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = aa;//CCR3_Val;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = aa;//CCR4_Val;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);//自动重载
  TIM_Cmd(TIM4, ENABLE);
}

void Moto_PwmRflash(float MOTO1_PWM,float MOTO2_PWM,float MOTO3_PWM,float MOTO4_PWM)
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	
	if(MOTO1_PWM<Moto_PwmMin)	MOTO1_PWM = Moto_PwmMin;
	if(MOTO2_PWM<Moto_PwmMin)	MOTO2_PWM = Moto_PwmMin;
	if(MOTO3_PWM<Moto_PwmMin)	MOTO3_PWM = Moto_PwmMin;
	if(MOTO4_PWM<Moto_PwmMin)	MOTO4_PWM = Moto_PwmMin;
	

	TIM4->CCR1 = (uint32_t)MOTO1_PWM;
	TIM4->CCR2 = (uint32_t)MOTO2_PWM;
	TIM4->CCR3 = (uint32_t)MOTO3_PWM;
	TIM4->CCR4 = (uint32_t)MOTO4_PWM;
	
//	TIM_SetCompare1(TIM4,(uint32_t)(MOTO1_PWM));	//PD12		PWM01			C
//	TIM_SetCompare2(TIM4,(uint32_t)(MOTO2_PWM));	//PD13		PWM02			D
//	TIM_SetCompare3(TIM4,(uint32_t)(MOTO3_PWM));	//PD14		PWM03			A
//	TIM_SetCompare4(TIM4,(uint32_t)(MOTO4_PWM));	//PD15		PWM04			B
}
