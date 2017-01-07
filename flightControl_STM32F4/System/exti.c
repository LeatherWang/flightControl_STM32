



#include "stm32f4xx_conf.h"
#include "exti.h"
#include "ov7670.h"
#include "delay.h"




void EXTI_Congfiguration(void)
{
	    EXTI_InitTypeDef EXTI_InitStructure;
	    NVIC_InitTypeDef NVIC_InitStructure;
	 
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); //开启系统设置时钟
	
	    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource10);
	    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	    EXTI_InitStructure.EXTI_LineCmd = ENABLE;   
	    EXTI_Init(&EXTI_InitStructure);
	
	    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			    //使能按键所在的外部中断通道
    	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	    //抢占优先级0 
    	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				   	//子优先级0 
    	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
    	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}


unsigned char ov_sta;
void EXTI15_10_IRQHandler(void)
{
	  if(EXTI_GetITStatus(EXTI_Line10  )==SET)//是8线的中断
	{     
		if(ov_sta<2)
		{
			if(ov_sta==0)
		 
			{
				OV7670_WRST=0;	 	//复位写指针	
        delay_us(50);				
				OV7670_WRST=1;	
				OV7670_WREN=1;		//允许写入FIFO
			}else 
			{
				OV7670_WREN=0;		//禁止写入FIFO 
				OV7670_WRST=0;	 	//复位写指针		  		 
				OV7670_WRST=1;	
			}
			ov_sta++;
		}
	}
 
	  EXTI_ClearITPendingBit(EXTI_Line10);  //清除EXTI8线路挂起位	
}
















