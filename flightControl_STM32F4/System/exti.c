



#include "stm32f4xx_conf.h"
#include "exti.h"
#include "ov7670.h"
#include "delay.h"




void EXTI_Congfiguration(void)
{
	    EXTI_InitTypeDef EXTI_InitStructure;
	    NVIC_InitTypeDef NVIC_InitStructure;
	 
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); //����ϵͳ����ʱ��
	
	    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource10);
	    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	    EXTI_InitStructure.EXTI_LineCmd = ENABLE;   
	    EXTI_Init(&EXTI_InitStructure);
	
	    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			    //ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
    	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	    //��ռ���ȼ�0 
    	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				   	//�����ȼ�0 
    	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
    	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}


unsigned char ov_sta;
void EXTI15_10_IRQHandler(void)
{
	  if(EXTI_GetITStatus(EXTI_Line10  )==SET)//��8�ߵ��ж�
	{     
		if(ov_sta<2)
		{
			if(ov_sta==0)
		 
			{
				OV7670_WRST=0;	 	//��λдָ��	
        delay_us(50);				
				OV7670_WRST=1;	
				OV7670_WREN=1;		//����д��FIFO
			}else 
			{
				OV7670_WREN=0;		//��ֹд��FIFO 
				OV7670_WRST=0;	 	//��λдָ��		  		 
				OV7670_WRST=1;	
			}
			ov_sta++;
		}
	}
 
	  EXTI_ClearITPendingBit(EXTI_Line10);  //���EXTI8��·����λ	
}
















