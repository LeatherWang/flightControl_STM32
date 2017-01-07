
#include "pwmin.h"
#include "stm32f4xx_it.h"
#include "uart.h"

u16 Rc_Pwm_In[10];

void Tim_Pwm_In_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);
//////////////////////////////////////////////////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
  
	TIM3->PSC = (168/2)-1;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
}
//u8  TIM5CH1_CAPTURE_STA=0; //����״̬       
//u32 TIM5CH1_CAPTURE_VAL; //����ֵ(TIM2/TIM5->32) 

void TIM3_IRQHandler(void)
{
	static u32 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	if(TIM3->SR & TIM_IT_CC1) 
	{
		TIM3->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM3->SR = ~TIM_FLAG_CC1OF;
		if(GPIOB->IDR & GPIO_Pin_4)
		{
			temp_cnt1 = TIM_GetCapture1(TIM3);
		}
		else
		{
			temp_cnt1_2 = TIM_GetCapture1(TIM3);
			if(temp_cnt1_2>=temp_cnt1)
				Rc_Pwm_In[0] = temp_cnt1_2-temp_cnt1;
			else
				Rc_Pwm_In[0] = 0xffff-temp_cnt1+temp_cnt1_2;
		}
	}
	if(TIM3->SR & TIM_IT_CC2) 
	{
		TIM3->SR = ~TIM_IT_CC2;
		TIM3->SR = ~TIM_FLAG_CC2OF;
		if(GPIOB->IDR & GPIO_Pin_5)
		{
			temp_cnt2 = TIM_GetCapture2(TIM3);
		}
		else
		{
			temp_cnt2_2 = TIM_GetCapture2(TIM3);
			if(temp_cnt2_2>=temp_cnt2)
				Rc_Pwm_In[1] = temp_cnt2_2-temp_cnt2;
			else
				Rc_Pwm_In[1] = 0xffff-temp_cnt2+temp_cnt2_2;
		}
	}
	if(TIM3->SR & TIM_IT_CC3) 
	{
		TIM3->SR = ~TIM_IT_CC3;
		TIM3->SR = ~TIM_FLAG_CC3OF;
		if(GPIOB->IDR & GPIO_Pin_0)
		{
			temp_cnt3 = TIM_GetCapture3(TIM3);
		}
		else
		{
			temp_cnt3_2 = TIM_GetCapture3(TIM3);
			if(temp_cnt3_2>=temp_cnt3)
				Rc_Pwm_In[2] = temp_cnt3_2-temp_cnt3;
			else
				Rc_Pwm_In[2] = 0xffff-temp_cnt3+temp_cnt3_2;
		}
	}
	if(TIM3->SR & TIM_IT_CC4) 
	{
		TIM3->SR = ~TIM_IT_CC4;
		TIM3->SR = ~TIM_FLAG_CC4OF;
		if(GPIOB->IDR & GPIO_Pin_1)
		{
			temp_cnt4 = TIM_GetCapture4(TIM3);
		}
		else
		{
			temp_cnt4_2 = TIM_GetCapture4(TIM3);
			if(temp_cnt4_2>=temp_cnt4)
				Rc_Pwm_In[3] = temp_cnt4_2-temp_cnt4;
			else
				Rc_Pwm_In[3] = 0xffff-temp_cnt4+temp_cnt4_2;
		}
	}
	/*
  if(TIM_GetITStatus(TIM5, TIM_IT_CC2) == SET)//CH1���񵽶���
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);//����жϱ�־λ
		if(GPIOA->IDR & GPIO_Pin_1)//PA1
		{
			temp_cnt2 = TIM_GetCapture2(TIM5);
		}
		else
		{
			temp_cnt2_2 = TIM_GetCapture2(TIM5);
			if(temp_cnt2_2>=temp_cnt2)
				Rc_Pwm_In[1] = temp_cnt2_2-temp_cnt2;
			else
				Rc_Pwm_In[1] = 0xffff-temp_cnt2+temp_cnt2_2;
		}
	}*/
}




/*
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//δ���� 
  {  
		if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//���
		{			
      if(TIM5CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ
       { 
 	        if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//��������ֵ
 	        { 
 	 	       TIM5CH1_CAPTURE_STA|=0X80; 	 	//ǿ�Ʋ������ 
 	 	       TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
 	        }
		      else TIM5CH1_CAPTURE_STA++;//����++
		   } 	  
	   }
  if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)//���������¼�
	{
     if(TIM5CH1_CAPTURE_STA&0X40) //�ڴ�֮ǰ�Ǹߵ�ƽ����ʱ�����½���  	 
      { 	   	 	 	 
         TIM5CH1_CAPTURE_STA|=0X80; //�������
         TIM5CH1_CAPTURE_VAL=TIM_GetCapture1(TIM5);//��ȡ��ǰ����ֵ 
         TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //���������ز���
      }
		 else    	 	 	 	 	 	//�ڴ�֮ǰ�ǵ͵�ƽ������һ��������
      { 
         TIM5CH1_CAPTURE_STA=0; //����״̬���㣬���¿�ʼ
         TIM5CH1_CAPTURE_VAL=0; 
         TIM5CH1_CAPTURE_STA|=0X40;//��ǲ���������
				 TIM_Cmd(TIM5,ENABLE );  //ʹ�ܶ�ʱ��5 
			   TIM_SetCounter(TIM5,0); //������ֵ����
				 TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);//�����½��ز���
			   TIM_Cmd(TIM5,ENABLE );  //ʹ�ܶ�ʱ��5 
      } 	 	     
 	 	      	     	 	 	 	 	    
   } 
 }
   TIM_ClearITPendingBit(TIM5,TIM_IT_Update|TIM_IT_CC1); //����жϱ�־λ
	*/
















