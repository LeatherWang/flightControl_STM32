#include "uart.h"
#include "stdio.h"
#include "delay.h"
#include "oled.h"

unsigned char a[11]={0};
unsigned char oled[8]={0};
unsigned char RX_BUF[8]={0};
extern unsigned char abc[6];
unsigned char RX_Num=0,start_flag=0,stop_flag=0;
int fly_rol,fly_pit,fly_oil,fly_yaw,fly_gao;
void USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
	
  USART_Cmd(USART2, ENABLE);
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//配置分组，2bit抢占，2bit响应
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

void USART2_Send1Byte(u8 byte)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, byte);
}

void USART2_SendNByte(u8 *data, u16 num)
{
	while(num--)
	{
		USART2_Send1Byte(*data++);
	}
}

void USART2_SendString(u8 *str)
{
	while(*str != '\0')
	{
		USART2_Send1Byte(*str++);
	}
}

void USART2_IRQHandler(void)
{
	unsigned char Res;
	int sum;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		Res = USART_ReceiveData(USART2);
		
	  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		if(Res == 's')
		{
			start_flag = 1;
			stop_flag = 0;
			RX_Num = 0;
		} 
		else if(start_flag == 1)
		{
			RX_BUF[RX_Num] = Res;
			//USART_SendData(USART2, RX_BUF[0]);
			RX_Num ++;
			if(RX_Num > 5) {RX_Num = 0;stop_flag = 1;start_flag = 0;}
		}
		
		if(stop_flag == 1)
		{
			sum = (RX_BUF[2] - 0x30)*1000+(RX_BUF[3]-0x30)*100+(RX_BUF[4]-0x30)*10+RX_BUF[5]-0x30;
	    if(RX_BUF[1] == '-')sum = -sum;     
	    switch(RX_BUF[0])
	    {
		    case  'r':{  
            				fly_rol = sum;break; 
			            }
        case  'p':{ 
					           fly_pit = sum; break;
			            }
		    case  'o':{ 
		                 fly_oil = sum; break;
			            }
	      case  'y':{ 
		                 fly_yaw = sum; break;
			            }
		    case  'g':{ 
				             fly_gao = sum; break;
				           }
		     default : break;
	    }
			stop_flag = 0;
		}
		//USART_SendData(USART2, Res);

	}
}
int fputc(int ch, FILE *f)
	{
	   /*regist*/
	   
	  // while(!(USART2->SR&(1<<6)));
	  // USART1->DR=ch;
     /*lib*/
// 		/* e.g. write a character to the USART */
	   USART_SendData(USART2, (uint8_t) ch);
//	
//	/* Loop until the end of transmission */
	   while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) ;	
	
	return ch;
	}
	
//用户自定义发送函数
void Send_data_float(float axis,unsigned char flog)
{	
	 int temp_data;
   int i;
	 a[0]=flog;
	 a[1]=' ';
  if(axis<0)
	{
	  axis=(-axis);
          a[2]='-';
	}
        else
	{
	  a[2]=' ';
	}
     temp_data=axis*100;
		 a[3]=temp_data/10000+0x30;
		 temp_data=temp_data%10000; 
		 a[4]=temp_data/1000+0x30;
		 temp_data=temp_data%1000;
     a[5]=temp_data/100+0x30;
		 a[6]='.';	
     temp_data=temp_data%100;	
     a[7]=temp_data/10+0x30;
     temp_data=temp_data%10;		
		 a[8]=temp_data+0x30;
     a[9]='\n';
		 for(i=0;i<10;i++)
		 {
		   USART_SendData(USART2, a[i]);
	     while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	
			 delay_us(1);
		 }
}
/**************************????********************************************
send int
*******************************************************************************/
void Send_data_int(int axis,unsigned char flog)
{	
	 int temp_data=0;
	 int i;
	 //unsigned char a[11]="00000000000";
	 //unsigned char a[9];
	 a[0]=flog;
	 a[1]=' ';
       if(axis<0)
	{
	  axis=(-axis);
          a[2]='-';
	}
        else
	{
	  a[2]=' ';
	}
		 temp_data=axis;
		 //a[3]=temp_data/100000+0x30;
		 //temp_data=temp_data%10000;
     a[3]=temp_data/10000+0x30;	
     temp_data=temp_data%10000;	
     a[4]=temp_data/1000+0x30;
     temp_data=temp_data%1000;	
     a[5]=temp_data/100+0x30;
		 temp_data=temp_data%100;
		 a[6]=temp_data/10+0x30;
		 temp_data=temp_data%10;
		 a[7]=temp_data+0x30;
    		 //a[8]=' ';
		 a[8]='\n';
		 for(i=0;i<9;i++)
		 {
		   USART_SendData(USART2, a[i]);
	     while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	
			 delay_us(1);
		 }
}

void Oled_data_int(int axis,unsigned char flog,unsigned char LIE)
{	
  int temp_data=0;
  if(axis<0)
	{
	  axis=(-axis);
    abc[0]='-';
	}
        else
	{
	  abc[0]=' ';
	}
		 temp_data=axis;
		 //a[3]=temp_data/100000+0x30;
		 //temp_data=temp_data%10000;
     abc[1]=temp_data/10000+0x30;	
     temp_data=temp_data%10000;	
     abc[2]=temp_data/1000+0x30;
     temp_data=temp_data%1000;	
     abc[3]=temp_data/100+0x30;
		 temp_data=temp_data%100;
		 abc[4]=temp_data/10+0x30;
		 temp_data=temp_data%10;
		 abc[5]=temp_data+0x30;
	   OLED_Show_FONT_6x8(flog,LIE,abc);
}
void Oled_data_float(float axis,unsigned char flog,unsigned char LIE)
{	
  int temp_data=0;
  if(axis<0)
	{
	  axis=(-axis);
          oled[0]='-';
	}
        else
	{
	  oled[0]=' ';
	}
		 temp_data=axis*100;
		 oled[1]=temp_data/10000+0x30;
		 temp_data=temp_data%10000; 
		 oled[2]=temp_data/1000+0x30;
		 temp_data=temp_data%1000;
     oled[3]=temp_data/100+0x30;
		 oled[4]='.';	
     temp_data=temp_data%100;	
     oled[5]=temp_data/10+0x30;
     temp_data=temp_data%10;		
		 oled[6]=temp_data+0x30;
	   OLED_Show_FONT_6x8(flog,LIE,oled);
}	 


