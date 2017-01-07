
#include "iic.h"

// IIC��ʱ
static void IIC_Delay() 
{
	volatile int i = 20;
	while(i--);
}

//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
 
	IIC_SCL = 1;
	IIC_SDA = 1;

}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	IIC_SDA = 1;	
	IIC_Delay();	
	IIC_SCL = 1;
	IIC_Delay();
 	IIC_SDA = 0; 
	IIC_Delay();
	IIC_SCL = 0; 
	IIC_Delay();
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	IIC_SCL = 0;
	IIC_Delay();
	IIC_SDA = 0;
 	IIC_Delay();
	IIC_SCL = 1; 
	IIC_Delay();
	IIC_SDA = 1;
	IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	IIC_SCL = 0;
	IIC_Delay();  
	IIC_SDA = 1;
	IIC_Delay();   
	IIC_SCL = 1;
	IIC_Delay();
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL = 0;
	IIC_Delay();
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL = 0;
	IIC_Delay();
	IIC_SDA = 0;
	IIC_Delay();
	IIC_SCL = 1;
	IIC_Delay();
	IIC_SCL = 0;
	IIC_Delay();
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL = 0;
	IIC_Delay();
	IIC_SDA = 1;
	IIC_Delay();
	IIC_SCL = 1;
	IIC_Delay();
	IIC_SCL = 0;
	IIC_Delay();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
  u8 t;       
	for(t=0;t<8;t++)
	{        
		IIC_SCL = 0;
		IIC_Delay();  		
		IIC_SDA=(txd&0x80)>>7;
		txd <<= 1; 	  
		IIC_Delay();  
		IIC_SCL = 1;
		IIC_Delay(); 
	}
	IIC_SCL = 0;
} 	

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA = 1;
	for(i=0;i<8;i++)
	{
		receive <<= 1;
		IIC_SCL = 0; 
		IIC_Delay();
		IIC_SCL = 1;
		IIC_Delay();
		if(READ_SDA)receive++;   
	}		
	IIC_SCL = 0; 	
	if (!ack)
		IIC_NAck();
	else
		IIC_Ack();  
	return receive;
}
// IICдһ���ֽ�����
uint8_t IIC_WriteRegister(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1);   
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(REG_Address);       
	IIC_Wait_Ack();	
	IIC_Send_Byte(REG_data);
	IIC_Wait_Ack();   
	IIC_Stop(); 
	return 0;
}

// IIC��1�ֽ�����
uint8_t IIC_ReadRegister(unsigned char SlaveAddress,unsigned char REG_Address)
{      		
	unsigned char REG_data; 
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1); 
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(REG_Address);     
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1 | 0x01);
	IIC_Wait_Ack();
	REG_data= IIC_Read_Byte(0);
	IIC_Stop();
	return REG_data;
}	
