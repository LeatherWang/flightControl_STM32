#ifndef __IIC_H
#define __IIC_H

#include "stm32f4xx.h"
#include "sys.h"

//IO��������	 
#define IIC_SCL    PEout(2)         // SCL
#define IIC_SDA    PBout(3)         // SDA	 
#define READ_SDA   PBin(3)          // ����SDA 

//IIC���в�������
void IIC_Init(void);                // ��ʼ��IIC��IO��				 
void IIC_Start(void);				        // ����IIC��ʼ�ź�
void IIC_Stop(void);	  			      // ����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			    // IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);// IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				      // IIC�ȴ�ACK�ź�
void IIC_Ack(void);					        // IIC����ACK�ź�
void IIC_NAck(void);				        // IIC������ACK�ź�

uint8_t IIC_WriteRegister(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);	
uint8_t IIC_ReadRegister(unsigned char SlaveAddress,unsigned char REG_Address);	

#endif

//------------------End of File----------------------------
