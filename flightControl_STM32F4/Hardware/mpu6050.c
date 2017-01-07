
#include "mpu6050.h"
#include "iic.h"
#include "delay.h"
#include "led.h"
#include "uart.h"


extern unsigned char mpu6050_err;
struct _sensor sensor;	
u8 BUF[16];       //???????
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(void)
{
	IIC_WriteRegister(mpu6050,PWR_MGMT_1,0x00);	 //??????
	delay_ms(1);
	IIC_WriteRegister(mpu6050,SMPLRT_DIV,0x07);	 //??????,???:0x07(125Hz)
	delay_ms(1);
	IIC_WriteRegister(mpu6050,CONFIG,0x02);	     //??????,???:0x06(5Hz)
	delay_ms(1);
	IIC_WriteRegister(mpu6050,GYRO_CONFIG,0x18); //??????????,???:0x18(???,2000deg/s) 
	delay_ms(1);
	IIC_WriteRegister(mpu6050,ACCEL_CONFIG,0x00);//?????????????????,???:0x01(???,2G,5Hz) 
	delay_ms(1);
	IIC_WriteRegister(mpu6050,MPU6050_RA_USER_CTRL,0x00);//?? MPU6050 ???AUX I2C????
	delay_ms(1);
	IIC_WriteRegister(mpu6050,0X37,0x02);//?????? 
	
  sensor.acc.origin.x=0;
	sensor.acc.origin.y=0;
	sensor.acc.origin.z=0;
	
}

void MPU6050_data_Read(void)//???,?????
{
   BUF[0]=IIC_ReadRegister(mpu6050,GYRO_XOUT_L);  	//???
   BUF[1]=IIC_ReadRegister(mpu6050,GYRO_XOUT_H);
   sensor.gyro.origin.x=(int16_t)(BUF[1]<<8 | BUF[0]);

   BUF[2]=IIC_ReadRegister(mpu6050,GYRO_YOUT_L);
   BUF[3]=IIC_ReadRegister(mpu6050,GYRO_YOUT_H);
   sensor.gyro.origin.y=(int16_t)(BUF[3]<<8 | BUF[2]);
   
   BUF[4]=IIC_ReadRegister(mpu6050,GYRO_ZOUT_L);
   BUF[5]=IIC_ReadRegister(mpu6050,GYRO_ZOUT_H);
   sensor.gyro.origin.z=(int16_t)(BUF[5]<<8 | BUF[4]);
   
   BUF[8]=IIC_ReadRegister(mpu6050,ACCEL_XOUT_L);  	//???
   BUF[9]=IIC_ReadRegister(mpu6050,ACCEL_XOUT_H);
   sensor.acc.origin.x=(int16_t)(BUF[9]<<8 | BUF[8]);//左移，变大
   
   BUF[10]=IIC_ReadRegister(mpu6050,ACCEL_YOUT_L);
   BUF[11]=IIC_ReadRegister(mpu6050,ACCEL_YOUT_H);
   sensor.acc.origin.y=(int16_t)(BUF[11]<<8 | BUF[10]);
   
   BUF[12]=IIC_ReadRegister(mpu6050,ACCEL_ZOUT_L);
   BUF[13]=IIC_ReadRegister(mpu6050,ACCEL_ZOUT_H);
   sensor.acc.origin.z=(int16_t)(BUF[13]<<8 | BUF[12]);
	 
	 
   //Send_data_int(sensor.gyro.origin.y,'x');
}

/**************************????********************************************
//?iic????????,???????
*******************************************************************************/
void MPU6050_Dataanl(void)
{
	//static int a=0;
	sensor.acc.histor.x=sensor.acc.origin.x;
	sensor.acc.histor.y=sensor.acc.origin.y;
	sensor.acc.histor.z=sensor.acc.origin.z;
	
	MPU6050_data_Read();
	
	sensor.acc.radian.x = (float)sensor.acc.histor.x*0.6f+(sensor.acc.origin.x- sensor.acc.quiet.x)*0.4f;
	sensor.acc.radian.y = (float)sensor.acc.histor.y*0.6f+(sensor.acc.origin.y- sensor.acc.quiet.y)*0.4f;
	sensor.acc.radian.z = (float)sensor.acc.histor.z*0.6f+(sensor.acc.origin.z)*0.4f;
	
	sensor.gyro.radian.x = (float)(sensor.gyro.origin.x - sensor.gyro.quiet.x)*0.0010636856f; //* 0.0005327;//???? =???-?????
	sensor.gyro.radian.y = (float)(sensor.gyro.origin.y - sensor.gyro.quiet.y)*0.0010636856f; //* 0.0005327;
	sensor.gyro.radian.z = (float)(sensor.gyro.origin.z - sensor.gyro.quiet.z)*0.0010636856f; //* 0.0005327; 
	  
	
}
/**************************????********************************************
//?????????
*******************************************************************************/
void MPU6050_Test(void)
{       float z_acc;
	unsigned char data1[9];
	mpu6050_err =0;
loop:
	
	data1[0] = IIC_ReadRegister(mpu6050,PWR_MGMT_1);
	
	data1[1] =IIC_ReadRegister(mpu6050,SMPLRT_DIV);
	
	data1[2] =IIC_ReadRegister(mpu6050,CONFIG);
	
	data1[3] =IIC_ReadRegister(mpu6050,GYRO_CONFIG);
	
	data1[4] =IIC_ReadRegister(mpu6050,ACCEL_CONFIG);
	
	data1[5] =IIC_ReadRegister(mpu6050,0x37);
	
	data1[6] =IIC_ReadRegister(mpu6050,WHO_AM_I);
	if((data1[0]==0x00)&&(data1[1]==0x07)&&(data1[2]==0x02)&&(data1[3]==0x18)&&(data1[4]==0x00)&&(data1[5]==0x02)&&(data1[6]==0x68))
	{
		mpu6050_err =0;
		data1[7] =IIC_ReadRegister(mpu6050,ACCEL_ZOUT_L);
    data1[8] =IIC_ReadRegister(mpu6050,ACCEL_ZOUT_H);
		z_acc = (((int)data1[8])<<8)|data1[7]; //Z????
		if(z_acc==0)
		{
			mpu6050_err =1;
			MPU6050_Init();
			goto loop;
		}
	}
	else
	{
		mpu6050_err =1;
		MPU6050_Init();
		goto loop;
	}
	
}


/**************************????********************************************
//???????
*******************************************************************************/

void Gyro_OFFEST(void)
{
   int cnt_g=2000;
	 double  tempgx=0,tempgy=0,tempgz=0;
	 sensor.gyro.averag.x=0;    //??????
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt_g--)       //????2000?   ???
	 {
	   MPU6050_data_Read();
     tempgx+= sensor.gyro.origin.x;
	   tempgy+= sensor.gyro.origin.y;
	   tempgz+= sensor.gyro.origin.z;
   }
	 sensor.gyro.quiet.x=(float)tempgx/2000;
	 sensor.gyro.quiet.y=(float)tempgy/2000;
	 sensor.gyro.quiet.z=(float)tempgz/2000;
	  
}
/**************************????********************************************
//????????
*******************************************************************************/
void MPU6050_OFFESET(void)
{
 double	tempax=0,tempay=0,tempaz=0;
 uint8_t cnt_a=200;
	
while(cnt_a--)
{ 
	MPU6050_data_Read();		
	tempax+= sensor.acc.origin.x;
	tempay+= sensor.acc.origin.y;
	tempaz+= sensor.acc.origin.z;
}	

	sensor.acc.quiet.x = (float)tempax/200;
	sensor.acc.quiet.y = (float)tempay/200;
	sensor.acc.quiet.z = (float)tempaz/200;

}
////解析
//void MPU6050_Dataanl(void)
//{
//	MPU6050_ACC_LAST.X=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - ACC_OFFSET.X;
//	MPU6050_ACC_LAST.Y=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - ACC_OFFSET.Y;
//	MPU6050_ACC_LAST.Z=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - ACC_OFFSET.Z;
//	//跳过温度ADC
//	MPU6050_GYRO_LAST.X=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - GYRO_OFFSET.X;
//	MPU6050_GYRO_LAST.Y=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - GYRO_OFFSET.Y;
//	MPU6050_GYRO_LAST.Z=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - GYRO_OFFSET.Z;
//	
//	while(!GYRO_OFFSET_OK)//陀螺仪 零偏计算
//	{
//		static int32_t tempgx=0,tempgy=0,tempgz=0;
//		static uint8_t cnt_g=0;
//		if(cnt_g==0)//刚进入 寄存器清零
//		{
//			GYRO_OFFSET.X=0;
//			GYRO_OFFSET.Y=0;
//			GYRO_OFFSET.Z=0;
//			tempgx = 0;
//			tempgy = 0;
//			tempgz = 0;
//			cnt_g = 1;
//			return;
//		}
//		//6050数据累加
//		tempgx+= MPU6050_GYRO_LAST.X;
//		tempgy+= MPU6050_GYRO_LAST.Y;
//		tempgz+= MPU6050_GYRO_LAST.Z;
//		if(cnt_g==200)//加两百次 求平均
//		{
//			GYRO_OFFSET.X=tempgx/cnt_g;
//			GYRO_OFFSET.Y=tempgy/cnt_g;
//			GYRO_OFFSET.Z=tempgz/cnt_g;
//			cnt_g = 0;
//			GYRO_OFFSET_OK = 1;//计算完成标志
//			return;
//		}
//		cnt_g++;
//	}
//	while(!ACC_OFFSET_OK)//加速度传感器数据 零偏数据计算
//	{
//		static int32_t	tempax=0,tempay=0,tempaz=0;
//		static uint8_t cnt_a=0;
//		if(cnt_a==0)
//		{
//			ACC_OFFSET.X = 0;
//			ACC_OFFSET.Y = 0;
//			ACC_OFFSET.Z = 0;
//			tempax = 0;
//			tempay = 0;
//			tempaz = 0;
//			cnt_a = 1;
//			return;
//		}
//		tempax+= MPU6050_ACC_LAST.X;
//		tempay+= MPU6050_ACC_LAST.Y;
//		//tempaz+= MPU6050_ACC_LAST.Z;
//		if(cnt_a==200)
//		{
//			ACC_OFFSET.X=tempax/cnt_a;
//			ACC_OFFSET.Y=tempay/cnt_a;
//			ACC_OFFSET.Z=tempaz/cnt_a;
//			cnt_a = 0;
//			ACC_OFFSET_OK = 1;
//			return;
//		}
//		cnt_a++;		
//	}
//}

//void MPU6050_Read(void)
//{
//	//IIC_Read_nByte(devAddr,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
//}


