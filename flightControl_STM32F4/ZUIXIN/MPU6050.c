#include "MPU6050.h"
#include "IIC.h"
#include "delay.h"

extern unsigned char mpu6050_err;
struct _sensor sensor;	
unsigned char BUF[16];       //???????
//**************************************
//???MPU6050
//**************************************
void MPU6050_Init(void)
{
	IIC_WriteRegister(mpu6050,PWR_MGMT_1,0x00);	 //??????
	delay_ms(10);
	IIC_WriteRegister(mpu6050,SMPLRT_DIV,0x07);	 //??????,???:0x07(125Hz)
	delay_ms(10);
	IIC_WriteRegister(mpu6050,CONFIG,0x02);	     //??????,???:0x06(5Hz)
	delay_ms(10);
	IIC_WriteRegister(mpu6050,GYRO_CONFIG,0x18); //??????????,???:0x18(???,2000deg/s) 
	delay_ms(10);
	IIC_WriteRegister(mpu6050,ACCEL_CONFIG,0x00);//?????????????????,???:0x01(???,2G,5Hz) 
	delay_ms(10);
	IIC_WriteRegister(mpu6050,MPU6050_RA_USER_CTRL,0x00);//设置 MPU6050 是否为AUX I2C线的主机
	delay_ms(10);
	IIC_WriteRegister(mpu6050,MPU6050_INT_PIN_CFG,0x02);//?????? 

	
  sensor.acc.origin.x=0;
	sensor.acc.origin.y=0;
	sensor.acc.origin.z=0;
}
void MPU6050_data_Read(void)//???,?????
{
   BUF[0]=IIC_ReadRegister(mpu6050,GYRO_XOUT_L);  	//???
   BUF[1]=IIC_ReadRegister(mpu6050,GYRO_XOUT_H);
   sensor.gyro.origin.x=(int)(((int)BUF[1]<<8)|BUF[0]);

   BUF[2]=IIC_ReadRegister(mpu6050,GYRO_YOUT_L);
   BUF[3]=IIC_ReadRegister(mpu6050,GYRO_YOUT_H);
   sensor.gyro.origin.y=(int)(((int)BUF[3]<<8)|BUF[2]);
   
   BUF[4]=IIC_ReadRegister(mpu6050,GYRO_ZOUT_L);
   BUF[5]=IIC_ReadRegister(mpu6050,GYRO_ZOUT_H);
   sensor.gyro.origin.z=(int)(((int)BUF[5]<<8)|BUF[4]);
   
   BUF[8]=IIC_ReadRegister(mpu6050,ACCEL_XOUT_L);  	//???
   BUF[9]=IIC_ReadRegister(mpu6050,ACCEL_XOUT_H);
   sensor.acc.origin.x=(int)(((int)BUF[9]<<8)|BUF[8]);
   
   BUF[10]=IIC_ReadRegister(mpu6050,ACCEL_YOUT_L);
   BUF[11]=IIC_ReadRegister(mpu6050,ACCEL_YOUT_H);
   sensor.acc.origin.y=(int)(((int)BUF[11]<<8)|BUF[10]);
   
   BUF[12]=IIC_ReadRegister(mpu6050,ACCEL_ZOUT_L);
   BUF[13]=IIC_ReadRegister(mpu6050,ACCEL_ZOUT_H);
   sensor.acc.origin.z=(int)(((int)BUF[13]<<8)|BUF[12]);

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
	 int32_t  tempgx=0,tempgy=0,tempgz=0;
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
	 sensor.gyro.quiet.x=tempgx/2000;
	 sensor.gyro.quiet.y=tempgy/2000;
	 sensor.gyro.quiet.z=tempgz/2000;
	  
}
/**************************????********************************************
//????????
*******************************************************************************/
void MPU6050_OFFESET(void)
{
static int32_t	tempax=0,tempay=0,tempaz=0;
static uint8_t cnt_a=0;
	
while(cnt_a<=200)
{ 
	MPU6050_data_Read();		
	tempax+= sensor.acc.origin.x;
	tempay+= sensor.acc.origin.y;
	tempaz+= sensor.acc.origin.z;
				
				if(cnt_a==200)
				{
					sensor.acc.quiet.x = tempax/cnt_a;
					sensor.acc.quiet.y = tempay/cnt_a;
					sensor.acc.quiet.z = tempaz/cnt_a;
					//ACC_OFFSET_OK = 1;
					//EE_SAVE_ACC_OFFSET();//????
				}
				cnt_a++;			
}

}

