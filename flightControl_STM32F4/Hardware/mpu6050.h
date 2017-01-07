#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"
#include "delay.h"

/*#define MPU6050_USE*/

#define	SMPLRT_DIV		0x19	//??????,???:0x07(125Hz)
#define	CONFIG			0x1A	//??????,???:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//??????????,???:0x18(???,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//?????????????????,???:0x01(???,2G,5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//????,???:0x00(????)
#define	WHO_AM_I		  0x75	//IIC?????(????0x68,??)
#define mpu6050       0x68   //IIC??????????,+1???
#define MPU6050_RA_USER_CTRL        0x6A		
				
typedef struct{
		float X;
		float Y;
		float Z;}FLOAT_XYZ;

struct _float{
	  float x;
		float y;
		float z;};

//struct _int16{
//       int x;
//	     int y;
//	     int z;};		

struct _trans{
     struct _float origin;  //???
	   struct _float averag;  //???
	   struct _float histor;  //???
	   struct _float quiet;   //???
	   struct _float radian;  //??? 
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };
extern struct _sensor sensor;
							
	
												
void MPU6050_Init(void);
void MPU6050_Dataanl(void);
void MPU6050_data_Read(void);//???,?????
void MPU6050_Test(void);

void MPU6050_OFFESET(void);
void Gyro_OFFEST(void);

#endif
