#ifndef __MPU6050_H_
#define __MPU6050_H_



//****************************************
// ����MPU6050�ڲ���ַ
//****************************************

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
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
#define	PWR_MGMT_1		0x6B	//��Դ����������ֵ��0x00(��������)
#define	WHO_AM_I		  0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define mpu6050       0x68   //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_INT_PIN_CFG			0x37

//typedef struct{
//		float X;
//		float Y;
//		float Z;}FLOAT_XYZ;

struct _float{
	  float x;
		float y;
		float z;};

//struct _int16{
//       int x;
//	     int y;
//	     int z;};		

struct _trans{
     struct _float origin;  //ԭʼֵ
	   struct _float averag;  //ƽ��ֵ
	   struct _float histor;  //��ʷֵ
	   struct _float quiet;   //��ֵ̬
	   struct _float radian;  //����ֵ 
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };
extern struct _sensor sensor;					
void MPU6050_Init(void);
void MPU6050_Test(void);
void MPU6050_data_Read(void);//???,?????
void MPU6050_Dataanl(void);
void InitMPU6050(void);					    //��ʼ��MPU6050

void MPU6050_OFFESET(void);
void Gyro_OFFEST(void);

#endif // __MPU6050_H__