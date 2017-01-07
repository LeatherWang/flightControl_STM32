#ifndef __IMU_H
#define	__IMU_H

#define RtA 		57.324841f		//  180/3.1415  �Ƕ��� ת��Ϊ������		
#define AtR    	0.0174533f		//  1/RtA             RtA����		
#define Acc_G 	0.0011963f		//  1/32768/4/9.8     ���ٶ�����Ϊ4G		
#define Gyro_G 	0.03051756f	//  1/32768/1000      ����������Ϊ +��1000			
#define Gyro_Gr	0.0005327f   //  1/32768/1000/57.3 

struct _angle{
        float pitch;
        float roll;
        float yaw;};


extern struct _angle angle;
				
extern float qa0, qa1, qa2, qa3;

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);




#endif













