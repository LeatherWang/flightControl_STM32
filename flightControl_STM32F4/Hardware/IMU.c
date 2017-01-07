/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * ???  :IMU.c
 * ??    :????         
 * ????:HT??
 * ???  :ST3.5.0
 * ??    :Air Nano Team 
**********************************************************************************/
#include "delay.h"
#include "IMU.h"
#include "math.h"
#include "MPU6050.h"

struct _angle angle;
//#define sampleFreq	250.0f			// sample frequency in Hz2.5 1.25
//#define twoKpDef	1.0f//(2.0f * 0.5f)	// 2 * proportional gain
//#define twoKiDef	0.5f //(2.0f * 0.0f)	// 2 * integral gain

//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
//volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
extern int fly_rl,fly_oil,fly_ud,fly_pid;
float roll_radian,pitch_radian;
extern float PIT,ROL,YAW;
//   快速求平方根倒数
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 
//  求float型数据绝对值
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}
/*   采用三角函数的泰勒展开式 求近似值*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}
/***********************************************
  * @brief  可变增益自适应参数
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.28f * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
/*************************************/
	
void Prepare_Data(void)
{
	MPU6050_Dataanl();          //读取6050的数据
	//HMC5883_Collect();   //读取地磁数据
	
}



/**************************************
 * ???:Get_Attitude
 * ??  :??????
 * ??  :?
 * ??  :?
 * ??  :????
 *************************************/
void Get_Attitude(void)
{       
	Prepare_Data();//????
	
	IMUupdate(sensor.gyro.radian.x,
		  sensor.gyro.radian.y,
		  sensor.gyro.radian.z,
		  sensor.acc.radian.x,
	    sensor.acc.radian.y,
	    sensor.acc.radian.z);	
        
}

//float vx, vy, vz;// wx, wy, wz;
#define Kp 10.0f                       // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.002f                   // half the sample period2é?ù?ü?úμ?ò?°?

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{       /*
	double Xr,Yr;
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	unsigned char ab[1]={'\n'};
        unsigned int temp_data;
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
		{
		// Normalise accelerometer measurement
		 recipNorm = Q_rsqrt(ax * ax + ay * ay + az * az);
		 ax *= recipNorm;
		 ay *= recipNorm;
		 az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		 halfvx = q1 * q3 - q0 * q2;
		 halfvy = q0 * q1 + q2 * q3;
		 halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		 halfex = (ay * halfvz - az * halfvy);
		 halfey = (az * halfvx - ax * halfvz);
	         halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) 
		{
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		 }
		else 
		{
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		 }

		// Apply proportional feedback
		 gx += twoKp * halfex;
		 gy += twoKp * halfey;
		 gz += twoKp * halfez;
	 }
	
	  // Integrate rate of change of quaternion
	  gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	  gy *= (0.5f * (1.0f / sampleFreq));
	  gz *= (0.5f * (1.0f / sampleFreq));
	  qa = q0;
	  qb = q1;
	  qc = q2;
	  q0 += (-qb * gx - qc * gy - q3 * gz);
	  q1 += (qa * gx + qc * gz - q3 * gy);
  	  q2 += (qa * gy - qb * gz + q3 * gx);
  	  q3 += (qa * gz + qb * gy - qc * gx); 
	
	  // Normalise quaternion
  	  recipNorm = Q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	  q0 *= recipNorm;
	  q1 *= recipNorm;
	  q2 *= recipNorm;
	  q3 *= recipNorm;*/
	  
  float norm;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // ?è°??aD?ó?μ?μ?μ??μ??o?
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
  if(ax*ay*az==0)
     return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //accêy?Y1éò??ˉ
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              1à????á|·??òoíá÷á?/±??¨
  vx = 2*(q1q3 - q0q2);												//???a???Dxyzμ?±íê?
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //?òá?ía?y?ú?à??μ?μ?2?·??íê??ó2?
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //???ó2???DD?y·?
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//???ó2?PIoó213￥μ?íó?Yò?￡??′213￥á?μ??ˉò?
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//?aà?μ?gzóéóú??óD1?2a????DD???y?á2úéú?ˉò?￡?±í??3?à′μ??íê??y·?×????ò×???

  // integrate quaternion rate and normalise						   //???a??μ??￠·?·?3ì
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  roll_radian = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1); // roll
  pitch_radian = asin(-2*q1*q3 + 2*q0*q2); // pitch
	
	/*          ????????????                       */    
	/*??  http://baike.baidu.com/view/1239157.htm?fr=aladdin */
	
	//Xr = X_HMC * COS(angle.pitch/AtR) + Y_HMC * SIN(-angle.pitch/AtR) * SIN(-angle.roll/AtR) - Z_HMC * COS(angle.roll/AtR) * SIN(-angle.pitch/AtR);
	//Yr = Y_HMC * COS(angle.roll/AtR) + Z_HMC * SIN(-angle.roll/AtR);
	
	 // angle.yaw = atan2((double)Y_HMC,(double)X_HMC) * RtA; // yaw 
	  angle.yaw += sensor.gyro.radian.z *0.229183118f;//*0.00000425474;//0.004*0.0010636856
	 // angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* RtA;
	  
	  angle.roll =roll_radian * RtA;//y?
	  angle.pitch =pitch_radian * RtA;//x?
	  
	
	//Send_data_float(angle.yaw,'Y');
	  //Send_data_float(sensor.gyro.radian.z,'z');
        //Send_data_float(sensor.gyro.radian.y ,'x');
	
	
	  // R_UART2_Send(ab,1);
	//  Send_data_float(ADNS_Data_x,'x');
	//Send_data_float(chao_meter,'r');
	//Send_data_int(fly_ud,'u');
}
















