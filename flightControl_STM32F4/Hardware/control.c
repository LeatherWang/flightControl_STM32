
#include "MPU6050.h"
#include "iic.h"
#include "delay.h"
#include "control.h"
#include "IMU.h"
#include "moto.h"
#include "MS5611.h"

float angle_x=0;          //X??????      ????
float angle_y=0;          //Y??????
float Piddeadband=0;          //pid??,????????
struct MYPID mypid_roll, mypid_pitch, pitch_core, roll_core, mypid_gao, mypid_yaw;//??roll?pitch?pid???
struct motor M_angleX;
struct motor M_angleY;
struct motor ALL;
extern int fly_oil;
//extern int fly_rl_quiet, fly_ud_quiet, fly_oil_quiet;
//extern uint8_t RX_BUF[8];
float THROTTLE=0;//??????
void myPid_init(void);
char updata_gao = 0,start_fei=0;
float PIT,ROL,YAW,GAO;
float gao_output;
float Ctrol_Altitude = 0;

void myPid_init(void)
{  
   //pitch
    mypid_pitch.kp=0.01;//0.01;0.046//0.06;//0.06;//0.049;//0.0052;//0.0052;//0.054;//0.058;//0.23;///0.28;//0.32;//0.4;//0.5;//1.2;
    mypid_pitch.ki=0;//mypid_pitch.kp/20;
    mypid_pitch.kd=0;//0.1;//.46;//0.45;//0.45;//0.40;//0.42;//0.25;0.16;//25;//30;//37
    mypid_pitch.outP=0;
    mypid_pitch.outI=0;
    mypid_pitch.outD=0;
    mypid_pitch.output=0;  
    mypid_pitch.lastoutput=0;    
    mypid_pitch.desired=0;//????????,????
    mypid_pitch.error=0;
    mypid_pitch.prevError=0;   
    mypid_pitch.integ=0;
    mypid_pitch.iLimit=200;//??????
    mypid_pitch.deriv=0;
    
    pitch_core.kp=1.2;//1.7;//0.70;
    pitch_core.ki=pitch_core.kp/50;//80
    pitch_core.kd=0;//0.7;//18;
    pitch_core.outP=0;
    pitch_core.outI=0;
    pitch_core.outD=0;
    pitch_core.output=0;  
    pitch_core.lastoutput=0; 
    pitch_core.integ=0;
    pitch_core.iLimit=200;
    pitch_core.error=0;
    pitch_core.prevError=0;
    
    //roll
    mypid_roll.kp=0.01;
    mypid_roll.ki=0;
    mypid_roll.kd=0;
    mypid_roll.outP=0;
    mypid_roll.outI=0;
    mypid_roll.outD=0;
    mypid_roll.output=0;
    mypid_roll.lastoutput=0;  
    mypid_roll.desired=0;
    mypid_roll.error=0;
    mypid_roll.prevError=0;
    mypid_roll.integ=0;
    mypid_roll.iLimit=200;//??????
    mypid_roll.deriv=0;	
    
    roll_core.kp=1.2;
    roll_core.ki=roll_core.kp/50;
    roll_core.kd=0;
    roll_core.outP=0;
    roll_core.outI=0;
    roll_core.outD=0;
    roll_core.output=0;  
    roll_core.lastoutput=0; 
    roll_core.integ=0;
    roll_core.iLimit=200;
    roll_core.error=0;
    
    mypid_gao.kp=0.71;
    mypid_gao.ki=0.042;//roll_core.kp/20;
    mypid_gao.kd=40;
    mypid_gao.outP=0;
    mypid_gao.outI=0;
    mypid_gao.outD=0;
    mypid_gao.output=0;  
    mypid_gao.lastoutput=0; 
    mypid_gao.desired=0;//1m
    mypid_gao.error=0;
    mypid_gao.integ=0;
    mypid_gao.iLimit=10000;//????
    
    mypid_yaw.kp=2;
    mypid_yaw.ki=0;//0.092727;///;0-1100
    mypid_yaw.kd=0;//0.9;//0.5;//0.5
    mypid_yaw.outP=0;
    mypid_yaw.outI=0;
    mypid_yaw.outD=0;
    mypid_yaw.output=0;  
    mypid_yaw.lastoutput=0; 
    mypid_yaw.desired=-0;//1m
    mypid_yaw.error=0;
    mypid_yaw.integ=0;
    mypid_yaw.iLimit=200;
}
float Get_MxMi(float num,float max,float min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}
void ZHUKONG(float targetX,float targetY,float targetZ,float targetG)
{
  float KAG=1;//?????????
  mypid_pitch.desired = targetX;
  mypid_roll.desired  = targetY;
  mypid_yaw.desired   = targetZ;
  mypid_gao.desired   = targetG;
  //outerloop
  mypid_pitch.error = mypid_pitch.desired - angle.pitch;//????p
  mypid_roll.error = mypid_roll.desired - angle.roll;
  mypid_pitch.outP = mypid_pitch.kp * mypid_pitch.error;
  mypid_roll.outP = mypid_roll.kp * mypid_roll.error;
  
  mypid_pitch.output = mypid_pitch.outP + mypid_pitch.outI + mypid_pitch.outD;
  mypid_roll.output = mypid_roll.outP + mypid_roll.outI + mypid_roll.outD;

  mypid_pitch.output = Get_MxMi(mypid_pitch.output,2.5,-2.5);//??????????
  mypid_roll.output = Get_MxMi(mypid_roll.output,2.5,-2.5);
  
  //core
  pitch_core.error = mypid_pitch.output * KAG - sensor.gyro.radian.y;
  roll_core.error = mypid_roll.output * KAG - sensor.gyro.radian.x;//radian
  
  pitch_core.integ += pitch_core.error;
  pitch_core.integ = Get_MxMi(pitch_core.integ,pitch_core.iLimit,-pitch_core.iLimit);
  roll_core.integ += roll_core.error;
  roll_core.integ = Get_MxMi(roll_core.integ,roll_core.iLimit,-roll_core.iLimit);
     
  pitch_core.outP = pitch_core.kp * pitch_core.error;
  pitch_core.outI = pitch_core.ki * pitch_core.integ;
  pitch_core.outD = pitch_core.kd * (pitch_core.error - pitch_core.prevError);
  pitch_core.output = pitch_core.outP + pitch_core.outI + pitch_core.outD;
  
  roll_core.outP = roll_core.kp * roll_core.error;
  roll_core.outI = roll_core.ki * roll_core.integ;
  roll_core.outD = roll_core.kd * (roll_core.error - roll_core.prevError);
  roll_core.output = roll_core.outP + roll_core.outI + roll_core.outD;
  
  //yaw
  mypid_yaw.error = mypid_yaw.desired - angle.yaw;//angle.yaw
  mypid_yaw.outP = mypid_yaw.kp * mypid_yaw.error;
  mypid_yaw.output = mypid_yaw.outP + mypid_yaw.outI + mypid_yaw.outD;
  mypid_yaw.output = Get_MxMi(mypid_yaw.output,100,-100);//????,??????

 updata_gao++;//?????,??32ms
 Ctrol_Altitude = MS5611_Altitude;
 if(Ctrol_Altitude < 0.0f) Ctrol_Altitude = 0;
 
 if(updata_gao == 8)//?????4ms????,????32ms????
 {
  mypid_gao.error = mypid_gao.desired - Ctrol_Altitude;//1m-100,0.7m-70,0m-0
  mypid_gao.integ += mypid_gao.error;
  mypid_gao.integ = Get_MxMi(mypid_gao.integ,mypid_gao.iLimit,0);//????  ??6000
  
  mypid_gao.outP = mypid_gao.kp * mypid_gao.error;
  mypid_gao.outI = mypid_gao.ki * mypid_gao.integ;//315
  mypid_gao.outD = mypid_gao.kd * (mypid_gao.error - mypid_gao.prevError);//
  
  mypid_gao.lastoutput = mypid_gao.output;//????
  mypid_gao.output = mypid_gao.outP + mypid_gao.outI + mypid_gao.outD;//????
  
  if((mypid_gao.output - mypid_gao.lastoutput)>50)//????					  
	 mypid_gao.output = mypid_gao.lastoutput+50;
  else if((mypid_gao.lastoutput - mypid_gao.output)>50)
         mypid_gao.output = mypid_gao.lastoutput-50;
	 
  mypid_gao.output = Get_MxMi(mypid_gao.output,500,0); //????  //???????,210????????i?????200,??????
  updata_gao =0;//????
 }
 gao_output = (float)mypid_gao.lastoutput + (mypid_gao.output - mypid_gao.lastoutput)*updata_gao/8.0f;//????
// THROTTLE = 2800;//????,??????
  //R_UART2_Receive(RX_BUF,8);////////
  
  if(Ctrol_Altitude < 10.0f)
    THROTTLE ++;//????????
  if(THROTTLE > 500)//????????????
    THROTTLE = 500;

  if (fly_oil > 200)//gao.output*4   //3045//80
  {
    ALL.a = 2700 + THROTTLE + fly_oil-3500 + 45*( roll_core.output + pitch_core.output) +  10*mypid_yaw.output + gao_output ; //+fllow_pwm; // mypid_gao.output; //+  (fly_rl - fly_rl_quiet) + (fly_ud - fly_ud_quiet);
    ALL.b = 2700 + THROTTLE + fly_oil-3500 + 45*(-roll_core.output + pitch_core.output) -  10*mypid_yaw.output + gao_output ; //+fllow_pwm; // mypid_gao.output;//+  (fly_rl - fly_rl_quiet) - (fly_ud - fly_ud_quiet);
    ALL.c = 2700 + THROTTLE + fly_oil-3500 + 45*( roll_core.output - pitch_core.output) -  10*mypid_yaw.output + gao_output ; //-fllow_pwm; // mypid_gao.output;//-  (fly_rl - fly_rl_quiet) + (fly_ud - fly_ud_quiet);  
    ALL.d = 2700 + THROTTLE + fly_oil-3500 + 45*(-roll_core.output - pitch_core.output) +  10*mypid_yaw.output + gao_output ; //-fllow_pwm; // mypid_gao.output;//-  (fly_rl - fly_rl_quiet) - (fly_ud - fly_ud_quiet);
   
    Moto_PwmRflash(ALL.a,ALL.b,ALL.c,ALL.d);//etflags == 
  }
  else 
  {
    Moto_PwmRflash(0,0,0,0);
    
    pitch_core.integ = 0;//clear error
    roll_core.integ = 0;
    mypid_gao.integ = 0;
    mypid_yaw.integ = 0;
    THROTTLE = 0;
  }
  
  pitch_core.prevError = pitch_core.error;//????
  roll_core.prevError = roll_core.error;
  mypid_gao.prevError = mypid_gao.error;
  mypid_yaw.prevError = mypid_yaw.error;
  
}



