#ifndef _CONTROL_H_
#define _CONTROL_H_

extern unsigned char Rense;
extern unsigned char si;
extern float sp;
extern float Piddeadband;//PID����
extern float angle_x;          //X��ĽǶ���Ϣ      ��λ�Ƕ�
extern float angle_y;          //Y��ĽǶ���Ϣ

struct motor{             //�ĵ����λ��ʽ��ֻ��λ���͵������,��Ҫ�ظ����÷����
  float a;
  float b;
  float c;
  float d;
};

struct MYPID{               //�����ɻ�������PID���Ű�Χ�����Ի��ǽ���һ��PID�ṹ��� 
    float kp;
    float ki;
    float kd;
    
    float outP;
    float outI;
    float outD;
    float output;
    float lastoutput;
    
    float desired;
    float error;
    float prevError;
    
    float integ;
    float iLimit;
    float deriv;
};

void myPid_init(void);
void ZHUKONG(float targetX,float targetY,float targetZ,float targetG);
///float pidUpdate(struct MYPID *mypid,float measured,float gyro_origin,float gyro_radian,float gyro_history);//pid������ֵ�����ٶ�ֵ


#endif
