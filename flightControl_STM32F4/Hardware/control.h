#ifndef _CONTROL_H_
#define _CONTROL_H_

extern unsigned char Rense;
extern unsigned char si;
extern float sp;
extern float Piddeadband;//PID死区
extern float angle_x;          //X轴的角度信息      单位是度
extern float angle_y;          //Y轴的角度信息

struct motor{             //四电机的位置式，只有位置型调用最好,不要重复引用否侧打架
  float a;
  float b;
  float c;
  float d;
};

struct MYPID{               //基本飞机就是由PID团团包围的所以还是建立一个PID结构体吧 
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
///float pidUpdate(struct MYPID *mypid,float measured,float gyro_origin,float gyro_radian,float gyro_history);//pid、测量值、角速度值


#endif
