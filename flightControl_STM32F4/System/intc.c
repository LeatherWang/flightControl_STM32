#include "intc.h"
#include "stm32f4xx_it.h"
#include "uart.h"
#include "sys.h"
#include "timer.h"
#include "delay.h"
#include "MS5611.h"

volatile uint32_t Rise_time,Fall_time;//必须是32位的！！！！

//------------超声波驱动状态机--------------
#define Ultra_ST_Started   0x01	  //启动测距
#define Ultra_ST_RX1       0x02	  //收到回波1
#define Ultra_ST_RX2       0x03	  //收到回波2
#define Ultra_ST_Idle      0x04	  //休息一下
#define Ultra_Restart      0x1f	  //重新再来一次测距
#define Ultra_ST_Error     0xf2	  //错误

#define MOVAVG_SIZE  10	   //保存最近的10个数据 进行平均滤波

#define Max_Range          (uint32_t)5    //超声波最大量程  单位：米
#define Time_outed        (uint32_t)(Max_Range*2)*1000000/340  // 超时时间 单位us

volatile uint8_t Ultra_Stauts = Ultra_ST_Idle; //当前状态
volatile uint32_t Ultra_High_time = 0,
				   Ultra_Start_time = 0,	
				   Ultra_Low_time = 0;	
static  float Dist_buffer[MOVAVG_SIZE];
static uint8_t Dis_index = 0;
float  Ultra_Distance = 0 ;
int16_t Ultra_health = 0; 

//添加一个新的值到 队列 进行滤波
void Ultrasonic_NewDis(float val) 
{
  if(val > (float)Max_Range)
  	return;		
  Dist_buffer[Dis_index] = val;
  Dis_index = ((Dis_index + 1) % MOVAVG_SIZE);  
}

//读取队列 的平均值
float Ultrasonic_getAvg(float * buff, int size) {
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return (sum / (float)size);
}

void Ultrasonic_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;   //EXTI0_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void EXTIX_Init(void)
{
	uint8_t i;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//配置IO口
	Ultrasonic_NVIC_Config(); //中断设置
	//外部中断配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//输出模式
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//设置上下拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//Trig
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);//使能SYSCFG时钟
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource4);//PD0连接到外部中断0
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line4; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);  

	GPIO_ResetBits(GPIOB, GPIO_Pin_5);//先不要发超声波
	for(i=0;i<MOVAVG_SIZE;i++)
	{
				Dist_buffer[i]=0; //清队列，准备行动！
  }
}

//启动 发送超声波信号。
void Ultrasonic_Start(void)
{
   GPIO_SetBits(GPIOB, GPIO_Pin_5);
 	 my_delay_us(10);
	 GPIO_ResetBits(GPIOB, GPIO_Pin_5);
   Ultra_Stauts = Ultra_ST_Started;	 //改状态，
   Ultra_Start_time = micros();	//取时间
}

//超声波测距的线程， 此程序需要定时调用。以更新距离
void Ultrasonic_Routine(void)
{
	uint32_t temp;
	static uint32_t HaltTime = 0;
	static int16_t Ultra_valid = 0;
	static float  Distance;
	//计算在当前温度下 对应的空气中声音的传播速度
	/*
	音速与介质的密度和弹性性质有关，因此也随介质的温度、
	压强等状态参量而改变。气体中音速每秒约数百米，
	随温度升高而增大，0℃时空气中音速为331．4米／秒，
	15℃时为340米／秒，温度每升高1℃，音速约增加0．6米／秒。
	*/
	float  Sound_Speed ;
switch(Ultra_Stauts)//查询当前状态
{ 
 case Ultra_ST_Started: 	 //已经启动测量了
			temp = micros();
			if((temp - Ultra_Start_time)>Time_outed)
			{
			  Ultra_Stauts = Ultra_ST_Error;	 //超时了，错误
			}
			break;
 case Ultra_ST_RX1: 	//已经收到了高电平
			temp = micros();
			if((temp - Ultra_Start_time)>Time_outed)
			{
			  Ultra_Stauts = Ultra_ST_Error; //超时了，错误	
			}
			break;
 case Ultra_ST_RX2: //已经收到低电平。可以计算距离了
			Sound_Speed = (332.0f+ (MS5611_Temperature/100.0f)*0.607f);//计算声速
			Distance = (float)(Ultra_Low_time - Ultra_High_time);  //时间   单位 us
			Distance = (Distance/2000000.0f); //计算 声音走一半，需要的时间，单位 S
			Distance = (Distance) * Sound_Speed; //距离 

			if((Distance > (float)Max_Range) || (Distance < 0.0f))
		  {
				Ultra_Stauts = Ultra_ST_Error; //超时了，错误
				break; //超过量程了，这次测量无效。
			}
			Ultrasonic_NewDis(Distance);

			Ultra_Distance = Ultrasonic_getAvg(Dist_buffer,MOVAVG_SIZE);

			if(Ultra_valid++>MOVAVG_SIZE+2)//连续MOVAVG_SIZE次超声波高度有效，那么应该用它来修正气压高度的漂移
			{ 
				MS561101BA_SetAlt(Distance);  //超声波 高度有效。标定气压高度
				Ultra_valid = MOVAVG_SIZE;
			}
			Ultra_health++; //此次采集高度有效，健康度++
			if(Ultra_health > 100)Ultra_health = 100;
			if(Ultra_health > 90)	//当超声波高度计健康度达到90%时。使用
				Set_Ultra_Priority(1); //MS5611子程序
			else 
				Set_Ultra_Priority(0);

			Ultra_Stauts = Ultra_ST_Idle;
			break;
	case Ultra_ST_Error:
			Ultra_valid = 0;  //超声波高度有效计数 清零
			Ultra_Stauts = Ultra_ST_Idle; //进入休息状态
			Ultra_health--; //此次采集高度无效，健康度--
			if(Ultra_health < 0)Ultra_health = 0;
			if(Ultra_health < 90)Set_Ultra_Priority(0);//如果超声波突然失灵，稍等片刻使能气压计
				
			break;
	case Ultra_ST_Idle:		 //休息时间。我们不要采集那么快。
		 if(HaltTime == 0)
		 {
		 	HaltTime =	micros();
		 }
		 else
		 {
			temp =	micros();
			if((temp - HaltTime)>Time_outed)
			{
				Ultra_Stauts = Ultra_Restart; //休息够了，重新启动一次采集
				HaltTime = 0;
			}
		 }
		 break;
	case Ultra_Restart:	   //重启一次测距
		 Ultrasonic_Start();  //发送超声波信号
		 Ultra_Stauts = Ultra_ST_Started; // 改变状态，
		 break;
	default: 
		   Ultrasonic_Start();
		   Ultra_Stauts = Ultra_ST_Started;
		   break;
	}
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
	{
	 if( GPIOB->IDR & 0x0010)
	 {	//PD0   高电平否？
	  Ultra_High_time = micros();
	  Ultra_Stauts = Ultra_ST_RX1; //我们收到了高电平 状态机变化	
	 }
	 else
	 {		//低电平
	   Ultra_Low_time = micros();
	   Ultra_Stauts = Ultra_ST_RX2;	//采样结束  
	 }		
	EXTI_ClearITPendingBit(EXTI_Line4);
 }
}

/*void EXTI4_IRQHandler(void)
{
	float distance;
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
	{
	 if( GPIOB->IDR & 0x0010)
	 {	//PD0   高电平否？
	   Rise_time = micros();
	 }
	 else
	 {		//低电平
	   Fall_time = micros();
		 if(Fall_time <= Rise_time)
			{
				distance = (float)(0xffffffff - Rise_time + Fall_time);//pwmin_temp = (0xffffffff - CH5.RisingTime) + CH5.FallingTime;
				distance = distance/2000000.0f;
				chao_meter = 340.0f *distance;
		  } 
		  else
			{
	      distance = (float)(Fall_time - Rise_time); //340*32*0.001*0.5=5.44
				distance = distance/2000000.0f;
				chao_meter = 340.0f *distance;
			}
	 }		
	EXTI_ClearITPendingBit(EXTI_Line4);
 }
}*/

