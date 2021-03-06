
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "mpu6050.h"
#include "stdio.h"
#include "iic.h"
#include "oled.h"
#include "control.h"
#include "timer.h"
#include "string.h"
#include "led.h"
#include "moto.h"
#include "intc.h"
#include "pwmin.h"
#include "MS5611.h"

/*注意：PB3,PB4,PA13~PA15不能使用，用于JLINK，否则将无法下载调试*/
void RCC_Config(void);
void OLED_show(void);
unsigned char mpu6050_err;
extern u8  TIM5CH1_CAPTURE_STA; 	 	//捕获状态 	     	 	 	 
extern u32 	TIM5CH1_CAPTURE_VAL; //捕获值
long long temp=0; 
extern float MS5611_Pressure,MS5611_Altitude,MS5611_Temperature;
extern unsigned char Ctrol_flag,Ultrasonic_flag,MS5611BA_flag;
unsigned char show_angle = 1;
extern int fly_rol,fly_pit,fly_oil,fly_yaw,fly_gao;
unsigned char abc[6]={0};
extern float PIT,ROL,YAW,GAO;
//extern volatile uint32_t Rise_time,Fall_time;//测试超声波专用
int main()
{
	//RCC_Config();
	SystemInit();
	Systick_Init();//system tick timer，系统节拍定时器
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); //中断优先级分组	
	LED_Init();
	PCout(2)=0;
	
	OLED_Init();//PA8~12
	OLED_Show_FONT_6x8(1,1,"P:");
	OLED_Show_FONT_6x8(1,3,"O.00");
  OLED_Show_FONT_6x8(1,12,"R:");
	OLED_Show_FONT_6x8(1,14,"O.00");
	
	OLED_Show_FONT_6x8(2,1,"Y:");
	OLED_Show_FONT_6x8(2,3,"O.00");
	OLED_Show_FONT_6x8(2,12,"H:");
	OLED_Show_FONT_6x8(2,14,"O.00");
	
	OLED_Show_FONT_6x8(4,1,"THR:");
	OLED_Show_FONT_6x8(4,5,"O.00");
	OLED_Show_FONT_6x8(4,12,"GAO:");
	OLED_Show_FONT_6x8(4,16,"O.00");
	myPid_init();  //PID parqameter initalize
  IIC_Init();//PB6/PB7
	mpu6050_err = 1;
	MPU6050_Init();			//6050 initalize
	MPU6050_Test();			//TEST initalize
	
  MPU6050_OFFESET(); //加速度计零漂校准
	Gyro_OFFEST();  //陀螺仪零漂校准
	
  MS561101BA_init();//初始化气压计
	//MS5611_OFFEST();//气压计校准
  Moto_Init();//PWM初始化,PD12~15
	//OLED_Show_FONT_6x8(2,14,"complete");
	
  USART2_Init();//PA2/PA3，优先级1，1,TX,RX
	EXTIX_Init();//超声波，优先级0、0,最高
	Initial_System_Timer();//用于计时、超声波、滤波都要用
	TIM2_Init();//主定时器1、2，第二高	
	
  //Tim_Pwm_In_Init();//PB0,PB1,PB4,PB5,TIM3

 //Moto_PwmRflash(1400,1400,1400,1400);
	while(1)
	{	//Send_data_float(chao_meter,'c');
		
		if(Ctrol_flag == 1)
		{
			PCout(2)=~PCout(2);
		  Get_Attitude();           //姿态结算
//			ROL = (float) fly_rol/12;
//		  PIT = (float) fly_pit/12;
//		  YAW = (float) fly_yaw/12;
		  GAO = (float) fly_gao/30;
		  ZHUKONG(PIT,ROL,YAW,GAO);	//主控函数
			OLED_show();
			Ctrol_flag = 0;
		}
		if(Ultrasonic_flag == 1)
		{
			Ultrasonic_Routine();
			Ultrasonic_flag = 0;
		}
		if(MS5611BA_flag == 1)
		{
			MS5611BA_Routing();
			MS5611BA_flag = 0;
		}
		PCout(2)=~PCout(2);
		
		//printf("%c",'n');
    //printf("%f",chao_meter);
//	printf("%f",MS5611_Pressure);//测试超声波专用
//	printf("%s","  "); 
//	printf("%f",MS5611_Altitude);//测试超声波专用
//	printf("%s","  "); 
//	printf("%f",Ultra_Distance);
// 	printf("%c",'\n'); 
 	/*printf("%d",Rise_time);//测试超声波专用
	printf("%s","  "); 
	printf("%d",Fall_time);
	printf("%s","  "); 
	printf("%d",Fall_time-Rise_time);
  printf("%c",'\n');*/ 		

	}
}
void OLED_show(void)
{
	  if(show_angle == 1)
		{
		 Oled_data_float(angle.pitch,1,3);
		}
		else if(show_angle == 2)
		{
			Oled_data_float(angle.roll,1,14);
		}
		else if(show_angle == 3)
		{
		 Oled_data_float(angle.yaw,2,3);
		}
		else if(show_angle == 4)
		{
			Oled_data_float(MS5611_Altitude,2,15);			
		}
		else if(show_angle == 5)
		{
		  Oled_data_int(fly_oil,4,5);
		}
		else if(show_angle == 6)
		{
		  Oled_data_int(fly_gao,4,16);
			show_angle = 0;
		}
		
	 	show_angle++;
}
void RCC_Config(void)
 { 
	uint32_t           PLL_M;       
  uint32_t           PLL_N; 
  uint32_t           PLL_P; 
  uint32_t           PLL_Q; 

  RCC_DeInit();              //RCC寄存器初始化
  RCC_HSEConfig(RCC_HSE_ON);  //使用外部时钟
	 
  if (RCC_WaitForHSEStartUp() == SUCCESS) //等待外部时钟启动
  {
		PLL_M = 8; // PLL_VCO input clock = (HSE_VALUE or HSI_VALUE / PLL_M)，PLL_VCO input clock = 1MHz 
		PLL_N = 336;// PLL_VCO output clock = (PLL_VCO input clock) * PLL_N，PLL_VCO output clock = 336 MHz
		PLL_P = 2; //System Clock = (PLL_VCO output clock)/PLL_P ，SystemClock = 168 Mhz
		PLL_Q = 7; // /配置SD,USB时钟频率，默认
		
    RCC_PLLCmd(DISABLE);                    //启动PLL前应关闭PLL
		RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q); 
		RCC_PLLCmd(ENABLE);        //PLL时钟开启	
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){} //等待PLL时钟准备好
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while (RCC_GetSYSCLKSource() != 0x08){};
			
    RCC_HCLKConfig(RCC_SYSCLK_Div1);     //HCLK(AHB)时钟为系统时钟的1分频
    RCC_PCLK1Config(RCC_HCLK_Div4);     //PCLK1(APB1)时钟为HCLK时钟的4分频，则TIM2时钟为HCLK的4分频
    RCC_PCLK2Config(RCC_HCLK_Div2);     //PCLK2(APB2)时钟为HCLK时钟2分频
    //RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7); //PLL时钟配置，详见system_stm43f4xx.c� Line149
  }
	
 }



 
 
 
 
 
 
 
 
