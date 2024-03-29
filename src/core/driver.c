//*************************************************************************
//*																								                        *
//*        **********************驱动程序*************************        *
//*文件说明：硬件端口的一些定义	同时包含光电编码的定义（注）																							                        *
//*************************************************************************
#include "includes.h"

//光电编码相关参数定义
unsigned long iPulesNumAll=0;		//需要观察数目越界的问题
unsigned long  timerCnt=0;
unsigned long  speedPules = 0;

double  eCurSpeed = 0.0;		//实时计算速度变量
double   abvSpeed	=0.0;

int speedBrake_var=0;
int crankFlag=0;

//*************************************************************************
//*			 *************************IO初始化*************************		  *
//*************************************************************************
void vIOPortInit(void)
{
	P1DDR = 0xff;
    P2DDR = 0xff;
    P3DDR = 0xff;
    P4DDR = 0xff;
    P5DDR = 0xff;
    P6DDR = 0xf0;                       /* DIP SW on CPU board          */
    P8DDR = 0xff;
    P9DDR = 0xf7;                       /* Communication Port           */
    PADDR = 0xff;
	
    PBDR  = 0xc0;
    PBDDR = 0xfe;                       /* Motor Drive Board Vol.3      */
    /* As P7 of the sensor board is an exclusive input, there are no input output settings. */
}
//*************************************************************************
//*			 *************************定时器初始化***********************		  *
//*************************************************************************
void vTimerInit(void)
{
	/* ITU0 Interrupt at every 1ms */
    ITU0_TCR = 0x23;
    ITU0_GRA = TIMER_CYCLE;
    ITU0_IER = 0x01;

    /* ITU3, 4 reset-synchronized PWM mode for right-left motor and servo */
    ITU3_TCR = 0x23;
    ITU_FCR  = 0x3e;
    ITU3_GRA = PWM_CYCLE;                   /* Setting of cycle         */
    ITU3_GRB = ITU3_BRB = 0;                /* PWM Setting of left  motor*/
    ITU4_GRA = ITU4_BRA = 0;                /* PWM Setting of right motor*/
    ITU4_GRB = ITU4_BRB = SERVO_CENTER;     /* PWM Setting of servo     */
    ITU_TOER = 0x38;

    /* Count start of ITU */
    ITU_STR = 0x09;//0x09;
}
//*************************************************************************
//*			 *************************光电编码器初始化***********************		  *
//*************************************************************************
void encoderInit(void)	//光电编码硬件初始化
{
	//光电编码初始化
	//寄存器端口方向初始化
	//P8DDR:bit0-->0(建议方法),IER(中断使能寄存器):bit0-->1(IRQ0,input)
	P8DDR = 0xff;
	P8DDR &= 0xfe;
	
	IER = 0x00;
	IER |= 0x01;	//IRQ0 ,enable

	//中断初始化：下降沿捕获
	ISCR = 0x01;	//高电平向低电平跳变是触发
	ISR =	0xfe;		//清楚标志寄存器
	//	IPRA =	;		//中断优先级寄存器
	
	//开启中断

	//参数计算
	abvSpeed =(double)((eWheelGirth)/(ePulesNum*eSpeedTime));	//可以直接用数值替代
}

//*************************************************************************
//*			 *************************系统初始化************************		  *
//*************************************************************************
void vInitialize(void)
{
 
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<IO口初始化>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>       
	vIOPortInit();  
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<定时器初始化>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
	vTimerInit();
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<光电编码器初始化>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
	encoderInit();
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<官方初始化>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 	
	init_sci1( 0x00, 79 );              /* Initialization of SCI1      */
    set_ccr( 0x00 );                    /* Entire interrupt permission */
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<输出初始化信息>>>>>>>>>>>>>>>>>>>>>>>>>>>>>        
	printf("\nInitialize is successful!");
  
}
