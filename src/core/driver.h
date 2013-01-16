//*************************************************************************
//*																								                        *
//*        **********************驱动程序*************************        *
//*	文件说明：尽量将所有跟硬件有关的操作，变量都放在																							                        *
//*************************************************************************
#ifndef __DRIVES_H__
#define __DRIVES_H__


//*************************************************************************
//*        **********************PWM相关设定**********************        *
//*************************************************************************
#define         TIMER_CYCLE     3071    /* Timer cycle 1ms              */
                                        /* When it is to be used by f/8 */
                                        /* f / 8 = 325.5[ns]            */
                                        /* Therefore, TIMER_CYCLE       */
                                        /*          = 1[ms] / 325.5[ns] */
                                        /*          = 3072              */
#define         PWM_CYCLE       49151   /* PWM cycle 16ms               */
                                        /* Therefore, PWM_CYCLE         */
                                        /*         = 16[ms] / 325.5[ns] */
                                        /*         = 49152              */
#define         SERVO_CENTER    4466 //4366//4745  //2//4460   /* Center value of Servo        */
#define         HANDLE_STEP     26      /* 1 degree part value          */

//传感器掩码定义
#define		MASK4_4		0xff
#define		MASK3_3		0xe7 /* oooxxooo                     */

//*************************************************************************
//*			 *************************光电编码器的定义************************		  *
//*************************************************************************
//程序基本思路：在一定时间内记录脉冲数目，通过固定周长的脉冲数，通过公式实现速度的获取

#define ePulesNum	(200)		//旋转编码器的脉冲数
#define eSpeedNum   (20)		//定时时间的次数，每一次为1MS
#define eWheelGirth (125)		//车轮周长，单位为米(125mm)
#define eSpeedTime	(0.02)		//定时周期，50ms

extern unsigned long iPulesNumAll;		//需要观察数目越界的问题
extern unsigned long speedPules;
extern double  eCurSpeed ;		//实时计算速度变量
extern unsigned long  timerCnt;	//光电编码专用定时器
extern double  abvSpeed;

extern int speedBrake_var;	//brake time
extern int crankFlag;
//*************************************************************************
//*			 *************************系统初始化************************		  *
//*************************************************************************
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<系统初始化>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void vInitialize(void);

#endif