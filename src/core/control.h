//*************************************************************************
//*																								                        *
//*        **********************车辆自动控制程序*************************        *
//*	文件说明：赛道控制（核心代码）																							                        *
//*************************************************************************
#ifndef __CONTROL_H__
#define __CONTROL_H__

//通过等级获取速度，角度情况：此方法可以应用到光电编码测速的速度调整上
#define			st0				0
#define			st1				1
#define			st2				2
#define			st3				3
#define			st4				4
#define			st5				5
#define			st_1			-1
#define			st_2			-2
#define			st_3			-3
#define			st_4			-4
#define			st_5			-5

//st0
#define			speedLevel_st0	100
#define			midDegree_st0	0
//st1
#define			speedLevel_st1	95
#define			midDegree_st1	2
//st2
#define			speedLevel_st2	90
#define			midDegree_st2	10
//st3
#define			speedLevel_st3	85
#define			midDegree_st3	15
//st4
#define			speedLevel_st4	80
#define			midDegree_st4	27
//st5
#define			speedLevel_st5	70
#define			midDegree_st5	35

//转弯标准量定义
#define std0	0
#define std1	6
#define std2 	10
#define std3 	15
#define std4 	27
#define std_1	-5
#define std_2 	-10
#define std_3 	-15
#define std_4 	-27

extern int angleFlag;	//进入直角标志
//extern int choiceSpeedMax;
//int getChoiceSpeedMax(void);	//以后添加：通过外加button进行最高速限制

int get_speedLevel(int curSt);
int get_midDegree(int curSt);
int get_near_std(int cur_degree);

void straight_run();
void right_turn();
void left_turn();

void rightAngle1();
//void rightAngle();

void right_blackArea();
void left_blackArea();
void right_blackArea1();
void left_blackArea1();
void right_blackArea3();

void speedAdjust(int acc_left,int acc_right);	//直接从PWM定时器上进行速度变化


#endif