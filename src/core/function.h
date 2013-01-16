//*************************************************************************
//*																								                        *
//*        **********************���ܺ�������*************************        *
//*	�ļ�˵�����������ʺ���																							                        *
//*************************************************************************
#ifndef __FUNCTION_H__
#define __FUNCTION_H__



extern unsigned long cnt0;
extern unsigned long cnt1;
extern unsigned long cnt2;

extern int degree;
extern int lastDegree;
extern int speedLevel;
 
extern int blackArea_direction;	
extern int turn_direction;

extern int turnSpeed;		//���ʱ�ٶȿ��Ʊ���

extern int pattern;
extern int curSt;
extern int lastSt;
extern int lastSensor;


#define LEFT	-1	//�����壺1����-1����
#define RIGHT   1

//��������״̬
//#define Par_Straight	10
//#define Par_Turn		20
//#define Par_RightAngle	30
//#define Par_BlackArea	40
 

#define RESET	0	//ʱ�䣬�ٶȣ��Ƕȵȵ�


void timer(unsigned long timer_set);	//��ʱ���ӳ�

unsigned  char   sensor_inp(unsigned char mask);	//������
unsigned  char   bit_change(unsigned char in);		

unsigned char startbar_get(void);

unsigned char dipsw_get(void);	//������button
//#define dipsw_get()	((unsigned char)((~P6DR)&0x0f))
//unsigned char pushsw_get(void);
#define pushsw_get()	((unsigned char)((~PBDR)&0x01))
//unsigned led_out(unsigned char led);	//led
#define led_out(led)	PBDR =  (PBDR&0x3f)|((~led)<<6)

 int check_crossline(void);	//��������������˴��ĳɺ궨��
 int  check_blackArea(void);
//#define cur_sensor	((~P7DR)&0xef)	//����startbar״̬
//#define check_crossline()	((cur_sensor&0xc0)&&(cur_sensor&0x2c)&&(cur_sensor&0x03))?TRUE:FALSE


//�ٶȽǶȿ��ƣ����
void speed(int accele_l , int accele_r);
void handle(int angle);
void do_speed(int degree ,int speed_outer);	//����Ȧƥ���ٶ�
int get_inner_speed(int degree,int speed_outer);

void start(void);	//��������
void stop(void);

//�����Զ�����ʹ��
int getLightLedSum();	//������Ŀ��˫����


#endif