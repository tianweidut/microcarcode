//*************************************************************************
//*																								                        *
//*        **********************���Դ�ӡ��������*************************        *
//*	�ļ�˵�������ԣ��������																							                        *
//*************************************************************************

#ifndef __DEBUG_H__
#define __DEBUG_H__

//��ӡ����
/*
#ifdef	DEBUG_SWITCH	// if want to debug, define it while compile this function
#define DEBUG(str, args...)	printf(" " str, ## args)
#else
#define DEBUG(str, args...) ( )
#endif
*/

//�������鼰���ر���
extern unsigned char curLedValue;
extern unsigned char lastLedValue;

extern int isDebug;
extern int curSuffix;
extern int lastSuffix;
extern int printNow;

extern int pushSw;	//������:ȡ�����жϣ���ӡ���ݿ���
extern int isStop;
extern int isStart;


void getData(unsigned char ledValue,int debug); 
void print();//�ú����в��ܵ���handle(),��Ϊֻ��handle()���ܵ��ô˺���
char * getInfo(int id,int dir);	//��Ϣ��ӡ
void getLightLed(unsigned char led);	//��ӡ�Ƶ���Ϣ
short int getDegree_(int degree);
int getSpeed_(int speed);

#endif