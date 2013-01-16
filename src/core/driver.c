//*************************************************************************
//*																								                        *
//*        **********************��������*************************        *
//*�ļ�˵����Ӳ���˿ڵ�һЩ����	ͬʱ����������Ķ��壨ע��																							                        *
//*************************************************************************
#include "includes.h"

//��������ز�������
unsigned long iPulesNumAll=0;		//��Ҫ�۲���ĿԽ�������
unsigned long  timerCnt=0;
unsigned long  speedPules = 0;

double  eCurSpeed = 0.0;		//ʵʱ�����ٶȱ���
double   abvSpeed	=0.0;

int speedBrake_var=0;
int crankFlag=0;

//*************************************************************************
//*			 *************************IO��ʼ��*************************		  *
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
//*			 *************************��ʱ����ʼ��***********************		  *
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
//*			 *************************����������ʼ��***********************		  *
//*************************************************************************
void encoderInit(void)	//������Ӳ����ʼ��
{
	//�������ʼ��
	//�Ĵ����˿ڷ����ʼ��
	//P8DDR:bit0-->0(���鷽��),IER(�ж�ʹ�ܼĴ���):bit0-->1(IRQ0,input)
	P8DDR = 0xff;
	P8DDR &= 0xfe;
	
	IER = 0x00;
	IER |= 0x01;	//IRQ0 ,enable

	//�жϳ�ʼ�����½��ز���
	ISCR = 0x01;	//�ߵ�ƽ��͵�ƽ�����Ǵ���
	ISR =	0xfe;		//�����־�Ĵ���
	//	IPRA =	;		//�ж����ȼ��Ĵ���
	
	//�����ж�

	//��������
	abvSpeed =(double)((eWheelGirth)/(ePulesNum*eSpeedTime));	//����ֱ������ֵ���
}

//*************************************************************************
//*			 *************************ϵͳ��ʼ��************************		  *
//*************************************************************************
void vInitialize(void)
{
 
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<IO�ڳ�ʼ��>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>       
	vIOPortInit();  
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<��ʱ����ʼ��>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
	vTimerInit();
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<����������ʼ��>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
	encoderInit();
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<�ٷ���ʼ��>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 	
	init_sci1( 0x00, 79 );              /* Initialization of SCI1      */
    set_ccr( 0x00 );                    /* Entire interrupt permission */
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<�����ʼ����Ϣ>>>>>>>>>>>>>>>>>>>>>>>>>>>>>        
	printf("\nInitialize is successful!");
  
}
