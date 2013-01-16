//��������ٳ���
//author:tianwei dut
/*======================================*/
/* Include                              */
/*======================================*/
#include    <no_float.h>                /* Simplifies stdio, place at beginning */
#include    <stdio.h>
#include    <machine.h>
#include    "h8_3048.h"

/*======================================*/
/* Symbol definitions                   */
/*======================================*/
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
#define         SERVO_CENTER    4745	//4362//4460   /* Center value of Servo        */
#define         HANDLE_STEP     26      /* 1 degree part value          */

/* Mask value setting x: With Mask (Invalid) o:Without mask (Valid)     */
#define         MASK2_2         0x66    /* xooxxoox                     */
#define         MASK2_0         0x60    /* xooxxxxx                     */
#define         MASK0_2         0x06    /* xxxxxoox                     */
#define         MASK3_3         0xe7    /* oooxxooo                     */
#define         MASK0_3         0x07    /* xxxxxooo                     */
#define         MASK3_0         0xe0    /* oooxxxxx                     */
#define         MASK4_0         0xf0    /* ooooxxxx                     */
#define         MASK0_4         0x0f    /* xxxxoooo                     */
#define         MASK1_1         0x81    /* oxxxxxxo                     */
#define			MASK4_4			0xff


//�����벿��
//***********************************************************************************
/*======================================*/
/* encoder declaration                */
/*======================================*/
//�������˼·����һ��ʱ���ڼ�¼������Ŀ��ͨ���̶��ܳ�����������ͨ����ʽʵ���ٶȵĻ�ȡ

/************************************************************************/
//�������ʼ����������
//�������˼·����һ��ʱ���ڼ�¼������Ŀ��ͨ���̶��ܳ�����������ͨ����ʽʵ���ٶȵĻ�ȡ

#define ePulesNum	(200)		//��ת��������������
#define eSpeedNum   (100)		//��ʱʱ��Ĵ�����ÿһ��Ϊ1MS
#define eWheelGirth (125)		//�����ܳ�����λΪ��(125mm)
#define eSpeedTime	(0.1)		//��ʱ���ڣ�0.1s

unsigned long iPulesNumAll;		//��Ҫ�۲���ĿԽ�������

double  eCurSpeed = 0.0;		//ʵʱ�����ٶȱ���
unsigned long  timerCnt=0;
double   abvSpeed	=0.0;

double Ki_Speed = 0.5;
double Kp_Speed = 1;
double lastError = 0.0;

int speedControl(int expSpeed);	//�ٶȴ�����������ֵΪ�������ٶ�


//�жϷ����ӳ���
/*======================================*/
/* Prototype declarations               */
/*======================================*/
void init( void );
void eInit(void);	//�������ʼ������

void timer( unsigned long timer_set );
int check_crossline( void );
unsigned char sensor_inp( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
unsigned char startbar_get( void );
void led_out( unsigned char led );
void speed( int accele_l, int accele_r );
void handle( int angle );
char unsigned bit_change( char unsigned in );

/*======================================*/
/* Global variable declarations         */
/*======================================*/

/***********************************************************************************************/

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
#define			midDegree_st1	3
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

//ת���׼������
#define std0	0
#define std1	6
#define std2 	10
#define std3 	15
#define std4 	27
#define std_1	-5
#define std_2 	-10
#define std_3 	-15
#define std_4 	-27

	
const double length = 15.5;
const double width = 15.5;
const double tan[46] ={ 0, 0.017455,0.034921,0.052408,0.069927,0.087489,
						   0.105104,0.122785,0.140541,0.158384,0.176327,
						   0.194380,0.212557,0.230868,0.249328,0.267949,
						   0.286745,0.305731,0.324920,0.344328,0.363970,
						   0.383864,0.404026,0.424475,0.445229,0.466308,
						   0.487733,0.509525,0.531709,0.554309,0.577350,
						   0.600861,0.624869,0.649408,0.674509,0.700208,
						   0.726543,0.753554,0.781286,0.809784,0.839100,
						   0.869287,0.900404,0.932515,0.965689,1.000000};

unsigned long   cnt0;
unsigned long   cnt1;                  
unsigned long	cnt2;


//�������
int direction;
int turn_direction;
int blackArea_direction;

//���Ʊ���
int pattern = 0;	//״̬
int curSt;
int lastSt;

int degree;		//�Ƕ�
int lastDegree;

int speedLevel = 0;	//�ٶ�
int lastSpeed_l;
int lastSpeed_r;

int var;		//����

//my function 
void myInit(void);
void start();
void straight_run();
void right_turn();
void left_turn();
void rightAngle();
int check_blackArea(void);
void right_blackArea();
void left_blackArea();
int get_inner_speed(int degree,int speed_outer);
int get_speedLevel(int curSt);
int get_midDegree(int curSt);
void do_speed(int degree,int speed_outer);
int get_near_std(int cur_speed);
void print();
void getData(unsigned char var_,int debug);
char * getInfo(int id,int dir);
void stop();
short int getDegree_(int degree);
void getLightLed(unsigned char led);
int getSpeed_(int speed);
int getLightLedSum();



/************************************************************************/
/* Main program                                                         */
/************************************************************************/
void main( void )
{
    int     i=0 , ret;
	double  show=0.56;
    char    c;

    /* Initialize MCU functions */
    init();                             /* Initialize                   */
	eInit();
    init_sci1( 0x00, 79 );              /* SCI1 initialize              */
    set_ccr( 0x00 );                    /* Enable all interrupts        */


	while(!pushsw_get())
	{
		if(cnt1<100)
			led_out(0x1);
		else if(cnt1<200)
			led_out(0x2);
		else	cnt1 = 0;
	}

	handle(0);

	while(1)
	{

		printf("now the new encoder test\n");
		printf("the current speed is %d\n\n",(int)eCurSpeed);
		if(eCurSpeed > 50)
		{
			printf("now ,the current speed above 50\n");
			led_out(0x00);
		}
		else if(eCurSpeed>30)
		{
			printf("now ,the current speed is 15--30\n");

			led_out(0x00);
			led_out(0x01);
		}
		else if(eCurSpeed>15)
		{
			printf(" the speed is 0--15\n");

			led_out(0x00);
			led_out(0x02);
		}
		else 
		{
			printf("the speed 0-60\n");
			led_out(0x00);
			led_out(0x03);
		}

	}
	 
  
}

/************************************************************************/
/* Initialize H8/3048F-ONE on-chip peripheral functions                 */
/************************************************************************/
void init( void )
{
    /* Port I/O settings */
    P1DDR = 0xff;
    P2DDR = 0xff;
    P3DDR = 0xff;
    P4DDR = 0xff;
    P5DDR = 0xff;
    P6DDR = 0xf0;                       /* DIP switches on CPU board    */
    P8DDR = 0xff;
    P9DDR = 0xf7;                       /* Communication ports          */
    PADDR = 0xff;
    PBDR  = 0xc0;
    PBDDR = 0xfe;                       /* Motor drive board, Vol. 3    */
    /* Pin 7 on the sensor board is input-only, so no I/O setting is needed. */

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


//��ʱ�����仯:ÿ1ms����һ���жϽ���
//����������ʼ��
void eInit(void)
{
	//�Ĵ����˿ڷ����ʼ��
	//P8DDR:bit0-->0(���鷽��),IER(�ж�ʹ�ܼĴ���):bit0-->1(IRQ0,input)
	P8DDR = 0xff;
	P8DDR &= 0xfe;
	
	IER = 0x00;
	IER |= 0x01;	//IRQ0 ,enable

	//�жϳ�ʼ�����½��ز���
	ISCR = 0x01;	//�ߵ�ƽ��͵�ƽ�����Ǵ���
	ISR =	0xfe;		//�����־�Ĵ���	
	//�����ж�

	
	//�����ٶ�ʱ��ϵ��
	abvSpeed =(double)((eWheelGirth)/(ePulesNum*eSpeedTime));

}

#pragma interrupt( interrupt_capture )
void interrupt_capture( void )////////////////////////////////////////////
{
	ISR &=	0xfe;		//�����־�Ĵ���				
	iPulesNumAll++;		//�������:
	//�˴���Ҫ������ĿԽ������
}

/************************************************************************/
/* ITU0 Interrupt process                                               */
/************************************************************************/
#pragma interrupt( interrupt_timer0 )
void interrupt_timer0( void )
{
    ITU0_TSR &= 0xfe;                   /* Flag clear                   */
	
	//�˴����speed��ȡʽ��
	timerCnt++;				//��ʱ������:����PLLֵ�����������ÿ�α仯��ʱ��ֵ
	cnt0++;
	cnt1++;


	if((eSpeedNum) == timerCnt)	//100,ʱ����
	{
		//��ʱ����һ���ٶȵļ���
		eCurSpeed=(double)((iPulesNumAll*abvSpeed)/10);	//�ٶȼ��㹫ʽ:
															//				      ������     ��  �ܳ�
															//		ʵʱ�ٶ�=---------------------------
															//					һȦ�������� �� �ɼ�ʱ��
		//��λ
		iPulesNumAll = 0;	//��λ������Ŀ
		timerCnt = 0;		//��λʱ����Ŀ
		
	}//if
	
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//��ʱ�����κ���:Ӧ����ֱ�߲��ٵĸ�������
/////////////////////////////////////////////////////////////////////////////////////////////////////
void led_out( unsigned char led )
{
    unsigned char data;

    led = ~led;
    led <<= 6;
    data = PBDR & 0x3f;
    PBDR = data | led;
}
//�������
void handle( int angle )
{
	int i;
	degree = angle;

	
	if(angle>44)
		angle = 44;
	else if(angle<-44)
		angle = -44;
		
	if(angle==lastDegree) return;
	else
	{
		if(angle-lastDegree>4)
			for(i=lastDegree;i<angle;i+=3)
				ITU4_BRB = SERVO_CENTER - i * HANDLE_STEP;
		else if(angle-lastDegree<-4)
			for(i=lastDegree;i>angle;i-=3)
				ITU4_BRB = SERVO_CENTER - i * HANDLE_STEP;
		ITU4_BRB = SERVO_CENTER - angle * HANDLE_STEP;
		lastDegree = angle;
	}
		
}
//�ٶȺ���
void speed( int accele_l, int accele_r )
{
	unsigned char sw_data;
	unsigned long speed_max;
    sw_data= dipsw_get()+5;
	speed_max=(unsigned long)(PWM_CYCLE-1) * sw_data / 20;
    if(accele_l>100)
		accele_l = 100;
	else if(accele_l<-100)
		accele_l = -100;
	if(accele_r>100)
		accele_r = 100;
	else if(accele_r<-100)
		accele_r = -100;
	
    /* Left motor */
    if( accele_l >= 0 ) {
        PBDR &= 0xfb;
        ITU3_BRB = (unsigned long)speed_max* accele_l / 100;
    } else {
        PBDR |= 0x04;
        accele_l = -accele_l;
        ITU3_BRB = (unsigned long)speed_max * accele_l / 100;
    }

    /* Right motor */
    if( accele_r >= 0 ) {
        PBDR &= 0xf7;
        ITU4_BRA = (unsigned long)speed_max* accele_r / 100;
    } else {
        PBDR |= 0x08;
        accele_r = -accele_r;
        ITU4_BRA = (unsigned long)speed_max * accele_r / 100;
    }	

}
//4�����ش���
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw  = ~P6DR;                        /* Reading of DIP switch */
    sw &= 0x0f;

    return  sw;
}

void straight_run()
{
	//ֱ�ߴ���
	switch( sensor_inp(MASK3_3) )
	{
		case 0x00:
					if(curSt!=st0)
					{
						lastSt = curSt;
						curSt = st0;	
					}						
					break;

		//right
		case 0x04: /* st1 */ 
					if(curSt!=st1)
					{
						lastSt = curSt;
						curSt = st1;	
					}					
					break;					
					
		case 0x06: /* st2 */
					if(curSt!=st2)
					{
						lastSt = curSt;
						curSt = st2;	
					}					
					break;
		case 0x02:					
		case 0x07:	
					if(check_crossline()){	pattern = 30;return;	}
				//	if(check_blackArea()){	pattern = 40;return;	}
									
					if(curSt!=st3)
					{
						lastSt = curSt;
						curSt = st3;
					}					
					break;
					
		case 0x03:
					if(check_crossline()){	pattern = 30;return;	}
				//	if(check_blackArea()){	pattern = 40;return;	}
										
					if(curSt!=st4) 
					{
						lastSt = curSt;
						curSt = st4;
					}
					if((bit_change(P7DR) )&0x18) return; //�������������
					else
					{
						turn_direction = 1; //����
						pattern = 20;
						return;
					}
		case 0x01:
					if(!(bit_change(P7DR) &0x18)) 
					{
						turn_direction = 1;
						pattern = 20;
						return;
					}
					break;

		//left
		case 0x20:
					if(curSt !=st_1)
					{
						lastSt = curSt;
						curSt = st_1;	
					}	
					break;
					
		case 0x60:
					if(curSt !=st_2)
					{
						lastSt = curSt;
						curSt = st_2;	
					}						
					break;
		case 0x40:
		case 0xe0:
					if(check_crossline()){	pattern = 30;return;	}
				//	if(check_blackArea()){	pattern = 40;return;	}
					
					if(curSt!=st_3)
					{
						lastSt = curSt;
						curSt = st_3;
					}					
					break;
					
		case 0xc0:
					if(check_crossline()){	pattern = 30;return;	}
				//	if(check_blackArea()){	pattern = 40;return;	}
										
					if(curSt!=st_4)
					{
						lastSt = curSt;
						curSt = st_4;
					}
					if(( bit_change(P7DR) )&0x18) return; //�������������
					else
					{
						turn_direction = -1; //����
						pattern = 20;
						return;
					}
		case 0x80:
					if(!(bit_change(P7DR) &0x18)) 
					{
						turn_direction = -1;
						pattern = 20;
						return;
					}
					break;
		case 0x81:
					if(!(bit_change(P7DR) &0x18))
					{
						if(curSt>0)	turn_direction = 1;
						else turn_direction = -1;
						pattern = 20;
						return;	
					}
					break;
		default:break;
	}
	
	degree = get_midDegree(curSt);
	speedLevel = get_speedLevel(curSt);
	
	handle(degree);
	
	lastSpeed_l=  speedControl(lastSpeed_l) ;
	lastSpeed_r= lastSpeed_l;
	speed(lastSpeed_l,lastSpeed_r);
//	do_speed(degree,speedLevel);

}
unsigned char sensor_inp( unsigned char mask )
{
    unsigned char sensor;
/*
    sensor  = P7DR;

    sensor  = bit_change( sensor );     // Bit replacement 
    sensor &= mask;
*/
	sensor = ~P7DR;
	sensor &= 0xef;
	if( sensor & 0x08 ) sensor |= 0x10;
	
	sensor &=mask;
	
    return sensor;
}
int check_crossline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
 /*   b = sensor_inp(MASK2_2);
    if( b==0x66 || b==0x64 || b==0x26 || b==0x62 || b==0x46 ) {
        ret = 1;
    }*/
	b = bit_change(P7DR);
	if((b&0xc0) && (b&0x3c) && (b&0x03))
		ret = 1;
    return ret;
}

char unsigned bit_change( char unsigned in )
{
    /*
	unsigned char ret;
    int i;
	
   for( i=0; i<8; i++ ) {
        ret >>= 1;                      // Right shift of return value 
        ret |= in & 0x80;               // Ret bit7 = in bit7           
        in  <<= 1;                      // Left shift of argument       
    }*/
	unsigned char sensor;
	sensor = ~in;
	sensor &=0xef;
	if( sensor & 0x08) sensor |= 0x10;
    return sensor; 
}

int get_speedLevel(int curSt)
{
	switch(curSt)
	{
		case st0:	return speedLevel_st0;
		case st1:
		case st_1:	return speedLevel_st1;
		case st2:
		case st_2:	return speedLevel_st2;
		case st3:
		case st_3:	return speedLevel_st3;
		case st4:
		case st_4:	return speedLevel_st4;
		case st5:
		case st_5:	return speedLevel_st5;

		default: return 40;//error 
	}
}

int get_midDegree(int curSt)
{
	switch(curSt)
	{
		case st0:	return midDegree_st0;

		case st1:	return midDegree_st1;
		case st_1:	return -1*midDegree_st1;

		case st2:	return midDegree_st2;
		case st_2:	return -1*midDegree_st2;

		case st3:	return midDegree_st3;
		case st_3:	return -1*midDegree_st3;

		case st4:	return midDegree_st4;
		case st_4:	return -1*midDegree_st4;

		case st5:	return midDegree_st5;
		case st_5:	return -1*midDegree_st5;

		default:return 20;
	}
}

unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~PBDR;                        /* Reading of port having a push switch */
    sw &= 0x01;

    return  sw;
}
void timer( unsigned long timer_set )
{
    cnt0 = 0;
    while( cnt0 < timer_set );
}

/************************************************************************/
//�ٶȱջ�������
//ʵ���ٶ������ֵ�Ķ�Ӧ��ϵ��V = 3S - 70(v��Ϊʵ���ٶȣ�sΪ����ٶ�)


int speedControl(int expSpeed)
{
	//	expSpeed �� eCurSpeed ��Ա�
	double error = 0.0;		//����¼ֵ
	double currentSpeed = ((eCurSpeed+70)/3);	//ת��Ϊ����ٶ�(0-100)
	int setSpeed;
	int speedMax = 100;
	int speedMin = 0;
	unsigned char mode_flag = 0;	//0:normal PI  1:speed up 2:speed down  
	int retSpeed;

	//PI�ٶȵ��ڣ����ϵ���ļ���
	error = expSpeed - currentSpeed;	//���ֵ
	setSpeed =(int) currentSpeed + Ki_Speed * error +Kp_Speed *(error - lastError);
	lastError = error;
	
	//���ٶȾ��ұ仯���жϣ�΢��ʱ��PI������ʱ��bang-bang����
	
	if(error > 30){mode_flag = 1; speedMax =100;}
	else if(error >20){mode_flag = 1; speedMax =95;}
	else if(error < -30){mode_flag = 2; speedMin =0;}
	else if(error < -20){mode_flag = 3; speedMin =5;}
	else {mode_flag = 0;}
		
	//״̬ѡ��
	switch(mode_flag)
	{
		case 0:	//����
				retSpeed = setSpeed;
				return retSpeed;	//ֱ�ӷ���PI��΢��ֵ
				break;
		case 1:	//����
				return speedMax;
				break;
		case 2:	//����
				return speedMin;
				break;

		default:
				return 0;
		break;
	}
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/