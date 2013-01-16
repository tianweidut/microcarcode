#include    <no_float.h>         
#include    <stdio.h>
#include    <machine.h>
#include    "h8_3048.h"

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
#define         SERVO_CENTER    4570   //3755   //enter value of Servo        */
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
#define eSpeedNum   (20)		//��ʱʱ��Ĵ�����ÿһ��Ϊ1MS
#define eWheelGirth (125)		//�����ܳ�����λΪ��(125mm)
#define eSpeedTime	(0.02)		//��ʱ���ڣ�0.1s
#define SampleTime (10)
long iPulesNumAll;		//��Ҫ�۲���ĿԽ�������

double  eCurSpeed = 0.0;		//ʵʱ�����ٶȱ���
unsigned long  timerCnt=0;
double   abvSpeed	=0.0;


int speedBrake_var=0;
int crankFlag=0;
//double Ki_Speed = 0.5;
//double Kp_Speed = 1;
//double lastError = 0.0;

int speedControl(int expSpeed);	//�ٶȴ�������������ֵΪ�������ٶ�



/*======================================*/
/* Prototype declaration                */
/*======================================*/
void init(void );
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

	
const double length = 17;
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
void right_blackArea3();
void left_blackArea3();

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


#define arrayLength 310

unsigned char pattern_[arrayLength] = { 0 };
unsigned char ledValue_[arrayLength]= {0};
short int duration_[arrayLength] = { 0 }; // ����ʱ��ʾ������Ϣ
unsigned char degree_[arrayLength] = {0};	//+0~49  -:50~
unsigned char leftSpeed_[arrayLength] = { 0 };
unsigned char rightSpeed_[arrayLength]={ 0 };
int instantSpeed[arrayLength]={0};

unsigned char curLedValue;
unsigned char lastLedValue;

int isDebug = 1;
int curSuffix = 1;
int lastSuffix = 0;
int printNow = 0;

int pushSw = 0;	//������:ȡ�����жϣ���ӡ���ݿ���
int isStop = 0;
int isStart = 0;

int turnSpeed ;
int lastDegree;
int lastSensor;

void main( void )
{
    int  i;	
	init();                             /* Initialization function     */
   eInit();
	init_sci1( 0x00, 79 );              /* Initialization of SCI1      */
    set_ccr( 0x00 );                    /* Entire interrupt permission */
		
	myInit();  //��ʼ������  3��direction = 0



	while(!pushsw_get())
	{
		if(cnt1<100)
			led_out(0x1);
		else if(cnt1<200)
			led_out(0x2);
		else	cnt1 = 0;
	}
	cnt1 = 0;
	while(startbar_get())
	{
		if(cnt1<50)
			led_out(0x1);
		else if(cnt1<100)
			led_out(0x2);
		else	cnt1 = 0;
	}
	/*
	while(cnt1<100)
	{
		if(cnt1<50)
			led_out(0x1);
		else
			led_out(0x2);
	}*/
	
	led_out(0);
	start();
//	isStart=1;
//	pushSw = 1;	
	
	pattern = 10;	
	while(1)
	{

		switch(pattern)
		{

			case 10:	//ֱ��
					if(check_crossline()){	pattern = 30;break;	}
					if(check_blackArea()){	pattern = 40;break;	}
					
					straight_run();				//ֱ��ǰ�У������⣬ä�����
					break;
			
			case 20:	//���
						switch(turn_direction)
						{
							case -1:
									handle(-1*midDegree_st4);
									left_turn();
									break;
							case 1:
									handle(midDegree_st4);
									right_turn();
									break;
							default:break;
						}
						break;


			case 30:	//ֱ��
						rightAngle();						
						break;


			case 40:	//ä��
						switch(blackArea_direction)
						{
							case -1:
									left_blackArea();
									break;
							case 1:
									right_blackArea();
									break;
							default:break;									
						}
						break;

			default:break;

		}//end switch

	}//end while
	
	
}//end main

/************************************************************************/
/* H8/3048F-ONE Built in Peripheral Function Initialization             */
/************************************************************************/
void init( void )
{
    /* I/O port Setting */
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
//�ҵĳ�ʼ������
void myInit(void)
{
	//if((~P6DR)&0x08) isDebug = 1;	
	//else isDebug = 1;
	isDebug = 0;
	lastSpeed_l = 0;
	lastSpeed_r = 0;
	lastDegree = 0;
	
	pushSw = 0;
	turnSpeed = 95;
//	turnSpeed = 100 - 2*(dipsw_get()&0x07);
}
//�ǶȺ���
void handle( int angle )
{
	int i;
	degree = angle;
	
	if(printNow) print(); //���Ժ���
	if(isStop) stop();
	
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
	speed_max=(unsigned long)(PWM_CYCLE-1) *sw_data/20;
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
/************************************************************************/
/* external Interrupt process                                               */
/************************************************************************/

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
    cnt0++;
    cnt1++;
	cnt2++;
/////////////////////////////////////////////////////////////////
    timerCnt++;	
	
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
	
	if(crankFlag==0&&(int)eCurSpeed>290&&(int)eCurSpeed<320&&timerCnt<2) 
	{
		
		//led_out(0x03);
	
	
		ITU4_BRA=(unsigned long)(PWM_CYCLE-1)>>2;//1/8
		ITU3_BRB=(unsigned long)(PWM_CYCLE-1)>>2;
	
		for(speedBrake_var=600;speedBrake_var!=0;speedBrake_var--);	//600
		
	}
   if(crankFlag==0&&(int)eCurSpeed<345&&(int)eCurSpeed>320&&timerCnt<15)	//345,330
	{
		ITU4_BRA=(unsigned long)(PWM_CYCLE-1)>>6;//1/8
		ITU3_BRB=(unsigned long)(PWM_CYCLE-1)>>6;
	
		for(speedBrake_var=800;speedBrake_var!=0;speedBrake_var--);	//800

	}
    if(crankFlag==0&&(int)eCurSpeed>345&&(int)eCurSpeed<500)
	{
		ITU4_BRA=0;//1/8
		ITU3_BRB=0;
		for(speedBrake_var=2500;speedBrake_var!=0;speedBrake_var--);
	}
////////////////////encoder for crank////////////////////////////////////
	

////////////////////////////////////////////////////////////////////	
	if(isStart)
	{
		if(isDebug)
		{
			if(!printNow)
			{
				if((~PBDR&0x01) && pushSw){ isStart = 0;isDebug = 0;printNow = 1;}//�н���;����������
				curLedValue = P7DR;
				if(curLedValue!=lastLedValue)
					{
						getData(curLedValue,0);
						
					}
				else if(bit_change(P7DR)==0xff && cnt0>200)	//���Գ���  before p7dr=0xff
				{
					//ITU3_BRB=0;ITU4_BRA=0;
					printNow = 1;	
				}
			}
		}
		else //�ǵ��Գ���
		{
			if(lastLedValue!=P7DR)
			{
				lastLedValue = P7DR;
				cnt0 = 0;	
			}else if(bit_change(P7DR) ==0xff && cnt0>200)	isStop = 1;// before p7dr=0xff
		}
	}
}
void getData(unsigned char ledValue,int debug) //debugȡֵ=>  0:������	<0:�����	>0���ҵ��� =>debug�ľ���ֵ����Ϣ��
{											   //�����������е�ֵ��duration[]����ֵ����:	>=0����״̬����ʱ��   -2�������  -1:�ҵ��� (��printDebug()��������)
	short int temp;
	unsigned char var;
	if(isDebug)
	{
		if(curSuffix<arrayLength)
		{	
			pattern_[curSuffix] = pattern;  	//��ǰģʽ	
			ledValue_[curSuffix] = ledValue;	//��ǰled����
			if(lastSuffix+1 ==curSuffix)//����˵����һ�μ����˵�����Ϣ�������޸�lastSuffix�±���ڵ�ֵ�����±�����һ�ε���ʱ�Ѿ�ͳ�����
				duration_[lastSuffix] = cnt0;		//��һ״̬����ʱ��
			
			temp = (short int)(SERVO_CENTER-ITU4_BRB)/HANDLE_STEP;
			if(temp<0) temp = -1*temp+50;
			degree_[curSuffix] =  temp; 			//��ǰled״̬��Ӧ�ĽǶȵ���
			
			var = (unsigned char)((1.0*ITU3_BRB/(PWM_CYCLE-1))*100);
			if(PBDR&0x04)
				leftSpeed_[curSuffix] = var+150;		//��ǰled״̬��Ӧ�������ٶȵ���
			else leftSpeed_[curSuffix] = var;
			var = (unsigned char)((1.0*ITU4_BRA/(PWM_CYCLE-1))*100);
			if(PBDR&0x08)
				rightSpeed_[curSuffix] = var+150;		//��ǰled״̬��Ӧ�������ٶȵ���
			else rightSpeed_[curSuffix] = var;
	///////////////������ĵ�����Ϣ//////////////////////////
			instantSpeed[curSuffix]=(int)eCurSpeed;
	//////////////////////////////////////////////////////////		
			
			if(debug)
			{
				if(debug>0)
					duration_[curSuffix] = -1*debug;		//��־�����������������,��-43
				else duration_[curSuffix] = -100+debug;		//��־���������ҵ�������,��-143(���϶�Ӧ)
				lastLedValue = 0xaa;		
				curSuffix++;
			}
			else
			{
				lastLedValue = ledValue;
				lastSuffix = curSuffix;
				curSuffix++;			
			}

			cnt0 = 0;
		}else printNow = 0 ;  //before printNow = 1;
	}
	else return;		
}

void print()//�ú����в��ܵ���handle(),��Ϊֻ��handle()���ܵ��ô˺���
{

	int i ;
	int sum;
	isStart = 0;
	isDebug = 0;
	printNow = 0;
	isStop = 0;
	
	cnt1 = 0;
	
	/*
	while(cnt1<=650)
	{
		//580ms
		if(cnt1<=100)
			speedLevel = (int)3*cnt1/10;
		else if(cnt1<=200)
			speedLevel = (int)(cnt1-100)/4+30;
		else if(cnt1<=325)
			speedLevel = (int)4*(cnt1-200)/25+55;
		else if(cnt1<=475)
			speedLevel = (int)(cnt1-325)/10+75;
		else speedLevel  = (int)2*(cnt1-475)/35+90;
		
		speedLevel = 100-speedLevel;
		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
					handle(0);
					speed(speedLevel,speedLevel);
					break;
			case 0x04:
					handle(5);
					do_speed(5,speedLevel);
					break;
			case 0x06:
					handle(10);
					do_speed(-10,speedLevel);
					break;
			case 0x07:
					handle(15);
					do_speed(15,speedLevel);
			case 0x03:
					handle(20);
					do_speed(20,speedLevel);
					break;
					
			//left
			case 0x20:
					handle(-5);
					do_speed(-5,speedLevel);
					break;
			case 0x60:
					handle(-10);
					do_speed(-10,speedLevel);
					break;
			case 0xe0:
					handle(-15);
					do_speed(-15,speedLevel);
					break;
			case 0xc0:
					handle(-20);
					do_speed(-20,speedLevel);
					break;				
		}
	}
	*/
	speed(0,0);
	
	timer(2000);
	
	while(!pushsw_get()){ if(cnt1<300) led_out(0x01); else if(cnt1<600) led_out(0x02); else cnt1 = 0; }


	sum = 0;
	for(i=0;i<curSuffix;i++)
	{
		
		if(duration_[i]<0)
		{			
			printf("sum time:%d\n",sum);
			if(duration_[i]<-100)
				printf(getInfo(-1*(duration_[i]+100),-1));
			else printf(getInfo(-1*duration_[i],1));
			
			printf("\n%3d>:  %2d (%02X,%02X)[",i,pattern_[i],bit_change(0xe7&ledValue_[i]),bit_change(ledValue_[i]));		
			getLightLed(bit_change(ledValue_[i]));
			printf("] %4d     ||     %3d (%3d,%3d)      %3d          \n",duration_[i],getDegree_(degree_[i]),getSpeed_(leftSpeed_[i]),getSpeed_(rightSpeed_[i]),instantSpeed[i]);
			
			sum = 0;
		}else
		{
			sum +=duration_[i];
			
			printf("%3d>:  %2d (%02X,%02X)[",i,pattern_[i],bit_change(0xe7&ledValue_[i]),bit_change(ledValue_[i]));		
			getLightLed(bit_change(ledValue_[i]));
			printf("] %4d     ||     %3d (%3d,%3d)      %3d          \n",duration_[i],getDegree_(degree_[i]),getSpeed_(leftSpeed_[i]),getSpeed_(rightSpeed_[i]),instantSpeed[i]);			
		}
	}
	printf("sum time:%d\n\n",sum);

	while(!pushsw_get()){ if(cnt1<500) led_out(0x01); else if(cnt1<1000) led_out(0x02); else cnt1 = 0; }
	printf("ht,good!^_^\n");
	stop();
}

/************************************************************************/
/* Timer main unit                                                      */
/* Argument Timer value 1=1ms                                           */
/************************************************************************/
void timer( unsigned long timer_set )
{
    cnt0 = 0;
    while( cnt0 < timer_set );
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
	
	if(sensor!=lastSensor&&sensor!=0xff&&pattern!=0)lastSensor=sensor;
	
	if(pattern==0)sensor=lastSensor;
	
	
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

int check_blackArea(void)
{
	unsigned char b = bit_change(P7DR);
	if((b&0x07)==0x07 && (b&0x38) && !(b&0xc0))
	{
		blackArea_direction = 1;
		return 1;
	}
	else if((b&0xe0)==0xe0 && (b&0x1c) && !(b&0x03))
	{
		blackArea_direction = -1;
		return -1;
	}
	else return 0;	
}

unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw  = ~P6DR;                        /* Reading of DIP switch */
    sw &= 0x0f;

    return  sw;
}

unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~PBDR;                        /* Reading of port having a push switch */
    sw &= 0x01;

    return  sw;
}
unsigned char startbar_get( void )
{
    unsigned char b;

    b  = ~P7DR;                        /* Read start bar signal  */
    b &= 0x10;
    b >>= 4;

    return  b;
}
void led_out( unsigned char led )
{
    unsigned char data;

    led = ~led;
    led <<= 6;
    data = PBDR & 0x3f;
    PBDR = data | led;
}

unsigned char  bit_change(unsigned char  in )
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
	
	if(sensor!=lastSensor&&sensor!=0xff&&pattern!=0)lastSensor=sensor;
	
	if(pattern==0)sensor=lastSensor;
	
    return sensor; 
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
					if(check_blackArea()){	pattern = 40;return;	}
									
					if(curSt!=st3)
					{
						lastSt = curSt;
						curSt = st3;
					}					
					break;
					
		case 0x03:
					if(check_crossline()){	pattern = 30;return;	}
					if(check_blackArea()){	pattern = 40;return;	}
										
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
					if(check_blackArea()){	pattern = 40;return;	}
					
					if(curSt!=st_3)
					{
						lastSt = curSt;
						curSt = st_3;
					}					
					break;
					
		case 0xc0:
					if(check_crossline()){	pattern = 30;return;	}
					if(check_blackArea()){	pattern = 40;return;	}
										
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
	do_speed(degree,speedLevel);

}
//���
void left_turn()
{	

	int	std_st; 
	unsigned char b;
	int max = 0;	 
	int loop;
	int isSteady;	
	getData(P7DR,-21); //debug-21
	
	//step1:����Ƿ��������	
	if(sensor_inp(MASK3_3)==0xc0 && (bit_change(P7DR) &0x18)){ pattern = 10; return; }
				
	//step2:��ƫ27��
	speed(0,60);
	for(degree=-15;degree>=-27;degree--)
		handle(degree);
		
	//step3:�����ƫת�̶ȴ���
	curSt = -4;
	loop = 1;
	max = 4;
	cnt2 = 0;
	cnt1 = 0;
	isSteady = 0;
	while(loop || !isSteady)
	{
		b = bit_change(P7DR);
		//step3_1:�Ҳ��е����Ĵ���(�Ƕ�)
		if(b&0x07)
		{
			if((b&0x0f)==0x0f)
			{
				if(curSt!=8)
				{ 
					curSt=8; 
					max = curSt;				
					if(degree>-38) degree = -38;
					else degree--;
					cnt1 = 0; 
				}
				else if(cnt1>60){	degree--; cnt1 = 0;  }	
			}
			//��3����
			else if((b&0x07)==0x07)
			{
				if(curSt!=7)
				{ 
					curSt=7; 
					max = curSt;				
					if(degree>-35) degree = -35;
					else degree--;
					cnt1 = 0; 
				}
				else if(cnt1>60){	degree--; cnt1 = 0;  }	
			}//��2����
			else if((b&0x03)==0x03)
			{
				if(curSt!=6)
				{
					curSt = 6;
					if(curSt>max)
					{
						max = curSt;
						if(degree>-33) degree=-33;
						else degree--;
						cnt1 = 0;
					}
					else loop=0;	
				}else if(cnt1>60){	degree--; cnt1 = 0; }	
			}//��1����
			else if((b&0x01)==0x01 || (b&0xe0)==0x80)
			{
				if(curSt!=5)
				{
					curSt = 5;
					if(curSt>max)
					{
						max = curSt;
						if(degree>-31) degree=-31;
						else degree--;
						cnt1 = 0;
					}
					else loop=0;	
				}else if(cnt1>60){	degree--; cnt1 = 0; }	
			}
			
			handle(degree);
			if(cnt2>400) speed(40,100);
			else if(cnt2>300) speed(20,90);
			else if(cnt2>200) speed(10,85);
			else if(cnt2>100) speed(7,80);
			else if(cnt2>50) speed(3,75);
			else speed(0,60);
		}
		else if((b&0xe7)==0xc0)
		{
			if(max>4)loop = 0;
			if(cnt2<50){ handle(-27);speed(0,60); }
			else if(cnt2<90){ handle(-28); do_speed(-28,70); }
			else if(cnt2<130){ handle(-29);do_speed(-29,80); }
			else{ handle(-30); do_speed(-30,90); }
		}else if((b&0xe7)==0x40 || (b&0xe7)==0xe0 || (b&0xe7)==0x60 || (b&0x20))
			loop = 0;
			
		if(!loop)
		{
			switch(max)
			{
				case 4: if(degree>-27)degree = -27;break;
				case 5: if(degree>-31)degree = -31;break;
				case 6: if(degree>-33)degree = -33;break;
				case 7: if(degree>-35)degree = -35;break;
				case 8:if(degree>-38)degree = -38;break;
			}
			handle(degree);
			b = bit_change(P7DR);
			while(b&0x07)
			{ 
				b = bit_change(P7DR);
				switch(b&0x0f)
				{
					case 0x01: curSt = 5;break;
					case 0x03: curSt = 6;break;
					case 0x07: curSt = 7;break;
					case 0x0f: curSt = 8;break;
					default: curSt = 8;
				}
				if(curSt>=7) speed(0,70);
				else if(max>=7)do_speed(degree,95);
				else do_speed(degree,90);
			}
			isSteady = 1;
		}			
	}//end while
	
	//step4:״̬��С����	
//	degree = (-27+degree)/2;
	if(degree>-27) degree = -27;
	handle(degree);
	
	getData(P7DR,-22);//add 
	
	curSt = -4;
	speedLevel = 95;	
	cnt1 = 0;
	do
	{
		//step4_1:����û����ʱҲ�ܼ������
		if(check_crossline())	{	pattern = 30;return;  }				//ֱ�Ǽ��

		//step4_2:���������������Ӧ�ı�׼״ֵ̬
		switch(get_near_std(degree))
		{
			case std_4: std_st = -3;break;	/*  (-~-25) 	*/
			case std_3: std_st = -2;break;	/*  [-25,-15) 	*/
			case std_2: std_st = -1;break;	/*  [-15,-10) 	*/
			case std_1: std_st = 0;break;	/*  [-10,-5) 	*/
			case std0:  					/*  [-5,+~) 	*/
						getData(P7DR,-23);  //debug-23
						pattern = 10;
						speedLevel = turnSpeed;
						cnt2 = 0;
						return;
		}

		//step4_3:ȡ��������ǰ״̬,�복��״ֵ̬����ƫת�Ƕ�
		b= bit_change(P7DR);			
		switch(b&0xe7)
		{
			case 0x80:	degree = -29;break;
			case 0xc0:	if(curSt!=-4)   
						{ 
							curSt=-4;    
							if(degree>-27)
								degree = -27;
							else degree+=(curSt-std_st)*4;  
						}
						break;
			case 0xe0:	if(curSt!=-3) { curSt=-3;    degree+=(curSt-std_st)*4;   }break;
			case 0x60:	if(curSt!=-2) { curSt=-2;    degree+=(curSt-std_st)*4;   }break;
			case 0x20:	if(curSt!=-1) { curSt=-1;    degree+=(curSt-std_st)*4;   }break;
			case 0x00:	if(curSt!=0)  { curSt=0;     degree+=(curSt-std_st)*4;   }break;
			case 0x04:	if(curSt!=1)  { curSt=1;     degree+=(curSt-std_st)*4;   }break;
			case 0x06:  if(curSt!=2)  { curSt=2;     degree+=(curSt-std_st)*4;   }break;													
			default:
				curSt = 1;
				cnt1 = 0;
				max = 0;
				while((bit_change(P7DR)&0x07))
				{
					if((bit_change(P7DR)&0x07)==0x07)
					{
						if(curSt!=3)
						{ 
							curSt = 3;
							cnt1 = 0; 
							if(degree<-35)degree--;
							else degree = -35;
						}
						else if(cnt1>100)
						{
							degree--;
							cnt1 = 0;	
						}
						if(3>max){  max = 3; speedLevel = turnSpeed-21; }
					}
					else if((bit_change(P7DR)&0x03)==0x03)
					{
						if(curSt!=2)
						{ 
							curSt = 2;
							cnt1 = 0; 
							if(degree<-33)degree--;
							else degree = -33;
						}
						else if(cnt1>100)
						{
							degree--;
							cnt1 = 0;	
						}
						if(2>max){  max = 2; speedLevel = turnSpeed-14; }
						else speedLevel = turnSpeed - 5;
					}else if(bit_change(P7DR)&0x01)
					{
						if(curSt!=1)
						{ 
							curSt = 1;
							cnt1 = 0; 
							if(degree<-31)degree--;
							else degree = -31;
						}
						else if(cnt1>100)
						{
							degree--;
							cnt1 = 0;	
						}
						if(1>max){  max = 1; speedLevel = turnSpeed-7; }
						else speedLevel = turnSpeed + 10;
					}
					
					handle(degree);
					do_speed(degree,speedLevel);							
				}//end while
				speedLevel = turnSpeed;
				curSt = -4;
		}//end switch

		handle(degree);
		do_speed(degree,speedLevel);
	}while(1);
}

void right_turn()
{
	
	int	std_st; 
	unsigned char b;
	int max = 0;	 
	int loop;
	int isSteady;
	
	
	getData(P7DR,21); //debug21
	
	//step1:����Ƿ��������	
	if(sensor_inp(MASK3_3)==0x03 && (bit_change(P7DR) &0x18)){ pattern = 10; return; }
				
	//step2:��ƫ27��
	speed(60,0);
	for(degree=15;degree<=27;degree++)
		handle(degree);
		
	//step3:�����ƫת�̶ȴ���
	curSt = 4;
	loop = 1;
	max = 4;
	cnt2 = 0;
	cnt1 = 0;
	isSteady = 0;
	while(loop || !isSteady)
	{
		b = bit_change(P7DR);
		if(b&0xe0)
		{
			if((b&0xf0)==0xf0)
			{
				if(curSt!=8)
				{ 
					curSt=8; 
					max = curSt;				
					if(degree<38) degree = 38;
					else degree++;
					cnt1 = 0; 
				}
				else if(cnt1>60){	degree++; cnt1 = 0;  }	
			}
			//��3����
			else if((b&0xe0)==0xe0)
			{
				if(curSt!=7)
				{ 
					curSt=7; 
					max = curSt;				
					if(degree<35) degree = 35;
					else degree++;
					cnt1 = 0; 
				}
				else if(cnt1>60){	degree++; cnt1 = 0;  }	
			}//��2����
			else if((b&0xc0)==0xc0)
			{
				if(curSt!=6)
				{
					curSt = 6;
					if(curSt>max)
					{
						max = curSt;
						if(degree<33) degree=33;
						else degree++;
						cnt1 = 0;
					}
					else loop=0;	
				}else if(cnt1>60){	degree++; cnt1 = 0; }	
			}//��1����
			else if((b&0x80)==0x80 || (b&0x07)==0x01)
			{
				if(curSt!=5)
				{
					curSt = 5;
					if(curSt>max)
					{
						max = curSt;
						if(degree<31) degree=31;
						else degree++;
						cnt1 = 0;
					}
					else loop=0;	
				}else if(cnt1>60){	degree++; cnt1 = 0; }	
			}
			
			handle(degree);
			if(cnt2>400) speed(100,40);
			else if(cnt2>300) speed(90,20);
			else if(cnt2>200) speed(85,10);
			else if(cnt2>100) speed(80,7);
			else if(cnt2>50) speed(75,3);
			else speed(60,0);
		}
		else if((b&0xe7)==0x03)
		{
			if(max>4)loop = 0;
			if(cnt2<50){ handle(27);speed(60,0); }
			else if(cnt2<90){ handle(28); do_speed(28,70); }
			else if(cnt2<130){ handle(29);do_speed(29,80); }
			else{ handle(30); do_speed(30,90); }
		}else if((b&0xe7)==0x02 || (b&0xe7)==0x07 || (b&0xe7)==0x06 || (b&0x04))
			loop = 0;
			
		if(!loop)
		{
			switch(max)
			{
				case 4: if(degree<27)degree = 27;break;
				case 5: if(degree<31)degree = 31;break;
				case 6: if(degree<33)degree = 33;break;
				case 7: if(degree<35)degree = 35;break;
				case 8: if(degree<38)degree = 38;break;
			}
			handle(degree);
			b = bit_change(P7DR);
			while(b&0xe0)
			{ 
				b = bit_change(P7DR);
				switch(b&0xf0)
				{
					case 0x80: curSt = 5;break;
					case 0xc0: curSt = 6;break;
					case 0xe0: curSt = 7;break;
					case 0xf0: curSt = 8;break;
					default: curSt = 8;
				}
				if(curSt>=7) speed(70,0);
				else if(max>=7)do_speed(degree,95);
				else do_speed(degree,90);
			}
			isSteady = 1;
		}			
	}//end while
	
	//step4:״̬��С����	
	degree = (27+degree)/2;
	if(degree<27) degree = 27;
	handle(degree);

	curSt = 4;
	speedLevel = 95;	
	cnt1 = 0;
	do
	{
		if(check_crossline())		//ֱ�Ǽ��
		{
			pattern = 30;
			return;
		}
		switch(get_near_std(degree))
		{
			case std4: std_st = 3;break;  /* (25,+~) */					
			case std3: std_st = 2;break;	/* (15,25] */
			case std2: std_st = 1;break;	/* (10,15] */
			case std1: std_st = 0;break;	/* (5,10] */
			case std0: /* [-5,+~) */
			getData(P7DR,23); //debug23
						pattern = 10;
						speedLevel = turnSpeed;
						cnt2 = 0;
						return;
		}
			
		switch(sensor_inp(MASK3_3))
		{
			case 0x01:	degree = 29;break;
			case 0x03:	if(curSt!=4)   
						{ 
							curSt=4;    
							if(degree<27)
								degree = 27;
							else degree+=(curSt-std_st)*4;  
						}
						break;
			case 0x07:	if(curSt!=3)   { curSt=3;    degree+=(curSt-std_st)*4;   }break;
			case 0x06:	if(curSt!=2)   { curSt=2;    degree+=(curSt-std_st)*4;   }break;
			case 0x04:	if(curSt!=1)   { curSt=1;    degree+=(curSt-std_st)*4;   }break;
			case 0x00:	if(curSt!=0)   { curSt=0;    degree+=(curSt-std_st)*4;   }break;
			case 0x20:	if(curSt!=-1)  { curSt=-1;   degree+=(curSt-std_st)*4;   }break;
			case 0x60:  if(curSt!=-2)  { curSt=-2;   degree+=(curSt-std_st)*4;   }break;													
			
			default:
					curSt = 1;
					cnt1 = 0;
					max = 0;
					while((bit_change(P7DR)&0xe0))
					{
						if((bit_change(P7DR)&0xe0)==0xe0)
						{
							if(curSt!=3)
							{ 
								curSt = 3;
								cnt1 = 0; 
								if(degree>35)degree++;
								else degree = 35;
							}
							else if(cnt1>100)
							{
								degree++;
								cnt1 = 0;	
							}
							if(3>max){  max = 3; speedLevel = turnSpeed-21; }
						}
						else if((bit_change(P7DR)&0xc0)==0xc0)
						{
							if(curSt!=2)
							{ 
								curSt = 2;
								cnt1 = 0; 
								if(degree>33)degree++;
								else degree = 33;
							}
							else if(cnt1>100)
							{
								degree++;
								cnt1 = 0;	
							}
							if(2>max){  max = 2; speedLevel = turnSpeed-14; }
							else speedLevel = turnSpeed - 5;
						}else if(bit_change(P7DR)&0x80)
						{
							if(curSt!=1)
							{ 
								curSt = 1;
								cnt1 = 0; 
								if(degree>31)degree++;
								else degree = 31;
							}
							else if(cnt1>100)
							{
								degree++;
								cnt1 = 0;	
							}
							if(1>max){  max = 1; speedLevel = turnSpeed-7; }
							else speedLevel = turnSpeed + 10;
						}
						handle(degree);
						do_speed(degree,speedLevel);							
					}//end while
					speedLevel = turnSpeed;	
					curSt = 4;	
		}//end switch
		
		handle(degree);
		do_speed(degree,speedLevel);
	}while(1);

}
//

//ֱ��
void rightAngle()
{
	unsigned char b;
	int std_st;	
	int waitForAngle;
	int AngleDirection;	
	int max = 0 ;
	int var;
	int reachMax;	
	int speedLevel_;
	int degree_;
	
	int speedToMax;
	
	if(!check_crossline()) { pattern =10; return ;}
	getData(P7DR,31); 								//debug31
	
	//step1:��˫����(31ms 34ms 34ms)
	led_out(0x03);
	handle(0);
	speed(0,0); //before speed(60,60);
	cnt1 = 0;
	var = 0;
	
	

	while(cnt1<180 && (int)eCurSpeed>140 )
	{
		crankFlag=1;	
		if(cnt1<30) speed(-60-2,-60-2);//-6
		else if(cnt1<80) speed(-63-2,-63-2);
		else if(cnt1<120) speed(-66-2,-66-2);
		else if(cnt1< 180) speed(-70-2,-70-2);
		
			
		switch( sensor_inp(MASK3_3) ) {
            case 0x00:
				degree = 0;
                break;
            case 0x04:
				degree = 2;	//5
				break;
            case 0x06:
				degree = 10;
				break;
            case 0x07:
			case 0x27:
			case 0x67:
				if(cnt1>90)
				cnt1=1000;
                break;
            case 0x20:
				degree = -2;//-5
				break;
            case 0x60:
				degree = -10;
				break;
            case 0xe0:
			case 0xe4:
			case 0xe6:
				if(cnt1>90)
				cnt1=1000;
				break;
            default:
                break;
        }
  		handle(degree);

	}
	crankFlag=0;
	/*
	while(cnt1<100)
	{
		if(var==0 && getLightLedSum()<=2)var++;
		if(var==1 && getLightLedSum()>2)var++;
		if(var==2 && getLightLedSum()<=2)var++;
		if(var==3)break;
	}
	
	//////////encode flag////////
	
	crankFlag=1;
	//speed(0,0);
	if(cnt1<32) speedLevel_ = 38;
	else if(cnt1<35) speedLevel_ = 42;
	else if(cnt1<40) speedLevel_ = 43;
	else speedLevel_ = 45;		
	;
									//debug32
*/
   getData(P7DR,32); 
   led_out(0x00);
   
   	
	//step2:���ٵ���(���˫���ߵ�ʱ���й�)
	cnt1 = 0;
	while(cnt1<300)
	{	
		
		if((int)eCurSpeed>140) speedLevel =65-2;//orig:60
		else if((int)eCurSpeed>100 )speedLevel= 70-2;//orig:80
		//speedLevel = 70;						//fast -> slow
		if(cnt1>250) speedLevel = speedLevel+3; 		//41 45 48 53
		else if(cnt1>200) speedLevel =speedLevel+2; 	//45 49 52 57
		else if(cnt1>150) speedLevel = speedLevel;	//48 52 55 60
		
				
		switch( sensor_inp(MASK3_3) ) {
            case 0x00:
				degree = 0;
                break;
            case 0x04:
				degree = 2;//5
				break;
            case 0x06:
				degree = 10;
				break;
 			case 0x07:
			case 0x27:
			case 0x67:
			case 0x03:	//new
			case 0x23:	//
			case 0x05:
				cnt1=1000;
                break;
            case 0x20:
				degree = -2;//5
				break;
            case 0x60:
				degree = -10;
				break;
            case 0xe0:
			case 0xe4:
			case 0xe6:
			case 0xc0:	//new
			case 0xc4:	//
			case 0xa0:

				cnt1=1000;
				break;
            default:
                break;
        }
  		handle(degree);
		do_speed(degree,speedLevel);
	}
	getData(P7DR,33); //debug33

	//step3:�ȴ�ֱ�ǵ���(175ms)
	speedLevel =70;
	waitForAngle = 1;
	cnt1 = 0;
	while(waitForAngle)
	{
		handle(0);
		if(cnt1>100) speedLevel = 70;
		else if(cnt1>50) speedLevel = 70;
		switch( sensor_inp(MASK3_3) ) {	
            case 0xe0:
			case 0xe4:
			case 0xe6:
			case 0xc4:	//
			case 0xa0:
					speed(0,0);
					//for(degree=-1;degree>=-44;degree-=1) handle(degree);  //before -40
					handle(-44);
					led_out( 0x1 );	                
					AngleDirection = -1;
					waitForAngle = 0;
					//step4:����ֱ�ǰ���
					getData(P7DR,-34);				//debug-34
	                break;
			case 0x07:
			case 0x27:
			case 0x67:
			case 0x23:	//
			case 0x05:
					speed(0,0);
	//				for(degree=1;degree<=44;degree+=1)	handle( degree );   //before 40
					handle(44);
					led_out( 0x2 );	                
					AngleDirection = 1;
					waitForAngle = 0;
					//step4:����ֱ�ǰ���
					getData(P7DR,34);				//debug34
	                break;
	            case 0x00:
	                handle( 0 );
	                speed(speedLevel,speedLevel);
	                break;
	            case 0x04:
					handle(5);
					do_speed(5,speedLevel);
					break;
	            case 0x06:
					handle(10);
					do_speed(10,speedLevel);
					break;
	            case 0x03:
	                handle( 15 );
				    do_speed(15,speedLevel);
	                break;
	            case 0x20:
					handle(-5);
					do_speed(-5,speedLevel);
					break;
	            case 0x60:
					handle(-10);
				 	do_speed(-10,speedLevel);
					break;
	            case 0xc0:
	                handle( -15 );
				   	do_speed(-15,speedLevel);
	                break;
	            default:
	                break;
	        }
		 	if(!sensor_inp(MASK4_4)) //��ä������Ϊֱ��
			{
				pattern = 40;
				return;
			}
	}	
	///////encode ///////////////
	

	//תֱ�ǹ��̴���(����)
	switch(AngleDirection)
	{
		case -1:
				cnt1 = 0;
				
				
				while(cnt1<50)// while(  ! (bit_change(P7DR)&0X03 ) )
				{
					if(!bit_change(P7DR) ) break;//���԰���(16ms)
				}
				var = cnt1;
				getData(P7DR,-35); 				//debug-35(��������)
				
				//step5:��������(��ͷ����)
				if(var<17){ degree_ = -43; speedLevel_ = 45; }
				else if(var<25){ degree_ = -42; speedLevel_ = 48; }
				else if(var<35){ degree_ = -41; speedLevel_ = 51; }
				else if(var<45){ degree_ = -40; speedLevel_ = 54; }
				else { degree_ = -40; speedLevel_ = 57; }
				
				cnt1 = 0;
				degree = degree_;
				speedLevel = speedLevel_;
				while(!bit_change(P7DR) )					//���ݰ���ʱ��var��������ƫת����
				{
					handle(degree);
					speed(0,speedLevel);	
				}				
				getData(P7DR,-36); 				//debug-36
				
				//step6:��ͷ����				
				speedLevel = 60;
				cnt1 = 0;
				reachMax = 0;
				max = 0;
				curSt = 0;
				b = bit_change(P7DR);
				//cnt2=0;
				while(  (b&0xe7)!=0x60  &&  (b&0xe7)!=0x20)//(   ( (b&0xe7)!=0x60 || cnt2<20 ) &&   ( (b&0xe7)!=0x20 || cnt2<20)  )
				{
					if(!reachMax)				//���״̬ǰ
					{
						if(degree>-40)
							degree = -40;
						else degree = degree_;
						
						switch(b&0xe7)
						{
							case 0x00: if(curSt!=0){ curSt = 0; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(0,75);break;
							case 0x20: if(curSt!=1){ curSt = 1; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(0,75);break;
							case 0x60: if(curSt!=2){ curSt = 2; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(0,75);break;
							case 0xe0: if(curSt!=3){ curSt = 3; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(0,75);break;
							case 0xc0: if(curSt!=4){ curSt = 4; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(0,75);break;
							case 0x80: if(curSt!=5){ curSt = 5; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(0,75);break;
							default:
								if((b&0x0f)==0x0f){  curSt = 9; if(degree>-44)degree = -44; max= 9; speed(0,52); }
								else if((b&0x07)==0x07)
								{ 
									if(curSt!=8)
									{ 
										curSt=8; 
										if(degree>-43)
											degree=-43; 
										if(curSt>=max) 
											max = curSt;
										else
										{ 
											reachMax = 1;
											cnt1 = 0; 
										} 
									}
									speed(0,51);
								}
								else if((b&0x03)==0x03)
								{ 
									if(curSt!=7)
									{ 
										curSt=7; 
										if(degree>-42)
											degree=-42; 
										if(curSt>=max) 
											max = curSt;
										else
										{ 
											reachMax = 1;
											cnt1 = 0; 
										} 
									}
									speed(0,50);
								}
								else if(b&0x01)
								{
								  	if(curSt!=6)
									{ 
										curSt=6; 
										if(degree>-41)
											degree=-41; 
										if(curSt>=max) 
											max = curSt;
										else
										{ 
											reachMax = 1;
											cnt1 = 0; 
										}   
									}
									speed(0,45);
								}
						}
						handle(degree);
			
					}else
					{
						if(degree>-40)
							degree = -40;								//���״̬��  (86ms)
						while(bit_change(P7DR)&0x07)
						{ 
							handle(degree);						
							if(cnt1>85)do_speed(-38,95);
							else if(cnt1>65)do_speed(-38,93);
							else if(cnt1>50)do_speed(-38,91);
							else if(cnt1>35)speed(get_inner_speed(-38,89)-5,89);
							else if(cnt1>20)speed(get_inner_speed(-38,87)-10,87);
							else speed(0,85);
						}
						handle(-38);
						do_speed(-38,90);
					}
					
					b = bit_change(P7DR);	
				}//end while
				
				//step7:ֱ��ƫת����,����������ֱ��
				getData(P7DR,-37);		
				speedLevel = 90;
				cnt1 = 0;
				curSt = -2;
				do{				
						switch(get_near_std(degree))
						{
							case std_4: std_st = -3;break; //-27
							case std_3: std_st = -2;break; //-15
							case std_2: std_st = -1;break; //-10
							case std_1: std_st = 0;break; //-5
							case std0:  				  //0
									led_out( 0x0 );									
									pattern = 10;
				//step8:��������    
									//crankFlag=0;
									getData(P7DR,-38); //debug-38
									return;
						}
						switch(sensor_inp(MASK3_3))
						{
							case 0xc0:  if(curSt!=-4) { curSt=-4;    degree+=(curSt-std_st)*4;   }break;
							case 0xe0:	if(curSt!=-3) { curSt=-3;    degree+=(curSt-std_st)*4;   }break;
							case 0x60:	if(curSt!=-2) { curSt=-2;    degree+=(curSt-std_st)*4;   }break;
							case 0x20:	if(curSt!=-1) { curSt=-1;    degree+=(curSt-std_st)*4;   }break;
							case 0x00:	if(curSt!=0)  { curSt=0;    degree+=(curSt-std_st)*4;   }break;
							case 0x04:	if(curSt!=1)  { curSt=1;   degree+=(curSt-std_st)*4;   }break;
							case 0x06:  if(curSt!=2)  { curSt=2;   degree+=(curSt-std_st)*4;   }break;
							case 0x07:  if(curSt!=3)  { curSt=3;   degree+=(curSt-std_st)*4;   }break;	
						}
						handle(degree);
						do_speed(degree,speedLevel);
					
				}while(1);
				

		case 1:
				cnt1 = 0;
				while(cnt1<50)  //while(! (bit_change(P7DR) &0Xc0))
				{
					if(!bit_change(P7DR) ) break;//���԰���(16ms)
				}
				var = cnt1;
				getData(P7DR,35); 				//debug35(��������)
				
				//step5:��������(��ͷ����)
				if(var<17){ degree_ = 43; speedLevel_ = 45; }
				else if(var<25){ degree_ = 42; speedLevel_ = 48; }
				else if(var<35){ degree_ = 41; speedLevel_ = 51; }
				else if(var<45){ degree_ = 40; speedLevel_ = 54; }
				else { degree_ = 40; speedLevel_ = 57; }
				
				cnt1 = 0;
				degree = degree_;
				speedLevel = speedLevel_;
				while(!bit_change(P7DR) )					//���ݰ���ʱ��var��������ƫת����
				{
					handle(degree);
					speed(speedLevel,0);	
				}				
				getData(P7DR,36); 				//debug36
				
				//step6:��ͷ����				
				speedLevel = 60;
				cnt1 = 0;
				reachMax = 0;
				max = 0;
				curSt = 0;
				b = bit_change(P7DR);
				//cnt2=0;
				while(  (b&0xe7) !=0x06   &&(b&0xe7)!=0x04)//(   ((b&0xe7)!=0x06  ||(cnt2<20) ) &&(   (b&0xe7)!=0x04  || (cnt2<20) )    )
				{
					if(!reachMax)				//���״̬ǰ
					{
						if(degree<40)
							degree = 40;
						else degree = degree_;
						
						switch(b&0xe7)
						{
							case 0x00: if(curSt!=0){ curSt = 0; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(75,0);break;
							case 0x04: if(curSt!=1){ curSt = 1; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(75,0);break;
							case 0x06: if(curSt!=2){ curSt = 2; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(75,0);break;
							case 0x07: if(curSt!=3){ curSt = 3; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(75,0);break;
							case 0x03: if(curSt!=4){ curSt = 4; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(75,0);break;
							case 0x01: if(curSt!=5){ curSt = 5; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }speed(75,0);break;
							default:
								if((b&0xf0)==0xf0){  curSt = 9; if(degree<44)degree = 44; max= 9; speed(52,0); }
								else if((b&0xe0)==0xe0)
								{ 
									if(curSt!=8)
									{ 
										curSt=8; 
										if(degree<43)
											degree=43; 
										if(curSt>=max) 
											max = curSt;
										else
										{ 
											reachMax = 1;
											cnt1 = 0; 
										} 
									}
									speed(51,0);
								}
								else if((b&0xc0)==0xc0)
								{ 
									if(curSt!=7)
									{ 
										curSt=7; 
										if(degree<42)
											degree=42; 
										if(curSt>=max) 
											max = curSt;
										else
										{ 
											reachMax = 1;
											cnt1 = 0; 
										} 
									}
									speed(50,0);
								}
								else if(b&0x80)
								{
								  	if(curSt!=6)
									{ 
										curSt=6; 
										if(degree<41)
											degree=41; 
										if(curSt>=max) 
											max = curSt;
										else
										{ 
											reachMax = 1;
											cnt1 = 0; 
										}   
									}
									speed(45,0);
								}
						}
						handle(degree);
			
					}else
					{
						if(degree<40)
							degree = 40;								//���״̬��  (86ms)
						while(bit_change(P7DR)&0xe0)
						{ 
							handle(degree);						
							if(cnt1>85)do_speed(38,95);
							else if(cnt1>65)do_speed(38,93);
							else if(cnt1>50)do_speed(38,91);
							else if(cnt1>35)speed(get_inner_speed(38,89)-5,89);
							else if(cnt1>20)speed(get_inner_speed(38,87)-10,87);
							else speed(85,0);
						}
						handle(38);
						do_speed(38,90);
					}
					
					b = bit_change(P7DR);	
				}//end while
				
				//step7:ֱ��ƫת����,����������ֱ��
				getData(P7DR,37);							//debug37	
				speedLevel = 90;
				cnt1 = 0;
				curSt = 2;
				do{				
						switch(get_near_std(degree))
						{
							case std4: std_st = 3;break;
							case std3: std_st = 2;break;/* [-25,-15) */
							case std2: std_st = 1;break;/* [-15,-10) */
							case std1: std_st = 0;break;/* [-10,-5) */
							case std0:  /* [-5,+~) */
									led_out( 0x0 );					
									pattern = 10;
									//crankFlag=0;
									getData(P7DR,38); //debug39   add before 39
									return;
						}
						switch(sensor_inp(MASK3_3))
						{
							case 0xe0:	if(curSt!=-3) { curSt=-3;   degree+=(curSt-std_st)*4;  }break;
							case 0x60:	if(curSt!=-2) { curSt=-2;   degree+=(curSt-std_st)*4;  }break;
							case 0x20:	if(curSt!=-1) { curSt=-1;   degree+=(curSt-std_st)*4;  }break;
							case 0x00:	if(curSt!=0)  { curSt=0;    degree+=(curSt-std_st)*4;  }break;
							case 0x04:	if(curSt!=1)  { curSt=1;    degree+=(curSt-std_st)*4;  }break;
							case 0x06:  if(curSt!=2)  { curSt=2;    degree+=(curSt-std_st)*4;  }break;
							case 0x07:  if(curSt!=3)  { curSt=3;    degree+=(curSt-std_st)*4;  }break;	
							case 0x03:  if(curSt!=4)  { curSt=4;    degree+=(curSt-std_st)*4;  }break;
						}
						handle(degree);
						do_speed(degree,speedLevel);
					
				}while(1);	

	}
}


//ä��
void left_blackArea()
{
	int max;
	unsigned char b;
	int reachMax;
	//step1:��˫����
	
	getData(P7DR,-41);
	handle(0);
	speed(0,0);
	var = 0;
	while(cnt1<100)
	{
		if(check_crossline()){	pattern = 30;return;  }
		if(var==0 && getLightLedSum()<=4)var++;
		if(var==1 && getLightLedSum()>4)var++;
		if(var==2 && getLightLedSum()<=4)var++;
		if(var==3)break;
	}		
	getData((P7DR),-42);				//add	

	
	//step2:���ٵȴ�����
	speedLevel = 80; 
	while(sensor_inp(MASK4_4))
	{
		if(check_crossline())		//��ֱ������Ϊä��
		{
			pattern = 30;
			getData((P7DR),-48); 		//debug-46
			return;
		}
		switch(sensor_inp(MASK3_3))
		{
			case 0x00:  handle(0);speed(speedLevel,speedLevel);break;
			//case 0x04:	handle(5);do_speed(5,speedLevel);break;
			case 0x04:	handle(2);do_speed(2,speedLevel);break;
			//case 0x06:	handle(7);do_speed(10,speedLevel);break;
			case 0x06:	handle(10);do_speed(10,speedLevel);break;
			case 0x05:	//new add:0101
			case 0x07:	handle(15);do_speed(15,speedLevel);break;
			case 0x03:	handle(20);do_speed(20,speedLevel);break;
				
			//case 0x20:	handle(-5);do_speed(-5,speedLevel);break;
			case 0x20:	handle(-2);do_speed(-2,speedLevel);break;			
			//case 0x60:	handle(-7);do_speed(-10,speedLevel);break;
			case 0x60:	handle(-10);do_speed(-10,speedLevel);break;		
			case 0xa0:	//new add:1010
			case 0xe0:	handle(-15);do_speed(-15,speedLevel);break;
			case 0xc0:	handle(-20);do_speed(-20,speedLevel);break;
			
			//ȱ�ټ���״ֵ̬
			default: break;							
		}
	}	
	getData((P7DR),-43);				//debug-42:
	
	//step3:��Խ����
//	speed(0,0);
//	speed(0,51);
//	degree=-1;
//	while(degree>=-25){	handle(degree);	degree--; }   //18
//	handle(-25);
	speed(0,51);   //40
//	while(degree>=-28){ handle(degree);	degree--; }   //22
	handle(-28);
	//cnt1 = 0;
	//while(cnt1<10)
	//	speed(0,80);
/*	while(cnt1<10);
	do_speed(-22,85);				
*/		

	/*	
	while(!(bit_change((P7DR))&0xf0))//��ä���ٶ�75
	{
		handle(-28);	//old 22
		do_speed(-28,80); //old 22
	}
	*/
	cnt1 = 0;
	while(!(bit_change(P7DR)&0xf0))//��ä���ٶ�75 /07
	{
		if(cnt1<100)
		{
			handle(-35);	//old 22
			do_speed(-35,80); //old 22
		}
		else
		{
			handle(-18);	//old 20
			do_speed(-18,80);
		}

	}


	//step4:���˹���,��ƴ�������
	getData((P7DR),-44);				//debug-44:
	speed(80,0);
	//handle(30);
	for(degree=1;degree<=30;degree+=1)// old 23 
	{ 
		handle(degree);
	}

	timer(20);
	
	reachMax = 0;
	max = 0;
	curSt = 0;
//	b = bit_change((P7DR));			
	
    //��ʼ��������
	getData(P7DR,-45);
	
	b = sensor_inp(MASK4_4);
//	while((sensor_inp(MASK4_4)!=0x18) && (sensor_inp(MASK4_4)!=0x78 ))//&& (sensor_inp(MASK3_3)!=0xc0)
	while((b != 0x18) && (b != 0x38) && (b != 0x20) )//&& (b != 0x78) && (b != 0x60))	//�Ƿ����������״̬ 0x60
//	while((b&0xf7)!=0x10 && (b&0xf7)!=0x30) //	while((b&0xe7)!=0x20 && (b&0xe7)!=0x60)
	{
	//	b = bit_change((P7DR));
		b = sensor_inp(MASK4_4);
		if(!reachMax)			//���״̬ǰ
		{
			degree = 30;
			switch(b&0x07)
			{
				case 0x00: if(curSt!=0){ curSt = 0; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x04: if(curSt!=1){ curSt = 1; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x06: if(curSt!=2){ curSt = 2; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x07: if(curSt!=3){ curSt = 3; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x03: if(curSt!=4){ curSt = 4; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x01: if(curSt!=5){ curSt = 5; if(curSt>=max) max = curSt; }break;
				default:break;					
			}
			if((b&0xe0)==0xe0)degree = 35;
			else if((b&0xc0)==0xc0)degree = 33;
			else if((b&0x80)==0x80)degree = 31;  //23
			
		}
	 
			handle(degree);//////////////////////////////////
			speed(80,15);//u55			
	}
	getData((P7DR),-46);				
	
	
	//step5:�ȶ����������
	for(speedLevel=80;speedLevel<=85;speedLevel+=1)
	{
		//Ԥ��
		if(check_crossline()){	pattern = 30;return;	}
		if(check_blackArea()){	pattern = 40;return;	}
	
		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
					handle(0);
					speed(speedLevel,speedLevel);
					break;
			case 0x04:
					handle(2);//old 5 
					do_speed(2,speedLevel);
					break;
			case 0x06:
					handle(10);//old 7
					do_speed(-10,speedLevel);
					break;
			case 0x07:
					handle(15);
					do_speed(15,speedLevel);
			case 0x03:
					handle(20);
					do_speed(20,speedLevel);
					break;
					
			//left
			case 0x20:
					handle(-2);//old -5
					do_speed(-2,speedLevel);
					break;
			case 0x60:
					handle(-10);// old -10
					do_speed(-10,speedLevel);
					break;
			case 0xe0:
					handle(-15);
					do_speed(-15,speedLevel);
					break;
			case 0xc0:
					handle(-20);
					do_speed(-20,speedLevel);
					break;				
		}	
	}
	getData((P7DR),-47);
	//Ԥ��
	if(check_crossline()){	pattern = 30;return;	}
	if(check_blackArea()){	pattern = 40;return;	}
	pattern = 10;

	
}


void right_blackArea()
{
	int max;
	unsigned char b;
	int reachMax;
	//step1:��˫����
	getData(P7DR,41);
	handle(0);
	speed(0,0);
	var = 0;
	while(cnt1<100)
	{
		if(check_crossline()){	pattern = 30;return;  }
		if(var==0 && getLightLedSum()<=4)var++;
		if(var==1 && getLightLedSum()>4)var++;
		if(var==2 && getLightLedSum()<=4)var++;
		if(var==3)break;
	}		
	getData((P7DR),42);				//debug47
	
	//step2:���ٵȴ�����
	speedLevel = 80; 
	while(sensor_inp(MASK4_4))
	{
		if(check_crossline())		//��ֱ������Ϊä��
		{
			pattern = 30;
			getData((P7DR),48); 		//debug46
			return;
		}
		switch(sensor_inp(MASK3_3))
		{
			case 0x00:  handle(0);speed(speedLevel,speedLevel);break;
			//case 0x04:	handle(5);do_speed(5,speedLevel);break;
			case 0x04:	handle(2);do_speed(2,speedLevel);break;
			//case 0x06:	handle(7);do_speed(10,speedLevel);break;
			case 0x06:	handle(10);do_speed(10,speedLevel);break;
			case 0x05:	//new add:0101
			case 0x07:	handle(15);do_speed(15,speedLevel);break;
			case 0x03:	handle(20);do_speed(20,speedLevel);break;
				
			//case 0x20:	handle(-5);do_speed(-5,speedLevel);break;
			case 0x20:	handle(-2);do_speed(-2,speedLevel);break;			
			//case 0x60:	handle(-7);do_speed(-10,speedLevel);break;
			case 0x60:	handle(-10);do_speed(-10,speedLevel);break;		
			case 0xa0:	//new add:1010
			case 0xe0:	handle(-15);do_speed(-15,speedLevel);break;
			case 0xc0:	handle(-20);do_speed(-20,speedLevel);break;
			
			//ȱ�ټ���״ֵ̬
			default: break;							
		}
	}	
	getData((P7DR),43);				//debug42:
	
	//step3:��Խ����
//	speed(0,0);
//	speed(40,0);
	speed(51,0);
//	degree=1;
	//while(degree<=25){	handle(degree);	degree++; }
//	handle(25);
//	speed(50,0);
//	while(degree<=28){ handle(degree);	degree++; }
	handle(28);

	
	cnt1 = 0;
/*	while(cnt1<10);
	while(cnt1<10)
		speed(80,0);
	do_speed(22,85);*/				
		cnt1 = 0;
	while(!(bit_change(P7DR)&0x0f))//��ä���ٶ�75 /07
	{
		if(cnt1<100)
		{
			handle(35);	//old 22
			do_speed(35,80); //old 22
		}
		else
		{
			handle(18);	//old 20
			do_speed(18,80);
		}

	}
	
	//step4:���˹��� �ҵƴ�������
	getData((P7DR),44);				//debug44:
	
	speed(80,0);
	for(degree=-1;degree>=-30;degree-=1)
	{
		handle(degree);
	}
	
	timer(10);

	reachMax = 0;
	max = 0;
	curSt = 0;
//	b = bit_change((P7DR));
	b = sensor_inp(MASK4_4);
				
	//��ʼ��������
	getData(P7DR,45);
//	while(sensor_inp(MASK3_3)!=0x04 && sensor_inp(MASK3_3)!=0x06 )//&& sensor_inp(MASK3_3)!=0x07
//	while(sensor_inp(MASK4_4)!=0x1c  && sensor_inp(MASK4_4)!=0x1e )//&& sensor_inp(MASK3_3)!=0x07
	while((b != 0x18) && (b != 0x1c) && (b != 0x04) )//&& (b != 0x1e) && (b != 0x06))	//�Ƿ����������״̬ 0x60
//	while((b&0xff)!=0x18 &&(b&0xff)!=0x38)   	//while( (b&0xe7)!=0x04 &&(b&0xe7)!=0x06)
	{
		
	//	b = bit_change((P7DR));
		b = sensor_inp(MASK4_4);
		if(!reachMax)			//���״̬ǰ
		{
			degree = -30;
			switch(b&0xe0)
			{
				case 0x00: if(curSt!=0){ curSt = 0; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x20: if(curSt!=1){ curSt = 1; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x60: if(curSt!=2){ curSt = 2; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0xe0: if(curSt!=3){ curSt = 3; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0xc0: if(curSt!=4){ curSt = 4; if(curSt>=max) max = curSt;else{ reachMax = 1;cnt1 = 0;} }break;
				case 0x80: if(curSt!=5){ curSt = 5; if(curSt>=max) max = curSt; }break;
				default:break;					
			}
			if((b&0x07)==0x07)degree = -35;
			else if((b&0x03)==0x03)degree = -33;
			else if((b&0x01)==0x01)degree = -31;
			
			handle(degree);
			speed(15,60);			
		}
		
	}
	
	
	getData((P7DR),46);				
	//step5:�ȶ����������
	
	
	for(speedLevel=80;speedLevel<=85;speedLevel+=1)
	{
				//Ԥ��
		if(check_crossline()){	pattern = 30;return;	}
		if(check_blackArea()){	pattern = 40;return;	}
		
		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
					handle(0);
					speed(speedLevel,speedLevel);
					break;
			case 0x04:
					handle(2);	//old 5
					do_speed(2,speedLevel);
					break;
			case 0x06:
					handle(10);	//old 7
					do_speed(-10,speedLevel);
					break;
			case 0x07:
					handle(15);
					do_speed(15,speedLevel);
			case 0x03:
					handle(20);
					do_speed(20,speedLevel);
					break;
					
			//left
			case 0x20:
					handle(-2);	//old -5
					do_speed(-2,speedLevel);
					break;
			case 0x60:
					handle(-10);//old -7
					do_speed(-10,speedLevel);
					break;
			case 0xe0:
					handle(-15);
					do_speed(-15,speedLevel);
					break;
			case 0xc0:
					handle(-20);
					do_speed(-20,speedLevel);
					break;				
		}	
	}
	
	getData((P7DR),47);					//debug-46
			//Ԥ��
	if(check_crossline()){	pattern = 30;return;	}
	if(check_blackArea()){	pattern = 40;return;	}
	pattern = 10;
	
}

int get_inner_speed(int degree,int speed_outer)
{
	speedLevel = speed_outer;
	
	if(degree>50)
		degree = 50;
	else if(degree<-50)
		degree = -50;
	else if(degree<0)
		degree = -1*degree;

	if(speed_outer>100)
		speed_outer = 100;
	
	return (int)speed_outer*((2*length-tan[degree]*width)/(2*length+tan[degree]*width));	
	
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

void do_speed(int degree,int speed_outer)
{
	if(degree>0)
		speed(speed_outer,get_inner_speed(degree,speed_outer));
	else
		speed(get_inner_speed(degree,speed_outer),speed_outer);

}

int get_near_std(int cur_degree)
{
	if(cur_degree>=0)
	{
		if(cur_degree>std4)
			return std4;
		else if(cur_degree>std3)
			return std3;
		else if(cur_degree>std2)
			return std2;
		else if(cur_degree>std1)
			return std1;
		else return std0;				
	}else
	{
		if(cur_degree<std_4)
			return std_4;
		else if(cur_degree<std_3)
			return std_3;
		else if(cur_degree<std_2)
			return std_2;
		else if(cur_degree<std_1)
			return std_1;
		else return std0;	
	}	
}
 void start()
{
	int i;
	isStart = 1;
	cnt0 = 0;
	
	cnt1 = 70;
	while(cnt1<=550)
	{
		//580ms
		speedLevel = 70;
		
		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
					handle(0);
					speed(speedLevel,speedLevel);
					break;
			case 0x04:
					handle(5);
					do_speed(5,speedLevel);
					break;
			case 0x06:
					handle(10);
					do_speed(-10,speedLevel);
					break;
			case 0x07:
					handle(15);
					do_speed(15,speedLevel);
			case 0x03:
					handle(20);
					do_speed(20,speedLevel);
					break;
					
			//left
			case 0x20:
					handle(-5);
					do_speed(-5,speedLevel);
					break;
			case 0x60:
					handle(-10);
					do_speed(-10,speedLevel);
					break;
			case 0xe0:
					handle(-15);
					do_speed(-15,speedLevel);
					break;
			case 0xc0:
					handle(-20);
					do_speed(-20,speedLevel);
					break;				
		}
	}
	pattern = 10;
	getData(P7DR,5); //debug5
	pushSw = 1;	
}

char * getInfo(int id,int dir)
{
	switch(id)
	{
		case 5: return "\n****************   start car over!     ****************\n\n";
		/**************���case*******************/
		case 21:
			if(dir<0) return "\n\n*******************************************************\n****************      left-turn        ****************\n*******************************************************\n\n[left-turn]: send out turn left -27 instruction";
			else return "\n\n*******************************************************\n****************      right-turn        ***************\n*******************************************************\n\n[right-turn]: send out turn left -27 instruction";
		case 22:
			if(dir<0) return "[left-turn]: send out turn left -27 instruction over \n\n[left-turn]: enter do-while  to regulate";
			else return "[right-turn]: send out turn right 27 instruction over \n\n[right-turn]: enter do-while  to regulate";
		case 23:
			if(dir<0) return "[left-turn]: left turn regulate over,leave function\n\n*******************************************************\n*****************    straight-line    *****************\n*******************************************************\n\n";
			else return "[right-turn]: right turn regulate over,leave function\n\n*******************************************************\n*****************    straight-line    *****************\n*******************************************************\n\n";

		
		/*****************ֱ��case***************************/
		case 31:
			return "\n\n*******************************************************\n******************      Angle        ******************\n*******************************************************\n\n[Angle]: run 100ms(deaf)";
		case 32:
			return "[Angle]: run 100ms(deaf) over \n\n[Angle]: regulate 300ms";
		case 33:
			return "[Angle]: regulate 300ms over \n\n[Angle]: waite for swagging state";
		case 34:
			if(dir<0) return "[left-angle]: detect left angle,send out speed and handle instruction \n\n[left-angle]: ignore 50ms to wait for all black";
			else return "[right-angle]: detect right angle,send out speed and handle instruction \n\n[right-angle]: ignore 50ms to wait for all black";
		case 35:
			if(dir<0) return "[left-angle]: ignore 50ms and all black come \n\n[left-angle]: wait for all black over";
			else return "[right-angle]: ignore 50ms and all black come \n\n[right-angle]: wait for all black over";
		case 36:
			if(dir<0) return "[left-angle]: all black over \n\n[left-angle]: swagging process,handle and speed adjust,wait for swagging over state";
			else return "[right-angle]: all black over \n\n[right-angle]: swagging process,handle and speed adjust,wait for swagging over state";
		case 37:
			if(dir<0) return "[left-angle]: swagging process over(state come) \n\n[left-angle]: regulate car body";
			else return "[right-angle]: swagging process over(state come) \n\n[right-angle]: regulate car body";
		case 38:
			if(dir<0) return "[left-angle]: regulate car body over,\n\n[left-angle]:increase speed to leave function\n\n*******************************************************\n*****************    straight-line    *****************\n*******************************************************\n\n";
			else return "[right-angle]: regulate car body over,\n\n[right-angle]:increase speed to leave function\n\n*******************************************************\n*****************    straight-line    *****************\n*******************************************************\n\n";

		/********************ä��***************************/
		case 41:
			if(dir<0) return "\n\n*******************************************************\n****************    left-blackArea     ****************\n*******************************************************\n\n[left-blackArea]:cross two while line";
			else return "\n\n*******************************************************\n****************   right-blackArea     ***************\n*******************************************************\n\n[right-blackArea]:cross two while line";
		case 42:
			if(dir<0) return "[left-blackArea]: two white line over\n\n[left-blackArea]: speed-down till all black";
			else return "[right-blackArea]: two white line over\n\n[right-blackArea]: speed-down till all black";	
		case 43:
			if(dir<0) return "[left-blackArea]: all-black come \n\n[left-blackArea]: cross till left-sensor touch left road";
			else return "[right-blackArea]: all-black come \n\n[right-blackArea]: cross till right-sensor touch right road";		
		case 44:
			if(dir<0) return "[left-blackArea]: left-sensor have touched left road \n\n[left-blackArea]: wait for first regulate state come";
			else return "[right-blackArea]: right-sensor have touched right road \n\n[right-blackArea]: wait for first regulate state come";		
		case 45:
			if(dir<0)return "[left-blackArea]: first regulate state come\n\n[left-blackArea]: begin first regualte";
			else return "[right-blackArea]: first regulate state come\n\n[right-blackArea]: begin first regualte";		
		case 46:
			if(dir<0) return "[left-blackArea]: first regulate over\n\n[left-blackArea]:steady and speed-up";
			else return "[right-blackArea]: first regulate over\n\n[right-blackArea]:steady and speed-up";		
		case 47:
			if(dir<0) return "[left-blackArea]: leave function\n\n*******************************************************\n*****************    straight-line    *****************\n*******************************************************\n\n";
			else return "[right-blackArea]: leave function\n\n*******************************************************\n*****************    straight-line    *****************\n*******************************************************\n\n";
       case 48:
    	    if(dir<0) return " error detect of [left-blackarea] ,return to rightangle ";
		  	else	return " error detect of [right-blackarea] ,return to rightangle";
			
    	/*		
		case 50: return "\n\ndetect end line\n\n";
		case 51: return "\n\nstart timer for  right turn\n\n";
		case 52: return "\n\ntime of turning right come,next turn 26\n\n";
		case 53: return "\n\nturn 26 finish,wait for increase degree and speed\n\n";
		case 54: return "\n\nincrease instruct send out,wait for turn right finish\n\n";
		case 55: return "\n\nturn right finish,next left turn\n\n";
		case 56: return "\n\nturn left -40 and delay 500ms finish,wait for end state\n\n";
		case 57: return "\n\nleft turn end state come,next regulate car body\n\n";
		case 58: return "\n\nregulate car body finish\n\n";
		case 59: return "finish ,wait for get data.\n\n";
		
		case 80:
			if(dir<0)	return "[left-storage]: find end line\n\n";
			else return "[right-storage]: find end line\n\n";
	  case 39:
			if(dir<0) return "[left-angle]:leave function\n";
			else return "[right-angle]:leave function\n";

		
		*/
	}
	return 0;
}

void stop()
{
	speed(0,0);
	while(1);
}
short int getDegree_(int degree)
{
	if(degree>=50)
		return -1*(degree-50);
	else return degree;
}
int getSpeed_(int speed)
{
	if(speed>=150)
		return -1*(speed-150);
	else return speed;
}
void getLightLed(unsigned char led)
{
	int i;	
	for(i=0;i<8;i++)
	{
		if(led&0x80)printf("%d",1);
		else printf(".");
		led <<=1;
	}
}

int getLightLedSum()
{
	int i;
	int sum = 0;
	unsigned char led = bit_change(P7DR);
	for(i=0;i<8;i++)
	{
		if(led&0x80)sum++;
		led <<=1;
	}
	return sum;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/
void left_blackArea3()
{
	//step0:��˫����
	int var ;
	var = 0;
	handle(0);
	cnt1=0;
	while(cnt1<100)
	{
        if(check_crossline()){    pattern = 30;return;  }
        if(var==0 && getLightLedSum()<=2)var++;
        if(var==1 && getLightLedSum()>2)var++;
        if(var==2 && getLightLedSum()<=2)var++;
        if(var==3)break;
	}
	getData((~P7DR),-47);                //debug-47

	speedLevel = 70;
	//step1:��⣬�ȶ���
	while(sensor_inp(MASK4_4))
	{
		if(check_crossline())		//��ֱ�ߣ�������Ϊä��
		{
			pattern = 30;
			getData(bit_change(P7DR),85); //debug85
			return;
		}
		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
                handle( 0 );
                speed( speedLevel ,speedLevel );
                break;
            case 0x04:
				handle(4);
				do_speed(4,speedLevel);
				break;
            case 0x06:
				handle(8);
				do_speed(8,speedLevel);
				break;
			case 0x07:
				handle(12);
				do_speed(12,speedLevel);
				break;
            case 0x03:
                handle( 20 );
                do_speed(20,speedLevel);
                break;
            case 0x20:
				handle(-4);
				do_speed(-4,speedLevel);
				break;
            case 0x60:
				handle(-8);
				do_speed(-8,speedLevel);
				break;
			case 0xe0:
				handle(-12);
				do_speed(-12,speedLevel);
				break;
            case 0xc0:
                handle( -20 );
                do_speed(-20,speedLevel);
                break;
			default:
				do_speed(speedLevel,speedLevel);
				break;
		}
	}

	//step2:ä������
	speed(0,0);
	handle(-22);
	speed(0,40);
	while(!(bit_change(P7DR)&0xf0))
	{
		handle(-22);
		do_speed(-22,70);
	}

	do_speed(25,80);
	handle(25);
	timer(40);

	while((sensor_inp(MASK3_3)!=0x20) && (sensor_inp(MASK3_3)!=0x60 )&& (sensor_inp(MASK3_3)!=0xc0))
//	while((sensor_inp(MASK4_4)!=0x10) && (sensor_inp(MASK4_4)!=0x30 ))
	{
		handle(15);
		do_speed(15,60);
	}
	handle(0);

	//step3:��ä����������
	for(speedLevel=60;speedLevel<90;speedLevel+=5)
	{
		if(check_crossline()){	pattern = 30;return;	}
		if(check_blackArea()){	pattern = 40;return;	}

		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
					handle(0);
					speed(speedLevel,speedLevel);
					break;
			case 0x04:
					handle(5);
					do_speed(5,speedLevel);
					break;
			case 0x06:
					handle(10);
					do_speed(-10,speedLevel);
					break;
			case 0x07:
					handle(15);
					do_speed(15,speedLevel);
			case 0x03:
					handle(20);
					do_speed(20,speedLevel);
					break;

			//left
			case 0x20:
					handle(-5);
					do_speed(-5,speedLevel);
					break;
			case 0x60:
					handle(-10);
					do_speed(-10,speedLevel);
					break;
			case 0xe0:
					handle(-15);
					do_speed(-15,speedLevel);
					break;
			case 0xc0:
					handle(-20);
					do_speed(-20,speedLevel);
					break;
		}
	}

	if(check_crossline()){	pattern = 30;return;	}
	if(check_blackArea()){	pattern = 40;return;	}
	pattern = 10;

}


void right_blackArea3()
{
	int var;
	var = 0;
	handle(0);
	cnt1=0;
	while(cnt1<100)
	{
        if(check_crossline()){    pattern = 30;return;  }
        if(var==0 && getLightLedSum()<=2)var++;
        if(var==1 && getLightLedSum()>2)var++;
        if(var==2 && getLightLedSum()<=2)var++;
        if(var==3)break;
	}
	getData(bit_change(P7DR),80); //debug80

	speedLevel = 70;
	while(sensor_inp(MASK4_4))
	{
		if(check_crossline())		//��ֱ�ߣ�������Ϊä��
		{
			pattern = 30;
			getData(bit_change(P7DR),85); //debug85
			return;
		}

		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
                handle( 0 );
                speed( speedLevel ,speedLevel );
                break;
            case 0x04:
				handle(4);
				do_speed(4,speedLevel);
				break;
            case 0x06:
				handle(8);
				do_speed(8,speedLevel);
				break;
			case 0x07:
				handle(12);
				do_speed(12,speedLevel);
				break;
            case 0x03:
                handle( 20 );
                do_speed(20,speedLevel);
                break;
            case 0x20:
				handle(-4);
				do_speed(-4,speedLevel);
				break;
            case 0x60:
				handle(-8);
				do_speed(-8,speedLevel);
				break;
			case 0xe0:
				handle(-12);
				do_speed(-12,speedLevel);
				break;
            case 0xc0:
                handle( -20 );
                do_speed(-20,speedLevel);
                break;
			default:
				do_speed(speedLevel,speedLevel);
		}
	}

	speed(40,0);
	handle(22);
	while(!(bit_change(P7DR)&0xf0))
	{
		handle(22);
		do_speed(22,70);
	}
	handle(-25);
	do_speed(-25,70);
	timer(40);

	while(sensor_inp(MASK3_3)!=0x04 && sensor_inp(MASK3_3)!=0x06 && sensor_inp(MASK3_3)!=0x07)
//	while(sensor_inp(MASK4_4)!=0x18 && sensor_inp(MASK4_4)!=0x1e)//=0x38)
	{		//���ǶȽ����ж�
		handle(-15);
		do_speed(-15,60);
	}
	handle(0);
	for(speedLevel=60;speedLevel<90;speedLevel+=5) //��ä�����ٶ�����
	{
		if(check_crossline()){	pattern = 30;return;}
		if(check_blackArea()){	pattern = 40;return;}

		switch(sensor_inp(MASK3_3))
		{
			case 0x00:
					handle(0);
					speed(speedLevel,speedLevel);
					break;
			case 0x04:
					handle(5);
					do_speed(5,speedLevel);
					break;
			case 0x06:
					handle(10);
					do_speed(-10,speedLevel);
					break;
			case 0x07:
					handle(15);
					do_speed(15,speedLevel);
			case 0x03:
					handle(20);
					do_speed(20,speedLevel);
					break;

			//left
			case 0x20:
					handle(-5);
					do_speed(-5,speedLevel);
					break;
			case 0x60:
					handle(-10);
					do_speed(-10,speedLevel);
					break;
			case 0xe0:
					handle(-15);
					do_speed(-15,speedLevel);
					break;
			case 0xc0:
					handle(-20);
					do_speed(-20,speedLevel);
					break;
		}
	}

	if(check_crossline()){	pattern = 30;return;}
	if(check_blackArea()){	pattern = 40;return;}
	pattern = 10;

}