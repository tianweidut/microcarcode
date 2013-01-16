//*************************************************************************
//*************************************************************************
//**<������>������ģ�ͳ�          															 	           **
//**<�汾˵��>����1�棬�������(�ο��������ٷ���HT����)                                 **
//**<���л���>������H8-3048                                  **
//**<����>��tianweidut																		            	 **
//**<���ʱ��>��2009��10��19��																	           **
//**<��ϵ��ʽ>��E-mail:liutianweidlut@gmail.com;QQ:416774905��							 **
//*************************************************************************
//*************************************************************************


/*======================================*/
/* Include                              */
/*======================================*/
#include "includes.h"

/*======================================*/
/* var declarations               */
/*======================================*/

#define enSpeedMax	290	//����ٶȿ��ƣ����Ըĳɰ���ѡ��ķ�ʽ
#define enSpeedPuleMax	232		//pules count 290 = 232 * 1.25

/*======================================*/
/* Prototype declarations               */
/*======================================*/
void myInit();

void main(void)
{
	vInitialize();
	myInit();
	
	while(!pushsw_get())
	{
		if(cnt1<100)
			led_out(0x1);
		else if(cnt1<200)
			led_out(0x2);
		else	cnt1 = 0;
	}
	cnt1 = 0;
	
	/*while(startbar_get())
	{
		if(cnt1<50)
			led_out(0x1);
		else if(cnt1<100)
			led_out(0x2);
		else	cnt1 = 0;
	}*/
	
	led_out(0);
	start();
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
							case LEFT:
									handle(-1*midDegree_st4);
									left_turn();
									break;
							case RIGHT:
									handle(midDegree_st4);
									right_turn();
									break;
							default:break;
						}
						break;


			case 30:	//ֱ��

						rightAngle1();						
						break;


			case 40:	//ä��
						switch(blackArea_direction)
						{
							case LEFT:
									left_blackArea1();
									break;
							case RIGHT:
									right_blackArea1();
									break;
							default:break;									
						}
						break;

			default:break;

		}//end switch

	}//end while
	
}
 
void myInit(void)
{
	if((~P6DR)&0x08) isDebug = 1;	
	else isDebug = 0;
 	
	lastDegree = 0;
	pushSw = 0;
	turnSpeed = 95;
//	turnSpeed = 100 - 2*(dipsw_get()&0x07);
}
#pragma interrupt( interrupt_timer0 )
void interrupt_timer0( void )
{
    ITU0_TSR &= 0xfe;                   /* Flag clear                   */
   
    cnt0++;
    cnt1++;
	cnt2++;
	timerCnt++;

	if((eSpeedNum) == timerCnt)	//20,ʱ����
	{
		//��ʱ����һ���ٶȵļ���
		eCurSpeed=(double)((iPulesNumAll*abvSpeed)/10);	//�ٶȼ��㹫ʽ:
															//				      ������     ��  �ܳ�
															//		ʵʱ�ٶ�=---------------------------
		
		//speedPules = iPulesNumAll;		//pules --> speed 												//					һȦ�������� �� �ɼ�ʱ��
		//��λ
		iPulesNumAll = 0;	//��λ������Ŀ
		timerCnt = 0;		//��λʱ����Ŀ
		
	}//if

	//if((int)speedPules > 232)
	
	if((int)eCurSpeed > enSpeedMax)
	{

		ITU4_BRA=(unsigned long)(PWM_CYCLE-1)*(20)/100;
		ITU3_BRB=(unsigned long)(PWM_CYCLE-1)*(20)/100;
		for(speedBrake_var=200;speedBrake_var!=0;speedBrake_var--);
	}


	if(isStart)
	{
		if(isDebug)
		{
			if(!printNow)
			{
				if((~PBDR&0x01) && pushSw){ isStart = 0;isDebug = 0;printNow = 1;}//�н���;����������
				curLedValue = P7DR;
				if(curLedValue!=lastLedValue)
					getData(curLedValue,0);
				else if(bit_change(P7DR)==0xff && cnt0>800)	//���Գ���  before p7dr=0xff
				{
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
			}else if(bit_change(P7DR) ==0xff && cnt0>800)	isStop = 1;// before p7dr=0xff
		}
	}


}

#pragma interrupt( interrupt_capture )
void interrupt_capture( void )////////////////////////////////////////////
{
	ISR &=	0xfe;		//�����־�Ĵ���				
	iPulesNumAll++;		//�������:
	//�˴���Ҫ������ĿԽ������
	


}