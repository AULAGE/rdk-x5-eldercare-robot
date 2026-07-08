#include "include.h"
//����:�������ֻ������Ƽ����޹�˾
//���ǵĵ���:lobot-zone.taobao.com



#define ADC_BAT		13		//��ص�ѹ��AD���ͨ��
// static bool UartBusy = FALSE;

u32 gSystemTickCount = 0;	//ϵͳ�����������ڵĺ�����

uint8 BuzzerState = 0;
uint16 Ps2TimeCount = 0;

uint16 BatteryVoltage;

static u8  fac_us=0;//us��ʱ������
static u16 fac_ms=0;//ms��ʱ������
//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void InitDelay(u8 SYSCLK)
{
//	SysTick->CTRL&=0xfffffffb;//bit2���,ѡ���ⲿʱ��  HCLK/8
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SYSCLK/8;
	fac_ms=(u16)fac_us*1000;
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864
void DelayMs(u16 nms)
{
	u32 temp;
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����
}
//��ʱnus
//nusΪҪ��ʱ��us��.
void DelayUs(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us; //ʱ�����
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ����
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����
}





void InitLED(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PA�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //LED0-->PC.2 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void InitKey(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void InitBuzzer(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void InitTimer2(void)		//100us
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = (10 - 1); //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =(720-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
	    TIM2, //TIM2
	    TIM_IT_Update  |  //TIM �ж�Դ
	    TIM_IT_Trigger,   //TIM �����ж�Դ
	    ENABLE  //ʹ��
	);

	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}

void InitADC(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE);	   //ʹ��ADC1ͨ��ʱ��

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //72M/6=12,ADC���ʱ�䲻�ܳ���14M
	//PA0/1/2/3 ��Ϊģ��ͨ����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_DeInit(ADC1);  //������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת���������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���



	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1

	ADC_ResetCalibration(ADC1);	//����ָ����ADC1��У׼�Ĵ���

	while(ADC_GetResetCalibrationStatus(ADC1));	//��ȡADC1����У׼�Ĵ�����״̬,����״̬��ȴ�

	ADC_StartCalibration(ADC1);		//��ʼָ��ADC1��У׼״̬

	while(ADC_GetCalibrationStatus(ADC1));		//��ȡָ��ADC1��У׼����,����״̬��ȴ�

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1������ת����������
}


uint16 GetADCResult(BYTE ch)
{
	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADCͨ��3,�������˳��ֵΪ1,����ʱ��Ϊ239.5����

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1������ת����������

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}


void CheckBatteryVoltage(void)
{
	uint8 i;
	uint32 v = 0;
	for(i = 0;i < 8;i++)
	{
		v += GetADCResult(ADC_BAT);
	}
	v >>= 3;
	
	v = v * 2475 / 1024;//adc / 4096 * 3300 * 3(3�����Ŵ�3������Ϊ�ɼ���ѹʱ�����ѹ��)
	BatteryVoltage = v;

}

uint16 GetBatteryVoltage(void)
{//��ѹ����
	return BatteryVoltage;
}

void Buzzer(void)
{//�ŵ�100us�Ķ�ʱ�ж�����
	static bool fBuzzer = FALSE;
	static uint32 t1 = 0;
	static uint32 t2 = 0;
	if(fBuzzer)
	{
		if(++t1 >= 2)
		{
			t1 = 0;
			BUZZER = !BUZZER;//2.5KHz
		}
	}
	else
	{
		BUZZER = 0;
	}

	
	if(BuzzerState == 0)
	{
		fBuzzer = FALSE;
		t2 = 0;
	}
	else if(BuzzerState == 1)
	{
		t2++;
		if(t2 < 5000)
		{
			fBuzzer = TRUE;
		}
		else if(t2 < 10000)
		{
			fBuzzer = FALSE;
		}
		else
		{
			t2 = 0;
		}
	}
}

BOOL manual = FALSE;
void TIM2_IRQHandler(void)   //TIM2�ж�
{//��ʱ��2�ж�  100us
	static uint32 time = 0;
	static uint16 timeBattery = 0;
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���ָ����TIM�жϷ������:TIM �ж�Դ
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);    //���TIMx���жϴ�����λ:TIM �ж�Դ
		
		
		Buzzer();
		if(++time >= 10)
		{
			time = 0;
			gSystemTickCount++;
			Ps2TimeCount++;
			if(GetBatteryVoltage() < 6400)//С��6.4V����
			{
				timeBattery++;
				if(timeBattery > 5000)//����5��
				{
					BuzzerState = 1;
				}
			}
			else
			{
				timeBattery = 0;
				if(manual == FALSE)
				{
					BuzzerState = 0;
				}
			}
		}
	}
}

void TaskTimeHandle(void)
{
	static uint32 time = 10;
	static uint32 times = 0;
	if(gSystemTickCount > time)
	{
		time += 10;
		times++;
		if(times % 2 == 0)//20ms
		{
			ServoPwmDutyCompare();
		}
		if(times % 50 == 0)//500ms
		{
			CheckBatteryVoltage();
		}
	}
	
}

int16 BusServoPwmDutySet[8] = {500,500,500,500,500,500,500,500};
uint8 i;
void TaskRun(u8 ps2_ok)
{
	static bool Ps2State = FALSE;
	static uint8 mode = 0;
	uint16 ly, rx,ry;
	uint8 PS2KeyValue;
	static uint8 keycount = 0;
	TaskTimeHandle();
	
	
	TaskPCMsgHandle();
	TaskBLEMsgHandle();
	TaskRobotRun();

	if(KEY == 0)
	{
		DelayMs(60);
		{
			if(KEY == 0)
			{
				keycount++;
			}
			else
			{
				if (keycount > 20)
				{
					keycount = 0;
					FullActRun(100,0);
					return;
				}
				else
				{
					keycount = 0;
					LED = ~LED;
					FullActRun(100,1);	
				}
			}
		}
	}
	if (ps2_ok == 0)
	{
		if(Ps2TimeCount > 50)
		{
			Ps2TimeCount = 0;
			PS2KeyValue = PS2_DataKey();
			if(mode == 0)
			{
				if( PS2_Button( PSB_SELECT ) & PS2_ButtonPressed( PSB_START ) )
				{
					FullActStop();  //ֹͣ����������
					ServoSetPluseAndTime( 1, 1500, 1000 );  //����е�۵Ķ����ת��1500��λ��
					ServoSetPluseAndTime( 2, 1500, 1000 );
					ServoSetPluseAndTime( 3, 1500, 1000 );
					ServoSetPluseAndTime( 4, 1500, 1000 );
					ServoSetPluseAndTime( 5, 1500, 1000 );
					ServoSetPluseAndTime( 6, 1500, 1000 );
					for (i = 1; i < 7; i++)
					{
						BusServoPwmDutySet[i] = 500;
						BusServoCtrl(i,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[i],1000);
					}
					mode = 1;
					Ps2State = 1;
					manual = TRUE;
					BuzzerState = 1;
					LED=~LED;
					DelayMs(80);
					manual = FALSE;
					DelayMs(50);
					manual = TRUE;
					BuzzerState = 1;
					DelayMs(80);
					manual = FALSE;
					LED=~LED;
				}
				else
				{
					if(PS2KeyValue && !PS2_Button(PSB_SELECT))
				{
					LED=~LED;
				}
							
					switch( PS2KeyValue )
							{
								//���ݰ��µİ��������ƶ��ת��
								case PSB_PAD_LEFT:
									ServoSetPluseAndTime( 6, ServoPwmDutySet[6] + 20, 50 );
									BusServoPwmDutySet[6] = BusServoPwmDutySet[6] + 10;
									if (BusServoPwmDutySet[6] > 1000)
										BusServoPwmDutySet[6] = 1000;
									BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
									break;
								case PSB_PAD_RIGHT:
									ServoSetPluseAndTime( 6, ServoPwmDutySet[6] - 20, 50 );
									BusServoPwmDutySet[6] = BusServoPwmDutySet[6] - 10;
									if (BusServoPwmDutySet[6] < 0)
										BusServoPwmDutySet[6] = 0;
									BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
									break;
								case PSB_PAD_UP:
									ServoSetPluseAndTime( 5, ServoPwmDutySet[5] + 20, 50 );
									BusServoPwmDutySet[5] = BusServoPwmDutySet[5] - 10;
									if (BusServoPwmDutySet[5] < 0)
										BusServoPwmDutySet[5] = 0;
									BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);
									break;
								case PSB_PAD_DOWN:
									ServoSetPluseAndTime( 5, ServoPwmDutySet[5] - 20, 50 );
									BusServoPwmDutySet[5] = BusServoPwmDutySet[5] + 10;
									if (BusServoPwmDutySet[5] > 1000)
										BusServoPwmDutySet[5] = 1000;
									BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);
									break;
								case PSB_L1:
									ServoSetPluseAndTime( 2, ServoPwmDutySet[2] + 20, 50 );
									BusServoPwmDutySet[2] = BusServoPwmDutySet[2] + 10;
									if (BusServoPwmDutySet[2] > 1000)
										BusServoPwmDutySet[2] = 1000;
									BusServoCtrl(2,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[2],50);
									break;
								case PSB_L2:
									ServoSetPluseAndTime( 1, ServoPwmDutySet[1] + 20, 50 );
									BusServoPwmDutySet[1] = BusServoPwmDutySet[1] + 10;
									if (BusServoPwmDutySet[1] > 1000)
										BusServoPwmDutySet[1] = 1000;
									BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[1],50);
									break;
								case PSB_TRIANGLE:
									ServoSetPluseAndTime( 4, ServoPwmDutySet[4] - 20, 50 );
									BusServoPwmDutySet[4] = BusServoPwmDutySet[4] + 10;
									if (BusServoPwmDutySet[4] > 1000)
										BusServoPwmDutySet[4] = 1000;
									BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
									break;
								case PSB_CROSS:
									ServoSetPluseAndTime( 4, ServoPwmDutySet[4] + 20, 50 );
									BusServoPwmDutySet[4] = BusServoPwmDutySet[4] - 10;
									if (BusServoPwmDutySet[4] < 0)
										BusServoPwmDutySet[4] = 0;
									BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
									break;
								case PSB_R1:
									ServoSetPluseAndTime( 2, ServoPwmDutySet[2] - 20, 50 );
									BusServoPwmDutySet[2] = BusServoPwmDutySet[2] - 10;
									if (BusServoPwmDutySet[2] < 0)
										BusServoPwmDutySet[2] = 0;
									BusServoCtrl(2,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[2],50);
									break;
								case PSB_R2:
									ServoSetPluseAndTime( 1, ServoPwmDutySet[1] - 20, 50 );
									BusServoPwmDutySet[1] = BusServoPwmDutySet[1] - 10;
									if (BusServoPwmDutySet[1] < 0)
										BusServoPwmDutySet[1] = 0;
									BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[1],50);
									break;
								case PSB_CIRCLE:
									ServoSetPluseAndTime( 3, ServoPwmDutySet[3] + 20, 50 );
									BusServoPwmDutySet[3] = BusServoPwmDutySet[3] + 10;
									if (BusServoPwmDutySet[3] > 1000)
										BusServoPwmDutySet[3] = 1000;
									BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
									break;
								case PSB_SQUARE:
									ServoSetPluseAndTime( 3, ServoPwmDutySet[3] - 20, 50 );
									BusServoPwmDutySet[3] = BusServoPwmDutySet[3] - 10;
									if (BusServoPwmDutySet[3] < 0)
										BusServoPwmDutySet[3] = 0;
									BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
									break;
								case PSB_START:
									ServoSetPluseAndTime( 1, 1500, 1000 );
									ServoSetPluseAndTime( 2, 1500, 1000 );
									ServoSetPluseAndTime( 3, 1500, 1000 );
									ServoSetPluseAndTime( 4, 1500, 1000 );
									ServoSetPluseAndTime( 5, 1500, 1000 );
									ServoSetPluseAndTime( 6, 1500, 1000 );
									for (i = 1; i < 7; i++)
									{
										BusServoPwmDutySet[i] = 500;
										BusServoCtrl(i,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[i],1000);
									}
									break;
								default:
									if (PS2_AnologData(PSS_LX) == 255)
									{
										ServoSetPluseAndTime( 3, ServoPwmDutySet[3] + 30, 60 );
										BusServoPwmDutySet[3] = BusServoPwmDutySet[3] + 10;
										if (BusServoPwmDutySet[3] > 1000)
											BusServoPwmDutySet[3] = 1000;
										BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
									}
									if (PS2_AnologData(PSS_LX) == 0)
									{
										ServoSetPluseAndTime( 3, ServoPwmDutySet[3] - 30, 60 );
										BusServoPwmDutySet[3] = BusServoPwmDutySet[3] - 10;
										if (BusServoPwmDutySet[3] < 0)
											BusServoPwmDutySet[3] = 0;
										BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
									}
									if (PS2_AnologData(PSS_RY) == 0)
									{
										LED=~LED;
										ServoSetPluseAndTime( 4, ServoPwmDutySet[4] + 30, 60 );
										BusServoPwmDutySet[4] = BusServoPwmDutySet[4] + 10;
										if (BusServoPwmDutySet[4] > 1000)
											BusServoPwmDutySet[4] = 1000;
										BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
									}
									if (PS2_AnologData(PSS_RY) == 255)
									{
										LED=~LED;
										ServoSetPluseAndTime( 4, ServoPwmDutySet[4] - 30, 60 );
										BusServoPwmDutySet[4] = BusServoPwmDutySet[4] - 10;
										if (BusServoPwmDutySet[4] < 0)
											BusServoPwmDutySet[4] = 0;
										BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
									}
									if (PS2_AnologData(PSS_LY) == 0)
									{
										ServoSetPluseAndTime( 5, ServoPwmDutySet[5] - 30, 60 );
										BusServoPwmDutySet[5] = BusServoPwmDutySet[5] - 10;
										if (BusServoPwmDutySet[5] < 0)
											BusServoPwmDutySet[5] = 0;
										BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);
									}
									if (PS2_AnologData(PSS_LY) == 255)
									{
										ServoSetPluseAndTime( 5, ServoPwmDutySet[5] + 30, 60 );
										BusServoPwmDutySet[5] = BusServoPwmDutySet[5] + 10;
										if (BusServoPwmDutySet[5] > 1000)
											BusServoPwmDutySet[5] = 1000;
										BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);	
									}
									if (PS2_AnologData(PSS_RX) == 0)
									{
										ServoSetPluseAndTime( 6, ServoPwmDutySet[6] + 30, 60 );
										BusServoPwmDutySet[6] = BusServoPwmDutySet[6] + 10;
										if (BusServoPwmDutySet[6] > 1000)
											BusServoPwmDutySet[6] = 1000;
										BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
									}
									if (PS2_AnologData(PSS_RX) == 255)
									{
										ServoSetPluseAndTime( 6, ServoPwmDutySet[6] - 30, 60 );
										BusServoPwmDutySet[6] = BusServoPwmDutySet[6] - 10;
										if (BusServoPwmDutySet[6] < 0)
											BusServoPwmDutySet[6] = 0;
										BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
									}
								}
				}
			}
			else
			{	
				 if( PS2_Button( PSB_SELECT ) && PS2_ButtonPressed( PSB_START ) )  //����ǲ��� SELECT��ť����ס��Ȼ����START��ť�� �ǵĻ����л�ģʽ
					{
						mode = 0; //��ģʽ��Ϊ0�� �Ͷ�����ģʽ
						Ps2State = 0;  //�����־��
						manual = TRUE;
						BuzzerState = 1;
						LED=~LED;
						DelayMs(80);
						manual = FALSE;
						DelayMs(50);
						manual = TRUE;
						BuzzerState = 1;
						DelayMs(80);
						manual = FALSE;
						LED=~LED;
					}
					else
					{
				if(PS2KeyValue && !Ps2State && !PS2_Button(PSB_SELECT))
				{
					LED=~LED;
				}

				switch(PS2KeyValue)
				{
					case 0:
						if(Ps2State)
						{
							Ps2State = FALSE;
						}
						break;
					
					case PSB_START:
						if(!Ps2State)
						{
							FullActRun(0,1);
						}
						Ps2State = TRUE;
						break;
					
					case PSB_PAD_UP:
						if(!Ps2State)
						{
							FullActRun(1,1);
						}
						Ps2State = TRUE;
						break;
					
					case PSB_PAD_DOWN:
						if(!Ps2State)
						{
							FullActRun(2,1);
						}
						Ps2State = TRUE;
						break;
					
					case PSB_PAD_LEFT:
						if(!Ps2State)
						{
						FullActRun(3,1);
					}
					Ps2State = TRUE;
					break;
					
				case PSB_PAD_RIGHT:
					if(!Ps2State)
					{
						FullActRun(4,1);
					}
					Ps2State = TRUE;
					break;

				case PSB_TRIANGLE:
					if(!Ps2State)
					{
						FullActRun(5,1);
					}
					Ps2State = TRUE;
					break;
					
				case PSB_CROSS:
					if(!Ps2State)
					{
						FullActRun(6,1);
					}
					Ps2State = TRUE;
					break;
					
				case PSB_SQUARE:
					if(!Ps2State)
					{
						FullActRun(7,1);
					}
					Ps2State = TRUE;
					break;
					
				case PSB_CIRCLE:
					if(!Ps2State)
					{
						FullActRun(8,1);
					}
					Ps2State = TRUE;
					break;

				case PSB_L1:
					if(!Ps2State)
					{
						FullActRun(9,1);
					}
					Ps2State = TRUE;
					break;
					
				case PSB_R1:
					if(!Ps2State)
					{
						FullActRun(10,1);
					}
					Ps2State = TRUE;
					break;
					
				case PSB_L2:
					if(!Ps2State)
					{
						FullActRun(11,1);
					}
					Ps2State = TRUE;
					break;
					
				case PSB_R2:
					if(!Ps2State)
					{
						FullActRun(12,1);
					}
					Ps2State = TRUE;
					break;
			}
		}
		}
	}
}
}
