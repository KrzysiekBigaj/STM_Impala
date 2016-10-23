//interrups.c
#include "stm32f10x.h"
#include "systick.h"
#include "ee_ram.h"
#include "ee_var.h"
#include "mixer.h"
#include "ppm_io.h"

#pragma O0
//#pragma OTime
/*******************************************************************************
* Function Name  : EXTI_Configuration
* Description    : Configures the different EXTI lines.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PPM_EXTI_Config(void);
void PPM_NVIC_Config(void);
void PPM_NVIC_Config1(FunctionalState enable);

extern unsigned short sbusIn[17];
//-------------------------------------------------
//parallel or serial #1
volatile unsigned int serialMap[EE_MAP_COUNT] = {0,1,2,3,4,5,6,7,8,9,10,11}; //configuration only from PC
//serial #2
volatile unsigned int serialMap2[EE_MAP_COUNT] = {0,1,2,3,4,5,6,7,8,9,10,11}; //configuration only from PC
int mapRSSI;

volatile unsigned int ppm_end[PPM_SER_LIMIT+1] = {0,0,0,0,0,0,0,0,0,0,0,0,0};  //private, internal timestamp
unsigned int ppm_raw_in[PPM_SER_LIMIT];		 //raw input in uS
int ppm_in[PPM_SER_LIMIT] = {0,0,0,-500,0,-500,0,0,0,0,0,0};	//normalised, +/-500us range, th=min, mode=off	
volatile int ppm_in_present=0; 
volatile int ppm_in2_present=0; 
volatile int sbus_present=0;
volatile int serialPPM = PSM_PPM; //0-parallel
volatile int ch_out[5] = {0,0,0,-512, 0}; //kana³y wyjœciowe z autopilota, moga wykraczaæ poza +/-512, przed obciêciem przez EPA 
int ppm_zero[7];					 //zero (raw) - common for both #1 and #2
//pozwala okresliæ (bitowo) które kana³y PPM IN maj¹ sygna³ 
//zerujemy zmienn¹ i po czasie > 20ms badamy stan bitów b0..b6 (IN 1..7)
//mo¿na te¿ u¿yæ do synchronizacji we/wy itd.
volatile int ap_mode = AP_NOPPMOUT; //AP_DIRECT; //	 

volatile int ppm_outidx = 9999; //no out
volatile int ppm_outidx2 = 9999; //no out
volatile short ppm_comp[PPM_SER_LIMIT] = {0,0,0,0,0,0,0,0,0,0,0,0};	//values 1000...2000us

//PPM internal limit (nominal values)
#define PPM_MIN 1000
#define PPM_MAX 2000
#define PPM_ZERO 1500
//PPM processing tolerance
#define PPM_MMIN 800
#define PPM_MMAX 2300


short epa_min[8];
short epa_max[8];

extern unsigned short sbusOut[17];

#define PPM_OUT_FRAME 18000 //below 1400 some servos (HS81/SkySurfer) start vibrating
volatile unsigned int tm_frames = 0;	//in PPM_OUT_FRAME units

int GetChOut(int idx)
{
	return ch_out[idx];
}
int GetOutForSBUS(int idx)
{
//translate zero centered ch_out data (before EPA, but nominal range -512..512) into 0..2047.
//do not apply EPA to SBUS
	int i = 1024 + ch_out[idx]*2;
	if( i < 0)
		return 0;
	if(i > 2047)
		return 2047;	 
	return i;
}
int GetInForSBUS(int idx)
{
	//translate zero centered input data (nominal range -512..512 but not limited to it) into 0..2047.
	int i = 1024 + ppm_in[idx]*2;
	if( i < 0)
		return 0;
	if(i > 2047)
		return 2047;	 
	return i;
}


int PPM_in_present()
{
	if(serialPPM & PSM_SBUS)
		return sbus_present;
	
	if(serialPPM & PSM_CPPM2 )
		return ppm_in2_present;
	else
		return ppm_in_present;
}

void Clear_ppm_state()
{
	ppm_in_present = 0;
	ppm_in2_present = 0;
	sbus_present = 0;
}


//these functions are unsafe due to overflow limits
static unsigned int _Get_us()
{
	unsigned int tb, tc;
	//avoid timer (tmus) update during data processing
	do
	{
		tb = tm_frames;
		tc = TIM8->CNT;
	}while(tb != tm_frames);
	
	return tb*PPM_OUT_FRAME+tc; //32bit in us, up to 1hour
}

unsigned int TimeAnchor()
{
	return _Get_us();
}
//up to 1,19 hour delay, no overflow problem but returns difference modulo (1,19 hour)
unsigned int TimeDiff_us(unsigned int anchor)
{
	unsigned int us = _Get_us();
	if( us < anchor)
		return 0xffffffff - anchor + us; 
	return us - anchor;
}
//up to 1,19 hour delay, no overflow problem but returns difference modulo (1,19 hour)
unsigned int TimeDiff_ms(unsigned int anchor) 
{
	unsigned int us = _Get_us();
	if( us < anchor)
		us =  (0xffffffff - anchor + us)/1000; 	//tested, passed :-)
	else
		us = (us - anchor)/1000;
	return us;
}


void Wait_ms(unsigned int ms)
{
	
	unsigned int tm = TimeAnchor();
	while( TimeDiff_ms(tm) < ms)
		; //wait for time lapse
}
void Wait_us(unsigned int us)
{
	
	unsigned int tm = TimeAnchor();
	while( TimeDiff_us(tm) < us)
		; //wait for time lapse
}


//access to normalised PPM input
//PPM_Get will be called first to get raw values!
int PPM_RawIn(int idx)
{
	if(serialPPM == PSM_SBUS) 
		return sbusIn[idx]/2 + 1000;  //range 1000..2048, close to PPM 
	return ppm_raw_in[idx];
}

int PPM_In(int idx)
{
	return ppm_in[idx];
}



int PPM_In0(int idx)
{
	return ppm_in[idx] - ppm_zero[idx];
}


int PPM_Zero(int idx)
{
	return ppm_zero[idx];
}


#define PPM_ALL_OUT_PINS  (/*GPIO_Pin_2 |*/ GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_14)
#define PPM_ALL_OUT2_PINS (GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_6 /*GPIO_Pin_9 | GPIO_Pin_8 |*/ )

void PPM_IO_Config(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//TIM_ICInitTypeDef  TIM_ICInitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 						 

//PPM_IN
	if( serialPPM ) //SBUS nie przeszkadza
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_12;	//IN1 & IN6
	else
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//PPM OUT: GPIOC 13,14,5,4,3,2
	GPIO_InitStructure.GPIO_Pin = PPM_ALL_OUT_PINS;
	if( serialPPM )	//równiez SBUS
		GPIO_InitStructure.GPIO_Pin |= PPM_ALL_OUT2_PINS; //IN7 jako redirected output to OSD, IN2..5 as auxiliary outputs
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//clear outputs
	if( serialPPM ) //równiez SBUS
		GPIOC->BRR = PPM_ALL_OUT2_PINS;
	GPIOC->BRR = PPM_ALL_OUT_PINS;
 	//TIM_ClearITPendingBit() to samo co TIM_ClearFlag()
/*	
	//watchdog timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
	TIM_DeInit(TIM7);
	TIM_TimeBaseStructure.TIM_Period = 10000;					//1sec no response (no reset)				
	TIM_TimeBaseStructure.TIM_Prescaler= 36000;				    //1ms   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV2; 			
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);							    
	TIM7->CNT = 0;
//	TIM_ITConfig(TIM7,	TIM_IT_Update, ENABLE);
//	TIM_Cmd(TIM7, ENABLE);
*/

	//TIM8_Config
	if(serialPPM)
	{
	  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE); 	//enable GOPIOC & alternate function						 
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9; // 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//PWM output
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	}
			
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
	TIM_DeInit(TIM8);

  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  	TIM_TimeBaseStructure.TIM_Period = PPM_OUT_FRAME;
  	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    // PWM1 Mode configuration: Channel 3,4 
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	//off
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
   	
	// TIM8 enable counter 
  	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);							    
	TIM_ClearFlag(TIM8, TIM_FLAG_CC1);							    

	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);

   	TIM8->CNT = 0;	
	TIM8->CCR1 = PPM_OUT_FRAME+100;

	if(serialPPM)
 		TIM_CtrlPWMOutputs(TIM8, ENABLE); //output
	else
 		TIM_CtrlPWMOutputs(TIM8, DISABLE); //output

/*
	//works, but not tested and published
	if(serialPPM & 2)
	{
		TIM_ICStructInit(&TIM_ICInitStructure); 
	   	//Initialize input capture structure: Ch2 
	   	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2; 
	   	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising; 
	   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	   	TIM_ICInitStructure.TIM_ICFilter    = 15;  //should be set by experiments 
	   	TIM_ICInit(TIM8, &TIM_ICInitStructure);
	   
	   	TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
	 	TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE); 
	}
	else
*/
	 	TIM_ITConfig(TIM8, TIM_IT_CC2, DISABLE); 

 	TIM_ITConfig(TIM8, TIM_IT_CC3 | TIM_IT_CC4, DISABLE); 

   	TIM_Cmd(TIM8, ENABLE);





	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
	TIM_DeInit(TIM6);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;					
	TIM_TimeBaseStructure.TIM_Prescaler= 72-1;				    //1us   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);							    
	TIM6->CNT = 0;
	TIM_Cmd(TIM6, ENABLE);
}



/*
void ReloadWatchdog()
{
	TIM7->CNT = 0;
}

void TIM7_IRQHandler(void)
{
	//watchdog interrupt
	if( TIM7->SR & (uint16_t)TIM_FLAG_Update) //koniec cyklu PPM OUT
	{
		//set mode to OFF
		ap_mode = AP_DIRECT; //direct transfer I/O during interrupts
		TIM7->SR = (uint16_t)~TIM_FLAG_Update;
	}
}
*/

void PPM_EXTI_Config(void)
{
  	EXTI_InitTypeDef EXTI_InitStructure;

	FunctionalState parEnable = serialPPM ? DISABLE : ENABLE; //0x03  disable for all serial inputs 

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //ENABLES EXTI	
	EXTI_DeInit();

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
  	EXTI_ClearITPendingBit(EXTI_Line6);
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  	EXTI_InitStructure.EXTI_LineCmd = parEnable;
  	EXTI_Init(&EXTI_InitStructure);

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);
  	EXTI_ClearITPendingBit(EXTI_Line8);
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  	EXTI_InitStructure.EXTI_LineCmd = parEnable;
  	EXTI_Init(&EXTI_InitStructure);

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
  	EXTI_ClearITPendingBit(EXTI_Line9);
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
  	EXTI_InitStructure.EXTI_LineCmd = parEnable;
  	EXTI_Init(&EXTI_InitStructure);

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);
  	EXTI_ClearITPendingBit(EXTI_Line10);
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  	EXTI_InitStructure.EXTI_LineCmd = parEnable;
  	EXTI_Init(&EXTI_InitStructure);

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);
  	EXTI_ClearITPendingBit(EXTI_Line11);
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  	EXTI_InitStructure.EXTI_LineCmd = parEnable;
  	EXTI_Init(&EXTI_InitStructure);

	if( serialPPM & PSM_CPPM) //1 
	{
		//at input #1
	  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
	  	EXTI_ClearITPendingBit(EXTI_Line12);
	  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  	EXTI_Init(&EXTI_InitStructure);
	}
	else
	{
	  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
	  	EXTI_ClearITPendingBit(EXTI_Line12);
	  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	  	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	  	EXTI_InitStructure.EXTI_LineCmd = parEnable; //enable only in parallel mode
	  	EXTI_Init(&EXTI_InitStructure);
	}


	if( serialPPM & PSM_CPPM2) 
	{
		//at input #6
	  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
	  	EXTI_ClearITPendingBit(EXTI_Line7);
	  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  	EXTI_Init(&EXTI_InitStructure);
	}
	else
	{
	  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
	  	EXTI_ClearITPendingBit(EXTI_Line7);
	  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	  	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	  	EXTI_InitStructure.EXTI_LineCmd = parEnable; //enable only in parallel mode
	  	EXTI_Init(&EXTI_InitStructure);
	}



	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_ClearITPendingBit(EXTI_Line7);
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearITPendingBit(EXTI_Line9);
	EXTI_ClearITPendingBit(EXTI_Line10);
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
}


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PPM_NVIC_Config(void)
{
  	NVIC_InitTypeDef NVIC_InitStructure; 

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;	//	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//najwyzsze
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 
	NVIC_Init(&NVIC_InitStructure); 
		
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;	//	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//najwyzsze
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

	//timer8 CCR1 output compare
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure); 

	//timer8 overflow
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure); 
	
	//re-set USB priority to 3
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //ni¿sze od PPM i/o
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

/*
	//timer7 - watchdog 
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure); 
*/
}


int GetPPMforOSD(int idx)
{
	//input channel 'idx' is validated and mapped to 'in" internal index,
	int val;
	if(serialPPM==0)
	{
		if(idx < 6)
			return ppm_raw_in[idx];
		else
			return 0;
	}
	
	if(idx == IDX_RSSI)
	{
		if( mapRSSI == 255)
			return 0; //no mapping exists
		idx = mapRSSI;
	}
	else
		idx = serialMap[idx];
	
	if(serialPPM == PSM_SBUS)
	{
		val = (sbusIn[idx]>>1)+1000;	//1000-2000
	}
	else
	{
		 val = ppm_end[idx];	
		//kompensujemy "przekrecenie" licznika
		if(val < 0)
			val += PPM_OUT_FRAME;	
	}
	//safety margin
	if( val > PPM_MMIN && val < PPM_MMAX ) 
		return val;
	return 0; //invalid;
}


void GetMap(int idx, int in)
{
	//input channel 'idx' is validated and mapped to 'in" internal index,
	int val;
	//docelowo analiza diversity i wybór kana³u który ma dane i nie ma failsafe

	if(serialPPM == PSM_SBUS)
	{
		val = (sbusIn[idx]>>1)-512;	//-152..512
		ppm_in[in] = val ; // +/-500 nominal, -700...+800 MMIN/MMAX
		ppm_raw_in[in] = val + PPM_ZERO; //emulate PPM range
		return;
	}
	else
	{
		 val = ppm_end[idx];	
		//"przekrecenie" licznika TIM8 daje z³y wynik w "end" dla parallel
		//korygujemy b³êdny wynik 
		if(val < 0)
			val += PPM_OUT_FRAME;	
	}
	//safety margin in case of missing or invalid timestamp, or invalid PPM pulse 
	//szerszy zakres, na wypadek rozszerzonego zakresu z nadajnika
	if( val > PPM_MMIN && val < PPM_MMAX ) 
	{
		//val = (ppm_raw_in[i] * (PPM_FILTER-1) + val)/ PPM_FILTER;  bez filtowania!
		ppm_raw_in[in] = val;
		ppm_in[in] = val - PPM_ZERO; // +/-500 nominal, -700...+800 MMIN/MMAX
	}
	else
	{
		//else zerujemy ppm_in_state - impuls by³ fa³szywy 
		ppm_in_present &= ~(1<<in);
		//else zostaje poprzedni stan, mo¿na by³oby zliczac b³êdy etc.
	}
}

//--------------------------------------------------------------------------------------------
void PPM_Get()
{
	//this function will be called before use of PPM inputs
	//it calculates PPM raw & normalized input values 
	if( !serialPPM)
	{
		//parallel
		//moze byc dowolne mapowanie 6 kana³ów, 7-y idzie bezposrednio do OSD
		GetMap(0, 0);
		GetMap(1, 1);
		GetMap(2, 2);
		GetMap(3, 3);
		GetMap(4, 4);
		GetMap(5, 5);
		GetMap(6, 6);
	}
	else
	{
		//all CPPM/SBUS modes
		//mapowanie zaleznie od rodzaju nadajnika
		GetMap(serialMap[0], 0);
		GetMap(serialMap[1], 1);
		GetMap(serialMap[2], 2);
		GetMap(serialMap[3], 3);
		GetMap(serialMap[4], 4);
		GetMap(serialMap[5], 5);
		GetMap(serialMap[6], 6); //for diagnostics only (OSD menu)
	}
}


void EXTI9_5_IRQHandler(void)
{
	if( TIM8->SR & (uint16_t)TIM_FLAG_CC1) //compare match regular channels
		GPIOC->BRR = PPM_ALL_OUT_PINS | PPM_ALL_OUT2_PINS;		//all PPM outputs low 

	if( serialPPM & PSM_CPPM2) //2==at input #6
	{
		static int ppm_idx2 = 0; //volatile in module?
		register int len = TIM6->CNT;

		if( (EXTI->PR & EXTI_Line7) ) //just to be sure - there will be only one PPM IRQ source
		{
			EXTI->PR = EXTI_Line7; //clear pending interrupt

			TIM6->CNT = 0;  				//rozpoczynamy kolejny pomiar
	
			if(len > 3000 ) //at least 3ms - sync pulse 
			{
				ppm_in2_present = (~0); //all channels present >>(32-ppm_idx);	//present channels
				ppm_idx2 = 0;	//close cycle
			}
			else
				if(ppm_idx2 < PPM_SER_LIMIT)
					ppm_end[ppm_idx2++] = len;
		}
		return;
	}

	//parallel inputs
	if( (EXTI->PR & EXTI_Line6) /* && (EXTI->IMR & EXTI_Line6) */)
	{
		static unsigned short b6 = 0;
		if( GPIOC->IDR & GPIO_Pin_6 ) 
			b6 = TIM8->CNT;  				//stan wysoki - rozpoczynamy pomiar
		else 
		{
			ppm_end[6] =  TIM8->CNT - b6;  // stan niski - konczymy pomiar		
			ppm_in_present |= (1<<6);
		}
		EXTI->PR = EXTI_Line6;
	}
	if( (EXTI->PR & EXTI_Line7)  /*&& (EXTI->IMR & EXTI_Line7) */)
	{
		static unsigned short b7 = 0;
		if( GPIOC->IDR & GPIO_Pin_7 ) 
			b7 = TIM8->CNT;  				//stan wysoki - rozpoczynamy pomiar
		else 
		{
			ppm_end[5] =  TIM8->CNT - b7;  // stan niski - konczymy pomiar		
			ppm_in_present |= (1<<5);
		}
		EXTI->PR = EXTI_Line7;
	}

	if( (EXTI->PR & EXTI_Line8)  /*&& (EXTI->IMR & EXTI_Line8) */)
	{
		static unsigned short b8 = 0;
		if( GPIOC->IDR & GPIO_Pin_8 ) 
			b8 = TIM8->CNT;  				//stan wysoki - rozpoczynamy pomiar
		else 
		{
			ppm_end[4] =  TIM8->CNT - b8;  // stan niski - konczymy pomiar		
			ppm_in_present |= (1<<4);
		}
		EXTI->PR = EXTI_Line8;
	}

	if( (EXTI->PR & EXTI_Line9)  /*&& (EXTI->IMR & EXTI_Line9) */)
	{
		static unsigned short b9 = 0;
		if( GPIOC->IDR & GPIO_Pin_9 ) 
			b9 = TIM8->CNT;  				//stan wysoki - rozpoczynamy pomiar
		else 
		{
			ppm_end[3] =  TIM8->CNT - b9;  // stan niski - konczymy pomiar		
			ppm_in_present |= (1<<3);
		}
		EXTI->PR = EXTI_Line9;
	}
}


void EXTI15_10_IRQHandler(void)
{
	//stop PPM out to reduce its jitter
	if( TIM8->SR & (uint16_t)TIM_FLAG_CC1) //compare match regular channels
		GPIOC->BRR = PPM_ALL_OUT_PINS | PPM_ALL_OUT2_PINS;		//all PPM outputs low 

	if( serialPPM & PSM_CPPM) //1==at input #1
	{
		//Otime: 42/43 cykle +24irq; 1us jitter 
		static int ppm_idx=0; //volatile in module?
		register int len;
		if( (EXTI->PR & EXTI_Line12) ) //just to be sure - there will be only one PPM IRQ source
		{
			EXTI->PR = EXTI_Line12; //clear pending interrupt
			len = TIM6->CNT;
			TIM6->CNT = 0;  				//rozpoczynamy kolejny pomiar

			if(len > 3000 ) //at least 3ms - sync pulse 
			{
				ppm_in_present = (~0); //all channels present >>(32-ppm_idx);	//present channels
				ppm_idx = 0;	//close cycle
			}
			else
				if(ppm_idx < PPM_SER_LIMIT)
					ppm_end[ppm_idx++] = len;

			return;
		}
	}

	//parallel input
	if( (EXTI->PR & EXTI_Line10)  /*&& (EXTI->IMR & EXTI_Line10)*/ )
	{
		static unsigned short b10 = 0;
		if( GPIOC->IDR & GPIO_Pin_10 ) 
			b10 = TIM8->CNT;  				//stan wysoki - rozpoczynamy pomiar
		else 
		{
			ppm_end[2] =  TIM8->CNT - b10;  // stan niski - konczymy pomiar		
			ppm_in_present |= (1<<2);
		}
		EXTI->PR = EXTI_Line10;
	}

	if( (EXTI->PR & EXTI_Line11)  /*&& (EXTI->IMR & EXTI_Line11)*/ )
	{
		static unsigned short b11 = 0;
		if( GPIOC->IDR & GPIO_Pin_11 ) 
			b11 = TIM8->CNT;  				//stan wysoki - rozpoczynamy pomiar
		else 
		{
			ppm_end[1] =  TIM8->CNT - b11;  // stan niski - konczymy pomiar		
			ppm_in_present |= (1<<1);
		}
		EXTI->PR = EXTI_Line11;
	}

	if( (EXTI->PR & EXTI_Line12)  /*&& (EXTI->IMR & EXTI_Line12)*/ )
	{
		static unsigned short b12 = 0;
		if( GPIOC->IDR & GPIO_Pin_12 ) 
			b12 = TIM8->CNT;  				//stan wysoki - rozpoczynamy pomiar
		else 
		{
			ppm_end[0] =  TIM8->CNT - b12;  // stan niski - konczymy pomiar		
			ppm_in_present |= (1<<0);
		}
		EXTI->PR = EXTI_Line12;
	}
}





extern int mix_F;	//mikser klapolotek, wyklucza siê z delta, 0-brak
extern int mix_D;

//mapa pinów kolejnych wyjœæ ppm
static unsigned int const  ch[9] =  {  //in flash
	GPIO_Pin_13,
	GPIO_Pin_14,
	GPIO_Pin_5,
	GPIO_Pin_4,
	GPIO_Pin_3,
	//tylko dla TIM8
	GPIO_Pin_6,  //I/O 7 - OSD menu
	GPIO_Pin_11, //IN2 as "serial" output Aux2
	GPIO_Pin_10, //IN3 as "serial" output Aux3
};
/*
static unsigned int const  ch2[6] =  { //in flash
	//tylko dla serial PPM input
	GPIO_Pin_6,  //I/O 7 - OSD menu
	GPIO_Pin_11, //IN2 as "serial" output Aux2
	GPIO_Pin_10, //IN3 as "serial" output Aux3
	//PWM
	GPIO_Pin_9,  //IN4 as "serial" output Aux4
	GPIO_Pin_8,  //IN5 as "serial" output Aux5
	0
};
*/

void TIM8_UP_IRQHandler(void)
{
	//low priority interrupt to change PWM duty
	if( TIM8->SR & (uint16_t)TIM_FLAG_Update) //koniec cyklu PPM OUT
	{
		static int c9  = 0;
		static int c10 = 0;
		if( serialPPM & (PSM_CPPM | PSM_CPPM2) )	//all CPPM modes. 
		{
			int cmp = ppm_end[serialMap[9]]; //zamapowane wejœcia serial AUX3
			if( (cmp >= PPM_MMIN) && (cmp <= PPM_MMAX) ) //range check 800-2300 us
			{
				c9 += (cmp-c9)/2;	//filter 2..4
				TIM8->CCR4 = c9;	//PWM
			}
			cmp = ppm_end[serialMap[10]]; //zamapowane wejœcia serial AUX4
			if( (cmp >= PPM_MMIN) && (cmp <= PPM_MMAX) ) //range check 800-2300 us
			{
				c10 += (cmp-c10)/2;	  //filter 2..4
			//	if( (c10 >= PPM_MMIN) && (c10 <= PPM_MMAX) ) //range check 800-2300 us
				TIM8->CCR3 = c10;	//PWM
			}
		}
		else
			if( serialPPM & PSM_SBUS )	//SBUS mode. 
			{
				int cmp = sbusIn[serialMap[9]]; //zamapowane wejœcia serial AUX3
				if( cmp < 2047 ) //range check 
				{
					TIM8->CCR4 = (cmp >> 1) + 1000;	//PWM
				}
				cmp = sbusIn[serialMap[10]]; //zamapowane wejœcia serial AUX4
				if( cmp < 2047 ) //range check 
				{
					TIM8->CCR3 = (cmp >> 1) + 1000;	//PWM
				}
			}
		tm_frames++;
		if(ap_mode != AP_NOPPMOUT) //disable - ppm_outidx will go over max channel index
		{ 
			ppm_outidx = -1;
			TIM8->CCR1 = TIM8->CNT + 100; //first out will be after 100uS
		}
		TIM8->SR = (uint16_t)~TIM_FLAG_Update;
	}
}

void TIM8_CC_IRQHandler()
{
	GPIOC->BRR = PPM_ALL_OUT_PINS | PPM_ALL_OUT2_PINS;		//all PPM outputs low 
	if( TIM8->SR & (uint16_t)TIM_FLAG_CC1) //compare match regular channels
	{
		//baardzo d³ugi kod
		//metod¹ na przyspieszenie by³oby stablicowanie wskaŸników na funkcje obs³uguj¹ce poszczególne kana³y i jeden circular pointer na aktualn¹ funkcjê 
		register int cmp = ++ppm_outidx;
		TIM8->SR = (uint16_t)~TIM_FLAG_CC1; //clear pending interrupt

		if(serialPPM)
		{
			if( cmp  > 8) //past all channels
				return;

			if( ap_mode == AP_DIRECT || cmp > 4) //direct, fast in->out transfer  or "aux" channels
			{
				if(cmp > 4)	//skip mode channel mapping
					cmp++;
				else
				{
					if(cmp == 4 && !(mix_D | mix_F) )
						cmp = 0; //kabel Y dla lotka2
				}	 
				if(serialPPM == PSM_SBUS)
					cmp = (sbusIn[serialMap[cmp]]>>1) + 1000; //zamapowane wejœcia serial 	
				else
					cmp = ppm_end[serialMap[cmp]]; //zamapowane wejœcia serial 	
			}
			else //STAB/AUTO mode, only regular channels 0..4 (ch1..Ch5)
			{
				cmp = ppm_comp[cmp]; //ppm_comp is in valid order, no remap
			}
			if( (cmp < PPM_MMIN) || (cmp > PPM_MMAX) ) //range check 800-2300 us
			{
				//invalid pulse - ignore
				TIM8->CCR1 = TIM8->CNT + 100;	//will generate COMPARE MATCH interrupt after 'n' pulse  length 
				return;
			}  
			__disable_irq();
			TIM8->CCR1 = TIM8->CNT + cmp;		//will generate COMPARE MATCH interrupt after 'n' pulse  length 
			GPIOC->BSRR = ch[ppm_outidx]; 	 	//appropriate output high
			__enable_irq();
			return;
		}
		
		//parallel mode				
		if( cmp >= 5) //past all channels
			return;

		if( ap_mode == AP_DIRECT)
		{
			if(cmp == 4 && !(mix_D | mix_F) )
				cmp = 0; 		//kabel Y dla lotka2 
			cmp = ppm_end[cmp];	//bez mapowania dane z wejœcia
			if(cmp < 0)
				cmp += PPM_OUT_FRAME;	//overflow
		}
		else  //regular channel STAB, AUTO
		{
			cmp = ppm_comp[cmp]; //bez mapowania
		}
		if( (cmp < PPM_MMIN) || (cmp > PPM_MMAX) ) //range check 800-2300 us
		{
			//invalid - ignore
			TIM8->CCR1 = TIM8->CNT + 100;	//will generate COMPARE MATCH interrupt after 'n' pulse  length 
			return;
		}  

		__disable_irq();
		TIM8->CCR1 = TIM8->CNT + cmp;	//will generate COMPARE MATCH interrupt after 'n' pulse  length 
		GPIOC->BSRR = ch[ppm_outidx]; //appropriate output high
		__enable_irq();
		return;
	}

	if( TIM8->SR & (uint16_t)TIM_FLAG_CC2) //compare match input capture
	{
/*	  it works well, but not published yet 
		static int ppm_idx2 = 0; //volatile in module?
		static int prv;
		register int len = (int)TIM8->CCR2 - prv;
		prv = TIM8->CCR2;
		if(len <= 0)
			len = PPM_OUT_FRAME+len;

		if(len > 3000 ) //at least 3ms - sync pulse 
		{
			ppm_in2_present = (~0); //all channels present >>(32-ppm_idx);	//present channels
			ppm_idx2 = 0;	//close cycle
		}
		else
			if(ppm_idx2 < PPM_SER_LIMIT)
				ppm_end[ppm_idx2++] = len;
*/
		TIM8->SR = (uint16_t)~TIM_FLAG_CC2; //clear pending interrupt
	}
}



void PPM_Out(int idx, int val) //-500..500
{
/*
	register int v = val+val+1024;
	if(v < 0)
		v = 0;
	if(val > 2047)
		v = 2047;
	sbusOut[idx] = v;	
*/
	ch_out[idx] = val;  
	val += 1500;
	if(val < epa_min[idx]) val = epa_min[idx];
	if(val > epa_max[idx]) val = epa_max[idx];
	ppm_comp[idx] = val; 
}

void PPM_Out0(int idx, int val) //val -500..500 for regular channels and 0..1000 for throttle
{
	PPM_Out(idx, val + ppm_zero[idx]);
}

void StoreTrim(void)
{
#define TRIM_CNT 10

	int i, j;
	unsigned int tm;
	for(j=0; j < 7; j++)
		ppm_zero[j] = 0;		 //

	for(i=0; i< TRIM_CNT; i++)
	{
		PPM_Get();
		for(j=0; j < 7; j++)
			ppm_zero[j] += ppm_in[j];		 //normalised input in range (-500(-700?)...500(800) ) uS
		//tm = Get_ms();
		//while( (Get_ms() - tm) < 24)
		//	;//wait for new frame
		tm = TimeAnchor();
		while( TimeDiff_ms(tm) < 25 )
			continue; //wait for new frame 
	}
	for(j=0; j < 7; j++)
	{
		ppm_zero[j] /= TRIM_CNT;		 //raw input in uS
		if( j == PPM_TH && ppm_zero[j] > -250)
			ppm_zero[j] = -250;		//ograniczenie na max trymera gazu +25% (idle dla spaliny) 
		EE_Write( EE_TRIM+j, ppm_zero[j]);
	}
}

void RestoreTrim(void)
{
	int j;
	for(j=0; j < 7; j++)
	{
		ppm_zero[j] = EE_Read( EE_TRIM+j, j == PPM_TH ? -500 : 0); //gaz min, reszta srodek
	}
	if( ppm_zero[PPM_TH] > -250)
		ppm_zero[j] = -250;		//ograniczenie na max trymera gazu +25% (idle dla spaliny) 
}

void RestoreMapPPM(void)
{
	//mapowanie wejœæ SERIAL do odpowiednich "funkcji" AP
	//A dok³adniej w tablicy jest przypisanie któremu kana³owi wewnêtrznemu (idx) odpowiada który kana³ serial (value)
	int j;
	for(j=0; j < EE_MAP_COUNT; j++) 
	{
		serialMap[j] = (unsigned int)EE_Read( EE_MAP+j, j); 
		if(	serialMap[j] > PPM_SER_LIMIT) //max serial channel allowed
			serialMap[j] = PPM_SER_LIMIT; 
	}
	//here map other functions/inputs 
	//to serial channels individually, up to PPM_MAP_COUNT stored

	serialPPM = EE_Read( EE_PPM_SERIAL, 0) & 0x07; //default parallel
	mapRSSI = EE_Read( EE_MAP_RSSI, 255); //default unsassigned
}

void ValidateEpa(int i)
{
	if( epa_min[i] < PPM_MMIN) epa_min[i] = PPM_MMIN;
	if( epa_min[i] > 1250) epa_min[i] = 1250;
	if( epa_max[i] > PPM_MMAX) epa_max[i] = PPM_MMAX;
	if( epa_max[i] < 1750) epa_max[i] = 1750;
}

void RestoreEPA(void)
{
	int i;
	for(i=0; i<=7; i++)
	{
		int v = EE_Read(EE_EPA+i, (2000<<16) + 1000);
		epa_min[i] = v & 0x0FFFF;
		epa_max[i] = v >> 16;
/*
		if( (epa_max[i] - epa_min[i]) < 500 ) 
		{
			epa_min[i] = 1000;
			epa_max[i] = 2000; 
		}	 
*/
		//same validation in usb_irq_redirect
		ValidateEpa(i);
	}
}
