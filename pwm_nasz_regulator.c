#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f10x_usart.h"
#include "deklaracje.h"




#define TIM2PERIOD 500
#define TIM2PRESCALER 72



void MotorDelay()
{
	int i;
	for(i=0; i < 10; i++)
		continue;
}

void MotorP(int start, int force)
{
//start=1 - napedzamy silnik
//start=0 - hamujemy silnik
//start=-1 -cofamy silnik
//force - jak mocno napedzamy lub hamujemy

	force = force*TIM2PERIOD/100;

	if (force >= TIM2PERIOD)
		force = TIM2PERIOD;
	if(force <= 0)
		force = -300;

	// force = force -20;
	double fp =  force_prescaler/100.0;


	if(start == 1 && fp >= 0.0 && fp <= 1.0) //!!! jesli chcemy preskalowac tylko jazdê do przodu to zamieniamy start != 0 na start == 1
	{
		force = force*fp;
	}

	if (start == 1)
	{
		TIM2->CCR4 = TIM2PERIOD + 10; // wartosc wieksza niz period
		GPIOA->BRR = GPIO_Pin_9; //wy³aczamy MOSFET-a od hamulca
		//odczekanie:
		MotorDelay();
		TIM2->CCR3 = TIM2PERIOD-force;

	}
	if (start == -1)
	{
		TIM2->CCR3 = TIM2PERIOD + 10;
		GPIOB ->BRR = GPIO_Pin_14 ; //wy³aczamy MOSFET-a od Power
		//odczekanie:
		MotorDelay();
		TIM2->CCR4 = TIM2PERIOD-force;
	}
	if (start == 0)
	{
		TIM2->CCR3 = TIM2PERIOD-force;
		//GPIOB ->BRR = GPIO_Pin_14 ; //wy³aczamy MOSFET-a od Power
		//odczekanie:
		//MotorDelay();
		TIM2->CCR4 = TIM2PERIOD-force;
	}
}

void MotorL(int start, int force)
{
//start=1 - napedzamy silnik
//start=0 - hamujemy silnik
//start=-1 -cofamy silnik
//force - jak mocno napedzamy lub hamujemy

	force = force*TIM2PERIOD/100;

	if (force >= TIM2PERIOD)
			force = TIM2PERIOD;
	if(force <= 0)
			force = -300;

	double fp =  force_prescaler/100.0;

	if(start == 1 && fp >= 0.0 && fp <= 1.0)  //!!! jesli chcemy preskalowac tylko jazdê do przodu to zamieniamy start != 0 na start == 1
	{
		force = force*fp;
	}

	if (start == 1)
	{
		TIM2->CCR2 = TIM2PERIOD + 10; // wartosc wieksza niz period
		GPIOA->BRR = GPIO_Pin_10; //wy³aczamy MOSFET-a od hamulca
		//odczekanie:
		MotorDelay();
		TIM2->CCR1 = TIM2PERIOD-force;

	}
	if (start == -1)
	{
		TIM2->CCR1 = TIM2PERIOD + 10;
		GPIOB ->BRR = GPIO_Pin_13 ; //wy³aczamy MOSFET-a od Power
		//odczekanie:
		MotorDelay();
		TIM2->CCR2 = TIM2PERIOD-force;
	}
	if (start == 0)
	{
			TIM2->CCR1 = TIM2PERIOD-force;
			//GPIOB ->BRR = GPIO_Pin_13 ; //wy³aczamy MOSFET-a od Power
			//odczekanie:
			//MotorDelay();
			TIM2->CCR2 = TIM2PERIOD-force;
	}
}





void InitMotors()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); //CCR1 -> PC6, CCR2->PC7


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	//TIM_DeInit(TIM3);

  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  	TIM_TimeBaseStructure.TIM_Period = TIM2PERIOD;
  	TIM_TimeBaseStructure.TIM_Prescaler = TIM2PRESCALER-1;
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //dodatkowy dzielnik; DIV1 = 1
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

//   Systemowy PWM
   	 // PWM1 Mode configuration: Channel 1,2,3,4
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; //TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM2PERIOD+10;	//off
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);


//	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	// TIM2 enable counter
  	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ClearFlag(TIM2, TIM_FLAG_CC1);
	TIM_ClearFlag(TIM2, TIM_FLAG_CC2);
	TIM_ClearFlag(TIM2, TIM_FLAG_CC3);
	TIM_ClearFlag(TIM2, TIM_FLAG_CC4);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);

   	TIM2->CNT = 0;

/*
   	//zerowe hamowanie
   	// zerowa predkosc jazdy
   	TIM2->CCR1 = TIM2PERIOD+10;
   	TIM2->CCR2 = TIM2PERIOD+10;
	TIM2->CCR3 = TIM2PERIOD+10;
	TIM2->CCR4 = TIM2PERIOD+10;
*/

//	TIM_CtrlPWMOutputs(TIM2, ENABLE); //output

   	TIM_Cmd(TIM2, ENABLE);

   	//NVIC_Config
   	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
   	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
   	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	//  I/O
	//inicjalizacja wyjœæ dopiero po inicjalizacji timera by byæ pewnym ¿e
	//nie bêdzie przypadkowego napiêcia na silniku


    //PWM

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE); 	// | RCC_APB2Periph_AFIO   enable alternate function
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//PWM output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIOB->BRR = GPIO_Pin_13 | GPIO_Pin_14; //reset output

	// Hamulec

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE); 	// | RCC_APB2Periph_AFIO   enable alternate function
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//PWM output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIOA->BRR = GPIO_Pin_9 | GPIO_Pin_10; //reset output hamulec OFF

}



void TIM2_IRQHandler(void)
{
	if( TIM2->SR & (uint16_t)TIM_FLAG_Update) //koniec cyklu PPM OUT
	{
		TIM2->SR = (uint16_t)~TIM_FLAG_Update;

		//brake PWM OFF
		GPIOA->BRR = GPIO_Pin_9 | GPIO_Pin_10; //reset output - hamulec OFF
		//motor PWM OFF
		GPIOB->BRR = GPIO_Pin_13 | GPIO_Pin_14; //reset output

		//GPIOC->BRR = GPIO_Pin_11 | GPIO_Pin_12;  //diody off

		return;
	}

	//Silnik lewy
	if( TIM2->SR & (uint16_t)TIM_FLAG_CC1) //compare match regular channels
	{
		TIM2->SR = (uint16_t)~TIM_FLAG_CC1; //clear pending interrupt

		if(TIM2->CCR1 < TIM2PERIOD)
		{
			//safety-brake OFF
			//GPIOA->BRR = GPIO_Pin_10; //reset output
			//motor ON
			GPIOB->BSRR = GPIO_Pin_13; //set output
			//led ON
			//GPIOC->BSRR = GPIO_Pin_11;
		}
		else
		{
			//motor OFF
			GPIOB->BRR = GPIO_Pin_13; //reset output
		}
		return;
	}

	//hamulec lewy
	if( TIM2->SR & (uint16_t)TIM_FLAG_CC2) //compare match regular channels
	{
		TIM2->SR = (uint16_t)~TIM_FLAG_CC2; //clear pending interrupt

		if(TIM2->CCR2 < TIM2PERIOD)
		{
			//safety: motor OFF
			//GPIOB->BRR = GPIO_Pin_13; //reset output
			//brake ON
			GPIOA->BSRR = GPIO_Pin_10; //set output
		}
		else
		{
			//brake OFF
			GPIOA->BRR = GPIO_Pin_10; //reset output
		}
 		return;
	}

	//Silnik prawy
	if( TIM2->SR & (uint16_t)TIM_FLAG_CC3) //compare match regular channels
	{
		TIM2->SR = (uint16_t)~TIM_FLAG_CC3; //clear pending interrupt


		if(TIM2->CCR3 < TIM2PERIOD)
		{
			//safety: brake OFF
			//GPIOA->BRR = GPIO_Pin_9; //reset output
			//MOTOR ON
			GPIOB->BSRR = GPIO_Pin_14; //set output
			//LED ON
			//GPIOC->BSRR = GPIO_Pin_12;
		}
		else
		{
			//MOTOR OFF
			GPIOB->BRR = GPIO_Pin_14; //reset output
		}
		return;
	}

	//hamulec prawy
	if( TIM2->SR & (uint16_t)TIM_FLAG_CC4) //compare match regular channels
	{
		TIM2->SR = (uint16_t)~TIM_FLAG_CC4; //clear pending interrupt
		if(TIM2->CCR4 < TIM2PERIOD)
		{
			//safety: MOTOR OFF
			//GPIOB->BRR = GPIO_Pin_14; //reset output
			//brake ON
			GPIOA->BSRR = GPIO_Pin_9; //set output
		}
		else
		{
			//brake OFF
			GPIOA->BRR = GPIO_Pin_9; //set output
		}
	 	return;
	}
}



