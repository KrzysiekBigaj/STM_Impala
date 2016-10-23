#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
// Moja biblioteka
#include "deklaracje.h"

/* -----------------------------------------------------------------------------------------
 * Inicjalizacja PWM do turbiny
 * -----------------------------------------------------------------------------------------
 */


void InitTurbine(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	//  I/O
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 	// | RCC_APB2Periph_AFIO   enable alternate function
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//PWM output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM4, ENABLE); //CCR1 -> PC6, CCR2->PC7

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);

  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  	TIM_TimeBaseStructure.TIM_Period = 15000;   //wymagane przez silniczki w Impali beta
  	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;  ////wymagane przes silniczki w Impali beta
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //dodatkowy dzielnik; DIV1 = 1
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM1 Mode configuration: Channel 4
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;	//off
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM4, &TIM_OCInitStructure);


	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


	// TIM4 enable counter
  	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ClearFlag(TIM4, TIM_FLAG_CC4);


	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC4, DISABLE);


   	TIM4->CNT = 0;
   	//wartosci
	TIM4->CCR4 = 1000;


	TIM_CtrlPWMOutputs(TIM4, ENABLE); //output
// 	TIM_ITConfig(TIM5, TIM_IT_CC4 | TIM_IT_CC2, ENABLE);
   	TIM_Cmd(TIM4, ENABLE);
}




