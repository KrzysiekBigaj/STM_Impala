#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "deklaracje.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f10x_usart.h"



/* -----------------------------------------------------------------------------------------
 * Inicjalizacja timera ktory determinuje czas trwania cyklu pomiarowego
 * -----------------------------------------------------------------------------------------
 */


void InitTimer1(void)
{

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;  // liczy w us
  	TIM_TimeBaseStructure.TIM_Period = 1000*Tp;   //okres w ms
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //dodatkowy dzielnik; DIV1 = 1
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


  	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // obsluguje przerwania
   	TIM1->CNT = 0;

   	TIM_Cmd(TIM1, ENABLE);

 	//NVIC_Config

   	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
   	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
   	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void TIM1_UP_IRQHandler(void)
{
	TIM1->SR = (uint16_t)~TIM_FLAG_Update; //clear pending interrupt
	cycle++;
}


