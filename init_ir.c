#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "deklaracje.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f10x_usart.h"
//#include "ppm_io.h"
#include "stm32f10x_exti.h"
#include "ee_ram.h"



void InitTimer8(void)
{

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);

  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  	TIM_TimeBaseStructure.TIM_Prescaler = 4050-1;  // liczy w us
  	TIM_TimeBaseStructure.TIM_Period = 1300;   //okres w ms
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //dodatkowy dzielnik; DIV1 = 1
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);


  	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); // obsluguje przerwania
   	TIM8->CNT = 0;

   	TIM_Cmd(TIM8, ENABLE);

 	//NVIC_Config
   	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
   	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
   	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void TIM8_UP_IRQHandler(void)
{
	// if( TIM8->SR == OVERFLOW)
	TIM8->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the TIM Counter


	iridx |= IRREADY; //safety: sprawdzenie czy wejscie do funkcji IrReceive() odbylo sie przez nacisniecie przycisku na pilocie

	TIM8->CNT = 0;
	TIM8->SR = (uint16_t)~TIM_FLAG_Update; //clear pending interrupt
// 	TIM_Cmd(TIM8, DISABLE);
}

void init_EXTI (void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;


  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //ENABLES EXTI
  	//EXTI_DeInit();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
  	EXTI_ClearITPendingBit(EXTI_Line7);
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;	//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//najwyzsze
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI9_5_IRQHandler(void)
{
	if( TIM8->CNT == 0) //first time, pierwsze zbocze, timer zablokowany
		{
		    TIM8->CR1 |= TIM_CR1_CEN; //Enable the TIM Counter
			iridx = 0;
		}
		else //timer dzia³a, jestesmy w czasie analizy impulsów
		{
			if(iridx < MAX_IR)
			{
				irbuf[iridx++] = TIM8->CNT;
			}
		}

	EXTI->PR = EXTI_Line7; //clear pending interrupt
}


int Get8(int offs)
{
	int a = 0;
	int c;
	int i;
	for(i = 0; i < 8; i++)
	{
		c = irbuf[i+offs] - irbuf[i+offs-1];

		//bit 0 nominal 20
		//bit 1 nominal 40
		if(c > 18 && c < 22)
		{
			a = a >> 1;
			continue;
		}
		else if(c > 36 && c < 44)
		{
			a = (a >> 1) | 0x80;
			continue;
		}
		else //error
			return -1;
	}
	return a;
}

void IrReceive()
{
	int a=0;
	int b=0;


	if(iridx != 289) // !(iridx & IRREADY) )
	{
		return;
	}
	ir_valid = 0;
	iridx = 0;

	//start pulse: nominal 240
	if( irbuf[0] < 220 || irbuf[0] > 260)
	{
		//error
		return;
	}
	a = Get8(1);
	b = Get8(9);
	b=b^0xFF;//negacja
	if(a != b || a < 0)
	{
		return;
	}

	ir_addr = a;

	a = Get8(17);
	b = Get8(25);
	b=b^0xFF;//negacja
	if(a != b || a < 0)
		{
			return;
		}

	ir_data = a;

	ir_valid = 1; //OK
	led_on = 1;

	//EE_Write(id_ir_data, ir_data );
	//EE_Write(id_ir_addr, ir_addr );
}

//---------------------------------------------------------------------------------
/*
irda button data list

name  ir_data
CH-   69
CH    70
CH+   71
|<<   68
>>|   64
>||   67
-     7
+     21
EQ    9
0     22
100+  25
200+  13
1 -   12
2 -   24
3 -   94
4 -   8
5 -   28
6 -   90
7 -   66
8 -   82
9 -   74
*/
//-------------------------------------------------------------------------------

void IrControl()
{
	switch(ir_data)
	{
	/*
		case 12: //button 1
			remote_control = 1;
		break;

		case 24: //button 2
			remote_control = 2;
		break;

		case 94: //button 3
			remote_control = 3;
		break;

		case 8: //button 4
			remote_control = 4;
		break;

		case 28: //button 5
			remote_control = 5;
		break;

		case 90: //button 6
			remote_control = 6;
		break;

		case 66: //button 7
			remote_control = 7;
		break;

		case 82: //button 8
			remote_control = 8;
		break;

		case 74: //button 9
			remote_control = 9;
		break;
*/
		case 67: //button >||
			remote_control = 'w';
		break;

		case 71: //button CH+
			remote_control = 's';
		break;

		case 9: //button EQ
			Calibrate();
			//remote_control = 'c';
		break;

/*

		case 22: //button 0
			if(remote_control == 1)
			{
				Kp = 0;
				//EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = 0;
				//EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = 0;
				//EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = 0;
				//EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = 0;
				//EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = 1000;
				//EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border = 0;
				//EE_Write(id_border, border);
			}
		break;


		case 25: //button 100+
			if(remote_control == 1)
			{
				Kp = Kp+100;
				//EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = Ki+100;
				//EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = Kd+100;
				//EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = Kl+100;
				//EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = force_prescaler+5;
				//EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = turbine_power+100;
				//EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border =border+100;
				//EE_Write(id_border, border);
			}
		break;

		case 13: //button 200+

			if(remote_control == 1)
			{
				Kp = Kp+200;
			//	EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = Ki+200;
			//	EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = Kd+200;
			//	EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = Kl+200;
			//	EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = force_prescaler+10;
			//	EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = turbine_power+200;
			//	EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border =border+200;
			//	EE_Write(id_border, border);
			}
		break;

		case 21: //button +

			if(remote_control == 1)
				{
					Kp = Kp+5;
				//	EE_Write(id_Kp, Kp );
				}
				if(remote_control == 2)
				{
					Ki = Ki+5;
				//	EE_Write(id_Ki, Ki );
				}
				if(remote_control == 3)
				{
					Kd = Kd+5;
				//	EE_Write(id_Kd, Kd );
				}
				if(remote_control == 4)
				{
					Kl = Kl+5;
				//	EE_Write(id_Kl, Kl );
				}
				if(remote_control == 5)
				{
					force_prescaler = force_prescaler+5;
				//	EE_Write(id_force_prescaler, force_prescaler );
				}
				if(remote_control == 6)
				{
					turbine_power = turbine_power+5;
				//	EE_Write(id_turbine_power, turbine_power );
				}
				if(remote_control == 7)
				{
					border =border+5;
				//	EE_Write(id_border, border);
				}
				break;

		case 7: //button -

			if(remote_control == 1)
			{
				Kp = Kp-5;
			//	EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = Ki-5;
			//	EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = Kd-5;
			//	EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = Kl-5;
			//	EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = force_prescaler-5;
			//	EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = turbine_power-5;
			//	EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border =border-5;
			//	EE_Write(id_border, border);
			}
		break;


		case 64: //button >>|
			if(remote_control == 1)
			{
				Kp = Kp+20;
			//	EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = Ki+20;
			//	EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = Kd+20;
			//	EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = Kl+20;
			//	EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = force_prescaler+20;
			//	EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = turbine_power+20;
			//	EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border =border+20;
			//	EE_Write(id_border, border);
			}
		break;

		case 68: //button |<<
			if(remote_control == 1)
			{
				Kp = Kp-20;
			//	EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = Ki-20;
			//	EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = Kd-20;
			//	EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = Kl-20;
			//	EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = force_prescaler-20;
			//	EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = turbine_power-20;
			//	EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border =border-20;
			//	EE_Write(id_border, border);
			}
		break;

		case 70: //button CH
			if(remote_control == 1)
			{
				Kp = Kp+100;
			//	EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = Ki+100;
			//	EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = Kd+100;
			//	EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = Kl+100;
			//	EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = force_prescaler+100;
			//	EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = turbine_power+100;
			//	EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border =border+100;
			//	EE_Write(id_border, border);
			}
		break;


		case 69: //button CH-
			if(remote_control == 1)
			{
				Kp = Kp-100;
			//	EE_Write(id_Kp, Kp );
			}
			if(remote_control == 2)
			{
				Ki = Ki-100;
			//	EE_Write(id_Ki, Ki );
			}
			if(remote_control == 3)
			{
				Kd = Kd-100;
			//	EE_Write(id_Kd, Kd );
			}
			if(remote_control == 4)
			{
				Kl = Kl-100;
			//	EE_Write(id_Kl, Kl );
			}
			if(remote_control == 5)
			{
				force_prescaler = force_prescaler-100;
			//	EE_Write(id_force_prescaler, force_prescaler );
			}
			if(remote_control == 6)
			{
				turbine_power = turbine_power-100;
			//	EE_Write(id_turbine_power, turbine_power );
			}
			if(remote_control == 7)
			{
				border =border-100;
			//	EE_Write(id_border, border);
			}
		break;
	*/
	}
	/*
	switch(ir_data)
		{
			case 27: //menu
				remote_control = 1;
			break;

			case 24: //play
				remote_control = 2;
			break;

			case 25: //aparat
				remote_control = 3;
			break;

			case 26: //kamera
				remote_control = 4;
			break;

			case 20: //plus w lupie
				remote_control = 5;
			break;

			case 22: //strza³ka do gory
				remote_control = 6;
			break;

			case 21: //minus w lupie
				Calibrate();
			break;

			case 23: // otwarta klodka
				remote_control = 'w';
			break;

			case 5: //zamknieta klodka
				remote_control = 's';
			break;


			case 16: //strzalka w lewo
				if(remote_control == 1)
				{
					Kp = Kp+5;
					//EE_Write(id_Kp, Kp );
				}
				if(remote_control == 2)
				{
					Ki = Ki+5;
					//EE_Write(id_Ki, Ki );
				}
				if(remote_control == 3)
				{
					Kd = Kd+50;
					//EE_Write(id_Kd, Kd );
				}
				if(remote_control == 4)
				{
					Kl = Kl+20;
					//EE_Write(id_Kl, Kl );
				}
				if(remote_control == 5)
				{
					force_prescaler = force_prescaler+5;
					//EE_Write(id_force_prescaler, force_prescaler );
				}
				if(remote_control == 6)
				{
					turbine_power = turbine_power+50;
					//EE_Write(id_turbine_power, turbine_power );
				}

			break;

			case 17: //ok

				if(remote_control == 1)
				{
					Kp = Kp+10;
				//	EE_Write(id_Kp, Kp );
				}
				if(remote_control == 2)
				{
					Ki = Ki+10;
				//	EE_Write(id_Ki, Ki );
				}
				if(remote_control == 3)
				{
					Kd = Kd+100;
				//	EE_Write(id_Kd, Kd );
				}
				if(remote_control == 4)
				{
					Kl = Kl+50;
				//	EE_Write(id_Kl, Kl );
				}
				if(remote_control == 5)
				{
					force_prescaler = force_prescaler+10;
				//	EE_Write(id_force_prescaler, force_prescaler );
				}
				if(remote_control == 6)
				{
					turbine_power = turbine_power+100;
				//	EE_Write(id_turbine_power, turbine_power );
				}

			break;

			case 18: //strzalka w prawo

				if(remote_control == 1)
				{
					Kp = Kp+20;
					//EE_Write(id_Kp, Kp );
				}
				if(remote_control == 2)
				{
					Ki = Ki+15;
					//EE_Write(id_Ki, Ki );
				}
				if(remote_control == 3)
				{
					Kd = Kd+200;
					//EE_Write(id_Kd, Kd );
				}
				if(remote_control == 4)
				{
					Kl = Kl+100;
					//EE_Write(id_Kl, Kl );
				}
				if(remote_control == 5)
				{
					force_prescaler = force_prescaler+15;
					//EE_Write(id_force_prescaler, force_prescaler );
				}
				if(remote_control == 6)
				{
					turbine_power = turbine_power+200;
					//EE_Write(id_turbine_power, turbine_power );
				}

			break;

			case 12: //CH

				if(remote_control == 1)
				{
					Kp = Kp-5;
					//EE_Write(id_Kp, Kp );
				}
				if(remote_control == 2)
				{
					Ki = Ki-5;
					//EE_Write(id_Ki, Ki );
				}
				if(remote_control == 3)
				{
					Kd = Kd-50;
					//EE_Write(id_Kd, Kd );
				}
				if(remote_control == 4)
				{
					Kl = Kl-20;
					//EE_Write(id_Kl, Kl );
				}
				if(remote_control == 5)
				{
					force_prescaler = force_prescaler-5;
					//EE_Write(id_force_prescaler, force_prescaler );
				}
				if(remote_control == 6)
				{
					turbine_power = turbine_power-50;
					//EE_Write(id_turbine_power, turbine_power );
				}

			break;


			case 13: //strzalka w dol
				if(remote_control == 1)
				{
					Kp = Kp-10;
				//	EE_Write(id_Kp, Kp );
				}
				if(remote_control == 2)
				{
					Ki = Ki-10;
				//	EE_Write(id_Ki, Ki );
				}
				if(remote_control == 3)
				{
					Kd = Kd-100;
				//	EE_Write(id_Kd, Kd );
				}
				if(remote_control == 4)
				{
					Kl = Kl-50;
				//	EE_Write(id_Kl, Kl );
				}
				if(remote_control == 5)
				{
					force_prescaler = force_prescaler-10;
				//	EE_Write(id_force_prescaler, force_prescaler );
				}
				if(remote_control == 6)
				{
					turbine_power = turbine_power-100;
				//	EE_Write(id_turbine_power, turbine_power );
				}

			break;

			case 14: //PT
				if(remote_control == 1)
				{
					Kp = Kp-20;
					//EE_Write(id_Kp, Kp );
				}
				if(remote_control == 2)
				{
					Ki = Ki-15;
					//EE_Write(id_Ki, Ki );
				}
				if(remote_control == 3)
				{
					Kd = Kd-200;
					//EE_Write(id_Kd, Kd );
				}
				if(remote_control == 4)
				{
					Kl = Kl-100;
					//EE_Write(id_Kl, Kl );
				}
				if(remote_control == 5)
				{
					force_prescaler = force_prescaler-15;
					//EE_Write(id_force_prescaler, force_prescaler );
				}
				if(remote_control == 6)
				{
					turbine_power = turbine_power-200;
					//EE_Write(id_turbine_power, turbine_power );
				}

			break;

		}
*/
	ir_valid = 0; //odebrano przeslana informacje
}





