#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f10x_usart.h"
#include "deklaracje.h"
#include "ee_ram.h"

#include "TouchPanel.h"
#include "GLCD.h"


/* -------------------------------
 * Wszystkie funkcje oprocz inicjujacych i PWM
 * -------------------------------
 */


void ReadSensors(void)
{

	int i =0;

	for(i=0; i<14; i++)
	{
		if (ADCBuffer[i] <= border)
		{
			sensor[i] = 0;
		}
		else
		{
			sensor[i] = 1;
		}
	}
	/*
    sensor[13] = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3 );
    sensor[12] = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2 );
	sensor[11] = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1 );
    sensor[10] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3 );
	sensor[9] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2 );
	sensor[8] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1 );
	sensor[7] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0 );
	sensor[6]= GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0 );
	sensor[5] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5 );
	sensor[4] = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4 );
	sensor[3] = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7 );
	sensor[2] = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6 );
	sensor[1] = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5 );
	sensor[0]= GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4 );
     */

}

void CountErr (void)
{
	err=0;
	number = 0;
	int i;

	for (i=0; i<14; i++)
		{
			err+=sensor[i]*weight[i];
			number+=sensor[i]; // sensor[] przyjmuje wartosci 0/1
		}

	if(number!=0)
		{
			if (lost==-1)
				{
					TIM1->CNT=0;
					cycle=0;
					prev_err=weight[0] * 1.2;
				}

			if (lost==1)
				{
					TIM1->CNT=0;
					cycle=0;
					prev_err=weight[13] * 1.2;
				}

			lost = 0;
			err/=number;
		}

	if(number==0) // wypadniecie poza linie
	{
		if(prev_err < weight[3])
			lost = -1;

		if(prev_err > weight[8])
			lost = 1;
	}
}

void CountAdjustment(void)
{
	/*if (err>0)
		{
			err_i = cycle*Tp+(TIM1->CNT)/1000;
		}
	else if (err<0)
		{
			err_i = - cycle*Tp+(TIM1->CNT)/1000;
		}
	else
		{
			err_i = 0;
		}*/


	adjustment = (Kp*err + Kd*err_d)/100;
}

void MotorsControl(void)
{
	if (lost == 0 )
		{



			if(adjustment > 0)
			{
						if(adjustment > 100)
							{
								MotorP(-1, adjustment * 0.8);
								MotorL(1, 100);
							}

						else	if(adjustment > 80)
										{
											MotorP(-1, adjustment * 0.7);
											MotorL(1, 100);
										}

						else	if(adjustment > 60)
										{
											MotorP(-1, adjustment * 0.45);
											MotorL(1, 100);
										}

						else	if(adjustment > 40)
										{
											MotorP(0, adjustment * 2.5);
											MotorL(1, 100);
										}

						else	if(adjustment > 20)
										{
											MotorP(1, 100 - adjustment - 65);
											MotorL(1, 100);
										}

						else
								{
									MotorP(1, 100 - adjustment);
									MotorL(1, 100);
								}


			}

			else	if(adjustment==0)
						{
							MotorP(1, 100);
							MotorL(1, 100);
						}

			else	if(adjustment < -100)
						{
							MotorP(1,  100);
							MotorL(-1, - adjustment * 0.8);
						}

			else	if(adjustment <- 80)
						{
							MotorP(1, 100);
							MotorL(-1, - adjustment * 0.7);
						}

			else	if(adjustment < -60)
						{
							MotorP(1, 100);
							MotorL(-1, - adjustment * 0.45);
						}

			else	if(adjustment < -40)
						{
							MotorP(1, 100);
							MotorL(0, - adjustment * 2.5);
						}

			else	if(adjustment < -20)
						{
							MotorP(1, 100);
							MotorL(1, 100 + adjustment - 65);
						}

			else
						{
							MotorP(1, 100);
							MotorL(1, 100 + adjustment);
						}
		}

	double time = cycle*Tp+(TIM1->CNT)/1000; // czas w ms


	if (lost == -1)
		{
				MotorL(-1, 100 - (time*Kl/100.0) );
				MotorP(1, 100 );
		}

	if (lost == 1)
		{
				MotorL(1,  100 );
				MotorP(-1, 100 - (time*Kl/100.0) );
		}

}




void Calibrate(void)
{
	int S = 0;
	int P = 0;
	int i =0;
	int j = 0;
	int number = 30;


	for (i=0; i<number; i++)
	{
		S = S + ADCBuffer[0];
		P = P + ADCBuffer[13];

		for(j=0; j < 1000; j++)
			continue;
	}

	S = S/number;
	P = P/number;

	border = ((S+P)/3);
}


int GetCharFromUSART ()
{
	if ((USART3->SR & USART_FLAG_RXNE) != 0)
		{
			 return USART3->DR;
		}
	else
		return 0;
}



char tbl[256];
volatile uint8_t ids = 0;
volatile uint8_t idr = 0;
volatile int send = 0;

void USART3_IRQHandler ()
{
	char c = USART3->DR;
	tbl[ids] = c;

	if(tbl[ids] == 'w' || tbl[ids] == 's')
		{
			ids++;
			tbl[ids] = '#';
		}

	if(tbl[ids] == '#')
		send = 1;

	ids++;

	USART3->SR &= ~USART_FLAG_RXNE;	          // clear interrupt
//  USART3->SR &= ~USART_FLAG_TXE;	          // clear interrupt
}

int Calculate()
{
	int value = 0;


	while(tbl[idr] != '#')
	{
		value = value*10 + (tbl[idr] - '0');
		idr += 1;
	}
	return value;
}

void SetWeights()
{
	int value = 0;
	int i;
	for (i=0; i<7; i++)
	{
		while (tbl[idr] != '_' && tbl[idr] != '#')
		{
			value = value*10 + (tbl[idr] - '0');
			idr += 1;
		}

		weight[i] = (-1)*value;
		weight[7+i] = value;
		EE_Write(id_weight+i, value );
		value = 0;
		if(tbl[idr] != '#')
		{
			idr += 1;
		}
	}

	return;
}


void ReadFromUSART()
{
	int temp = 0;

	if (send == 1)
	{
		char c = tbl[idr];
		idr += 1;
		switch (c)
		{
		case 'w':
			remote_control = 'w';
			break;
		case 's':
			remote_control = 's';
			break;

		case 'p':
			Kp = Calculate();
			//EE_Write(id_Kp, Kp );
			break;
		case 'i':
			Ki = Calculate();
			//EE_Write(id_Ki, Ki );
			break;
		case 'd':
			Kd = Calculate();
			//EE_Write(id_Kd, Kd );
			break;
		case 'l':
			Kl = Calculate();
			//EE_Write(id_Kl, Kl );
			break;

		case 'f':
			temp = Calculate();
			if(temp>=0 && temp <= 100)
				force_prescaler = temp;

			//EE_Write(id_force_prescaler, force_prescaler );
			break;
		case 't':
			temp = Calculate();
			if(temp>=1000 && temp<=2000)
				turbine_power = temp;

			if(remote_control != 's')
				TIM4->CCR4 = turbine_power;

			//EE_Write(id_turbine_power, turbine_power );
			break;

		case 'c':
			Calibrate();
			//remote_control = 'c';
			//EE_Write(id_border, border);
			break;
		case 'b':
			border = Calculate();
			//EE_Write(id_border, border);
			break;
		case 'v':
			SetWeights();
			break;
		default:
			while(tbl[idr] != '#')
			{
				idr += 1;
			}
			break;
		}

		send = 0;
		idr += 1;
		led_on = 1;

	}

}

void LedSendConfirm()
{
	if (led_on != 0)
	{
		GPIOC->BSRR = GPIO_Pin_12;
		led_on++;
	}
	if(led_on >= led_on_time)
	{
		GPIOC->BRR = GPIO_Pin_12;
		led_on = 0;
	}

}

//1V = 389 jednostek
#define cut_off 389*6.2

void CheckBattery(void)
{
	if (ADCBuffer[14]<cut_off)     // 3184 odpowiada pe³nej baterii (7,4V) , max wartosc buffera to 4095
		GPIOC->BSRR = GPIO_Pin_11;  //GPIO_SetBits  (  GPIOC, GPIO_Pin_11) ;
	else
		GPIOC->BRR = GPIO_Pin_11;  //GPIO_ResetBits  (  GPIOC, GPIO_Pin_11) ;
}
