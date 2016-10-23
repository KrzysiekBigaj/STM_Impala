#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include <stdint.h>
#include "misc.h"
#include <stdio.h>
#include "ee_ram.h"
//#include "TouchPanel.h"
//#include "GLCD.h"

// Moja biblioteka
#include "deklaracje.h"

/*
 * Co nalezy przeanalizowac:
 * -wspolczynniki w MotorsControl();
 * -brak prescalera przy jezdzie do tylu
 * -wspolczynnik Kl przy wypadaniu
 */

//---------------------------------------------------------------------------------
/*

irda opis guzikow
1 - Kp
2 - Ki
3 - Kd
4 - Kl
5 - force_prescaler
6 - turbine_power
7 - border
CH-  -100
CH   +100
|<<  -20
>>|  +20
-    -5
+    +5
CH+  stop
>||  start
EQ   kalibracja
0    zerowanie parametru
100+ 100+
200+ 200+



list of ir_data:

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

// Parametry ktore moze modyfikowac uzytkownik
int weight[14]= {-125,-95,-70,-46,-26,-14,-6, 6, 14, 26, 46, 70, 95, 125};

//wartosci Kp, Ki, Kd, div sa dzielone w programie przez 100
int Kp = 80;
int Ki = 0;
int Kd = 1000;
int Kl = -50; //wspolczynnik czasu przy wypadnieciu

double force_prescaler = 92;//tu podajemy wartosc od 0 do 100 , bo potem prescaler jest dzielony przez 100
							//tak jest przyjete zeby mozna bylo przeslac to przez bluetootha
							//
							//prescaler preskaluje jazde do przodu i do tylu.
							//Jeli chcesz zeby np. tylko preskalowa³ jazde do przodu
							//wprowadz zmiany w 2 miejscach oznaczonych '!!!' w pliku 'pwm_nasz_regulator'

int turbine_power = 2000; // od 1000 do 2000

int border = 600;   // na bialym czujnik wskazuje ok 100-150 , a na czarnym 1800-3000

int Tp = 10;  // okres petli liczacej czas [ms]

int speed_err = 20;

int led_on_time = 3000; // cykli petli glownej

//-------------------------------------------------------------
//  Indeksy pamieci:
	//int id_ir_data = 0;
	//int id_ir_addr = 8;
	int id_Kp = 1;
	int id_Ki = 2;
	int id_Kd = 3;
	int id_Kl = 4;
	int id_force_prescaler = 5;
	int id_turbine_power = 6;
	int id_border = 7;


	int id_weight = 12; //indeks poczatku zapisu wag
	// id_weight_1 : id_weight;
	// id_weight_2 : id_weight+1;
	// id_weight_3 : id_weight+2;
	// id_weight_4 : id_weight+3;
	// id_weight_5 : id_weight+4;
	// id_weight_6 : id_weight+5;
	// id_weight_7 : id_weight+6;

	int id_mem = 20; //poczatkowy indeks od ktorego zapisujemy stan czujnikow

//-------------------------------------------------------------



// Inicjalizaca pozostalych parametrow
#define PREV_ERR_START_VALUE 1  // param. pocz.

int prev_err = PREV_ERR_START_VALUE;
int err = 0;
double err_i = 0;
double err_d = 0;

int number = 0;
int adjustment = 0;
int remote_control = 0;
int lost = 0;
int prev_lost=0;
volatile int cycle = 0;
double prev_time=0;
int turbine_stop = 1000;
int sensor[14];

int mem = 255;
int memory[256]; // 0-id_mem - stale parametry; id_mem-255 - pomiary z trasy w kolejnosci: prev_time, err, prev_time, err, ...


//irda
volatile int iridx = 0;
const int MAX_IR = 34;
const int IRREADY = 0x100;
int ir_addr = 0;
int ir_data = 0;
int ir_valid = 0;
volatile int irbuf[34];

int led_on = 0;


//--------------------------------------------------------------------------------------------------------------
//#define MEMORY  // jesli to odkomentujemy to bedzie dzialac zapisywanie do pamieci flash
//#define BLUETOOTH
//--------------------------------------------------------------------------------------------------------------


int main(void)
{
	int i;

	for (i=0; i<14; i++)
		{
			sensor[i]=0;
		}

	for (i=0; i<256; i++)
		{
			tbl[i]=0;
		}
/*
	for (i=0; i<7; i++)
	{
		EE_Write(id_weight+i, weight[7+i] );
	}
*/
	InitLED();
	//InitSensors();  // zakomentowane poniewaz inicjujemy sesory w initADC();
#ifdef BLUETOOTH
	InitUSART();
#endif
	InitTimer1();

	InitMotors();

	MotorP(0 ,100);
	MotorL(0, 100);

	EE_Init(); //inicjalizacja pamieci FLASH (ta pamiêæ nie znika po restarcie procka)
	InitADC();


	for (i=0; i<256; i++)
	{
	memory[i] = EE_Read(i, -1); // -1 wartosc domyslna gdy nic nie ma zapisane pod danym indeksem w pamieci
	}

/*
 	//czyszczenie calej pamieci
	for (i=0; i<256; i++)
	{
		EE_Write(i, -1 );
	}
*/


	InitTimer8();
	init_EXTI ();

	InitTurbine();


	for (i=0; i<7200000*5; i++)
		{
			continue;
		}
	TIM4->CCR4 = turbine_power;

	for (i=0; i<6000000; i++)
		{
			continue;
		}

	TIM4->CCR4 = turbine_stop;

	for (i=0; i<6000000; i++)
		{
			continue;
		}

	while (1)
	{
		TIM4->CCR4 = turbine_stop;
		MotorP(0 ,100);
		MotorL(0, 100);

		while(remote_control != 'w')
			{
#ifdef BLUETOOTH
				//remote_control = GetCharFromUSART ();
				ReadFromUSART();
#endif
				IrReceive();
				if(ir_valid == 1)
				{
					IrControl();
				}

				LedSendConfirm();
				CheckBattery();
			}


		TIM4->CCR4 = turbine_power;

		for (i=0; i<1200000; i++)
			{
				continue;
			}

		prev_err = PREV_ERR_START_VALUE;
		mem = id_mem; //prev_time , err i adjustment zapisywne sa w pamieci od indeksu id_mem
		lost = 0;
		cycle = 0;
		TIM1->CNT = 0; // zerowanie licznika
		while(remote_control != 's')
			{
				ReadSensors();
				CountErr();
				if(prev_err != err)
				{
					prev_time=cycle*Tp+(TIM1->CNT)/1000;

					if(prev_time != 0)
						err_d = (err - prev_err) / prev_time;
					else
						err_d = 0;

					if ((err<= 14 && err >= 0) || (err <= 0 && err >= -14) )
					{
						err_d=err_d;
					}
				}

				CountAdjustment();

				if(prev_err != err)
				{
#ifdef MEMORY
					if(mem<253)
					{
						memory[mem] = Kp*err;
						mem++;
						memory[mem] = Kd*err_d;
						mem++;
						//memory[mem] = adjustment ;
						//mem++;

						memory[mem] = -1 ;
						mem++;
					}

#endif
					prev_err = err;
					TIM1->CNT=0;
					cycle=0;
				}

				MotorsControl();
#ifdef BLUETOOTH
				ReadFromUSART();
				//remote_control = GetCharFromUSART ();
#endif
				IrReceive();
				if(ir_valid == 1)
				{
					IrControl();
					TIM4->CCR4 = turbine_power; // mozemy zmienic predkosc turbiny podczas jazdy
				}

				//LedSendConfirm();
				//CheckBattery();
			}

#ifdef MEMORY
		TIM4->CCR4 = turbine_stop;
		MotorP(0 ,100);
		MotorL(0, 100);

		EE_Write(id_Kp, Kp );
		EE_Write(id_Ki, Ki );
		EE_Write(id_Kd, Kd );
		EE_Write(id_Kl, Kl );
		EE_Write(id_force_prescaler, force_prescaler );
		EE_Write(id_turbine_power, turbine_power );
		EE_Write(id_border, border);


		for (i=id_mem; i<256; i++)
			{
				EE_Write(i, memory[i]); // -1 wartosc domyslna gdy nic nie ma zapisane pod danym indeksem w pamieci
			}
#endif


	}



}
