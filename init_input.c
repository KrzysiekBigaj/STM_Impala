#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include <stdint.h>
#include <stdio.h>
#include "deklaracje.h"


void InitSensors (void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure; //deklaracja struktury


	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //wlaczenie zasilania portu
	  //inicjalizacja:
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //ustawia jako wejscie
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5; //ktore piny konfigurujemy
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //wlaczenie zasilania portu
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //ktore piny konfigurujemy
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //wlaczenie zasilania portu
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; //ktore piny konfigurujemy
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void InitLED(void)
{
	//Inicjalizacja DIOD
		  GPIO_InitTypeDef  GPIO_InitStructure; //deklaracja struktury
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //wlaczenie zasilania portu
		  //inicjalizacja:
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //ustawia jako wyjscie
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11; //ktore piny konfigurujemy
		  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
