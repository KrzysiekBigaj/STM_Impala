#include<stm32f10x_adc.h>
#include<stm32f10x_rcc.h>
#include<stm32f10x_gpio.h>
#include<stm32f10x_rcc.h>
#include<stm32f10x_dma.h>


/* -----------------------------------------------------------------------------------------
 * Ten blok kodu bedzie implementowany i testowany gdy bedziemy korzystac z przetwornikow AC
 * Na razie ten kod nie jest uzywany
 * -----------------------------------------------------------------------------------------
 */

//zmienic w inicjalizacji sensorow w init_input.c rodzaj wejsc na GPIO_Mode_AIN


//rozmiar tablicy: 14 sensorów + 1 na baterie = 15
volatile uint16_t ADCBuffer[] = {0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA}; // volatile informuje procesor ze dana zmienna moze byc modyfikowana przez peryferia

void InitADC(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;


    // Enable ADC1 and GPIOC clock
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_ADC1, ENABLE);
    // Enable DMA clock
	RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA1 , ENABLE ) ;

	//inicjalizacja DMA
	DMA_InitStructure.DMA_BufferSize = 15;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCBuffer;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd ( DMA1_Channel1 , ENABLE ) ;

	//inicjalizacja wejsc (dubluje sie z funkcja init_input, ktora trzeba skasowac)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	//konfiguracja ADC
	RCC_ADCCLKConfig ( RCC_PCLK2_Div6 ) ;  // 72MHz / 6 = 12MHz <- max predkosc przetwornikow ac
//	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1 ,	ENABLE ) ; //??dubluje sie RCC_APB2Periph_ADC1. Co robi RCC_APB2Periph_AFIO?

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel = 15; // 15;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 12, ADC_SampleTime_7Cycles5); //PA_1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 13, ADC_SampleTime_7Cycles5);//PA_2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 14, ADC_SampleTime_7Cycles5);//PA_3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_7Cycles5);//PA_4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_7Cycles5);//PA_5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_7Cycles5);  //PA_6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_7Cycles5);  //PA_7
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 7, ADC_SampleTime_7Cycles5);//PB_0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 15, ADC_SampleTime_7Cycles5); //PB_1 bateria
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 8, ADC_SampleTime_7Cycles5); //PC_0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 9, ADC_SampleTime_7Cycles5);//PC_1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 10, ADC_SampleTime_7Cycles5);//PC_2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 11, ADC_SampleTime_7Cycles5);//PC_3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5, ADC_SampleTime_7Cycles5); //PC_4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 6, ADC_SampleTime_7Cycles5); //PC_5

	ADC_Cmd ( ADC1 , ENABLE ) ;
	ADC_DMACmd ( ADC1 , ENABLE ) ;

	//kalibracja ADC
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_SoftwareStartConvCmd ( ADC1 , ENABLE ) ; // rozpoczecie wykonywania pomiarow. Gdyby ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; to po odpaleniu tej funkcji wykonany by zostal tylko 1 pomiar. Kolejny pomiar trzeba by bylo wywolac jeszcze raz ta funkcja
}
