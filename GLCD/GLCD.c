/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			GLCD.c
** Descriptions:		STM32 FSMC TFT²Ù×÷º¯Êý¿â
**						
**------------------------------------------------------------------------------------------------------
** Created by:			poweravr
** Created date:		2010-11-7
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_fsmc.h"
#include "GLCD.h" 
//#include "HzLib.h"
#include "AsciiLib.h"
#include <math.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static uint16_t DeviceCode = 0x8989; //Pitlab LCD
static uint16_t TimerPeriod = 0;
uint16_t Channel2Pulse = 100;

short font = 0; //gothic

void LCD_WriteReg(uint8_t LCD_Reg,uint16_t LCD_RegValue)
{
  /* Write 16-bit Index, then Write Reg */
  LCD_REG = LCD_Reg;
  LCD_RAM = LCD_RegValue;
}

void LCD_WriteRAM_Prepare(void)
{
  LCD_REG = R34;
}

void LCD_WriteRAM(uint16_t RGB_Code)
{
  /* Write 16-bit GRAM Reg */
  LCD_RAM = RGB_Code;
}

uint16_t LCD_ReadRAM(void)
{
  volatile uint16_t dummy;
  /* Write 16-bit Index (then Read Reg) */
  LCD_REG = R34; // Select GRAM Reg
  /* Read 16-bit Reg */
  dummy = LCD_RAM;
  return LCD_RAM;
}


/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD Control lines (FSMC Pins) in alternate function
                   Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable FSMC, GPIOD, GPIOE and AFIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
                        
  /* PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9),
     PE.13(D10), PE.14(D11), PE.15(D12) */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                 GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                 GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* PD.00(D2), PD.01(D3), PD.04(RD), PD.5(WR), PD.7(CS), PD.8(D13), PD.9(D14),
     PD.10(D15), PD.11(RS) PD.14(D0) PD.15(D1) */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | 
                                 GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
                                 GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : LCD_FSMCConfig
* Description    : Configures the Parallel interface (FSMC) for LCD(Parallel mode)
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_FSMCConfig(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMTimingInitStructure;
  /* FSMC¶ÁËÙ¶ÈÉèÖÃ */
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 8;  //was 30
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;	 //was 0  
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 8;	 //was 0
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	/* FSMC ·ÃÎÊÄ£Ê½ */

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable; // FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
  /* FSMCÐ´ËÙ¶ÈÉèÖÃ */

  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 6; //2  	  //5,0,5 works even for "dead" displays
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;	 //0  
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 6;	 //2  
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;	  

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  /* Enable FSMC Bank1_SRAM Bank */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}


//#define LCD_REG              (*((volatile unsigned short *) 0x64000000)) /* RS = 0 */
//#define LCD_RAM              (*((volatile unsigned short *) 0x64000008)) /* RS = 1 */

static void __LCD_FSMCConfig(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  Timing_read,Timing_write;


    // FSMC GPIO configure 
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF
                               | RCC_APB2Periph_GPIOG, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        /*
        FSMC_D0 ~ FSMC_D3
        PD14 FSMC_D0   PD15 FSMC_D1   PD0  FSMC_D2   PD1  FSMC_D3
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

        /*
        FSMC_D4 ~ FSMC_D12
        PE7 ~ PE15  FSMC_D4 ~ FSMC_D12
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10
                                      | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOE,&GPIO_InitStructure);

        /* FSMC_D13 ~ FSMC_D15   PD8 ~ PD10 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

        /*
        FSMC_A0 ~ FSMC_A5   FSMC_A6 ~ FSMC_A9
        PF0     ~ PF5       PF12    ~ PF15
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                                      | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOF,&GPIO_InitStructure);

        /* FSMC_A10 ~ FSMC_A15  PG0 ~ PG5 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
        GPIO_Init(GPIOG,&GPIO_InitStructure);

        /* FSMC_A16 ~ FSMC_A18  PD11 ~ PD13 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

        /* RD-PD4 WR-PD5 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

        /* NBL0-PE0 NBL1-PE1 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
        GPIO_Init(GPIOE,&GPIO_InitStructure);

        /* NE1/NCE2 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_Init(GPIOD,&GPIO_InitStructure);
        /* NE2 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOG,&GPIO_InitStructure);
        /* NE3 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOG,&GPIO_InitStructure);
        /* NE4 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_Init(GPIOG,&GPIO_InitStructure);
    }
    /* FSMC GPIO configure */

    /*-- FSMC Configuration -------------------------------------------------*/
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &Timing_read;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &Timing_write;
    FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure);

    Timing_read.FSMC_AddressSetupTime = 8;             /* ??????  */
    Timing_read.FSMC_AddressHoldTime  = 8;             /* ??????  */
    Timing_read.FSMC_DataSetupTime = 8;                /* ??????  */
    Timing_read.FSMC_AccessMode = FSMC_AccessMode_A;    /* FSMC ???? */

    Timing_write.FSMC_AddressSetupTime = 8;             /* ??????  */
    Timing_write.FSMC_AddressHoldTime  = 8;             /* ??????  */
    Timing_write.FSMC_DataSetupTime = 8;                /* ??????  */
    Timing_write.FSMC_AccessMode = FSMC_AccessMode_A;   /* FSMC ???? */

    /* Color LCD configuration ------------------------------------
       LCD configured as follow:
          - Data/Address MUX = Disable
          - Memory Type = SRAM
          - Data Width = 16bit
          - Write Operation = Enable
          - Extended Mode = Enable
          - Asynchronous Wait = Disable */
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
}

/*******************************************************************************
* Function Name  : LCD_Configuration
* Description    : Configure the LCD Control pins and FSMC Parallel interface
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void LCD_Configuration(void)
{
 /* Configure the LCD Control pins --------------------------------------------*/
  	LCD_CtrlLinesConfig();

/* Configure the FSMC Parallel interface -------------------------------------*/
  	LCD_FSMCConfig();
}


/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Reads the selected LCD Register.
* Input          : None
* Output         : None
* Return         : LCD Register Value.
* Attention		 : None
*******************************************************************************/
__inline uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD_REG = LCD_Reg;
  /* Read 16-bit Reg */
  return (LCD_RAM);
}



/*******************************************************************************
* Function Name  : LCD_SetCursor
* Description    : Sets the cursor position.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position. 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetCursor(uint16_t Xpos,uint16_t Ypos)
{
  if(DeviceCode==0x8989)
  {
    LCD_WriteReg(0x004e,Xpos); /* Row */
    LCD_WriteReg(0x004f,Ypos); /* Line */ 
  }
  else if(DeviceCode==0x9919)
  {
    LCD_WriteReg(0x004e,Xpos); /* Row */
    LCD_WriteReg(0x004f,Ypos); /* Line */	
  }
  else
  {
    LCD_WriteReg(0x0020,Xpos); /* Row */
    LCD_WriteReg(0x0021,Ypos); /* Line */
  }
}

//uint16_t cXpos, uint16_t cYpos;
int LCD_GetCursor()
{
  if(DeviceCode==0x8989)
  {
    int x = LCD_ReadReg(0x004e); /* Row */
    int y = LCD_ReadReg(0x004f); /* Line */ 
	return (x<<16) + y;
  }
  else if(DeviceCode==0x9919)
  {
    int x = LCD_ReadReg(0x004e); /* Row */
    int y = LCD_ReadReg(0x004f); /* Line */ 
	return (x<<16) + y;
  }
  else
  {
    int x = LCD_ReadReg(0x0020); /* Row */
    int y = LCD_ReadReg(0x0021); /* Line */ 
	return (x<<16) + y;
  }
}

/*******************************************************************************
* Function Name  : LCD_Delay
* Description    : Delay Time
* Input          : - nCount: Delay Time
* Output         : None
* Return         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

//void delay_ms(uint16_t nms);

void LCD_Delay(uint16_t nCount)
{
//	delay_ms(nCount*10);

 	uint16_t TimingDelay; 
 	while(nCount--)
   	{
    	for(TimingDelay=0;TimingDelay<10000;TimingDelay++);
   	}

}

/*******************************************************************************
* Function Name  : LCD_Initializtion
* Description    : Initialize TFT Controller.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Initializtion()
{
  LCD_Configuration();
  LCD_Delay(5);  /* delay 50 ms */		
  DeviceCode = LCD_ReadReg(0x0000);		/* ¶ÁÈ¡ÆÁID	*/
  //DeviceCode = 0x8989;	//forces some "dead" displays to work 

  if(DeviceCode==0x9325 || DeviceCode==0x9328)	/* ²»Í¬ÆÁÇý¶¯IC ³õÊ¼»¯²»Í¬ */
  {
    LCD_WriteReg(0x00e7,0x0010);      
    LCD_WriteReg(0x0000,0x0001);  	/* start internal osc */
    LCD_WriteReg(0x0001,0x0100);     
    LCD_WriteReg(0x0002,0x0700); 	/* power on sequence */
	LCD_WriteReg(0x0003,(1<<12)|(1<<5)|(1<<4)|(0<<3) ); 	/* importance */
    LCD_WriteReg(0x0004,0x0000);                                   
    LCD_WriteReg(0x0008,0x0207);	           
    LCD_WriteReg(0x0009,0x0000);         
    LCD_WriteReg(0x000a,0x0000); 	/* display setting */        
    LCD_WriteReg(0x000c,0x0001);	/* display setting */        
    LCD_WriteReg(0x000d,0x0000); 			        
    LCD_WriteReg(0x000f,0x0000);
    /* Power On sequence */
    LCD_WriteReg(0x0010,0x0000);   
    LCD_WriteReg(0x0011,0x0007);
    LCD_WriteReg(0x0012,0x0000);                                                                 
    LCD_WriteReg(0x0013,0x0000);                 
    LCD_Delay(5);  /* delay 50 ms */		
    LCD_WriteReg(0x0010,0x1590);   
    LCD_WriteReg(0x0011,0x0227);
    LCD_Delay(5);  /* delay 50 ms */		
    LCD_WriteReg(0x0012,0x009c);                  
    LCD_Delay(5);  /* delay 50 ms */		
    LCD_WriteReg(0x0013,0x1900);   
    LCD_WriteReg(0x0029,0x0023);
    LCD_WriteReg(0x002b,0x000e);
    LCD_Delay(5);  /* delay 50 ms */		
    LCD_WriteReg(0x0020,0x0000);                                                            
    LCD_WriteReg(0x0021,0x0000);           
    LCD_Delay(5);  /* delay 50 ms */		
    LCD_WriteReg(0x0030,0x0007); 
    LCD_WriteReg(0x0031,0x0707);   
    LCD_WriteReg(0x0032,0x0006);
    LCD_WriteReg(0x0035,0x0704);
    LCD_WriteReg(0x0036,0x1f04); 
    LCD_WriteReg(0x0037,0x0004);
    LCD_WriteReg(0x0038,0x0000);        
    LCD_WriteReg(0x0039,0x0706);     
    LCD_WriteReg(0x003c,0x0701);
    LCD_WriteReg(0x003d,0x000f);
    LCD_Delay(5);  /* delay 50 ms */		
    LCD_WriteReg(0x0050,0x0000);        
    LCD_WriteReg(0x0051,0x00ef);   
    LCD_WriteReg(0x0052,0x0000);     
    LCD_WriteReg(0x0053,0x013f);
    LCD_WriteReg(0x0060,0xa700);        
    LCD_WriteReg(0x0061,0x0001); 
    LCD_WriteReg(0x006a,0x0000);
    LCD_WriteReg(0x0080,0x0000);
    LCD_WriteReg(0x0081,0x0000);
    LCD_WriteReg(0x0082,0x0000);
    LCD_WriteReg(0x0083,0x0000);
    LCD_WriteReg(0x0084,0x0000);
    LCD_WriteReg(0x0085,0x0000);
      
    LCD_WriteReg(0x0090,0x0010);     
    LCD_WriteReg(0x0092,0x0000);  
    LCD_WriteReg(0x0093,0x0003);
    LCD_WriteReg(0x0095,0x0110);
    LCD_WriteReg(0x0097,0x0000);        
    LCD_WriteReg(0x0098,0x0000);  
    /* display on sequence */    
    LCD_WriteReg(0x0007,0x0133);
    
    LCD_WriteReg(0x0020,0x0000);  /* ÐÐÊ×Ö·0 */                                                          
    LCD_WriteReg(0x0021,0x0000);  /* ÁÐÊ×Ö·0 */     
  }
  else if(DeviceCode==0x9320 || DeviceCode==0x9300)
  {
    LCD_WriteReg(0x00,0x0000);
	LCD_WriteReg(0x01,0x0100);	/* Driver Output Contral */
	LCD_WriteReg(0x02,0x0700);	/* LCD Driver Waveform Contral */
	LCD_WriteReg(0x03,0x1018);	/* Entry Mode Set */
	
	LCD_WriteReg(0x04,0x0000);	/* Scalling Contral */
    LCD_WriteReg(0x08,0x0202);	/* Display Contral */
	LCD_WriteReg(0x09,0x0000);	/* Display Contral 3.(0x0000) */
	LCD_WriteReg(0x0a,0x0000);	/* Frame Cycle Contal.(0x0000) */
    LCD_WriteReg(0x0c,(1<<0));	/* Extern Display Interface Contral */
	LCD_WriteReg(0x0d,0x0000);	/* Frame Maker Position */
	LCD_WriteReg(0x0f,0x0000);	/* Extern Display Interface Contral 2. */
	
    LCD_Delay(10);  /* delay 100 ms */		
	LCD_WriteReg(0x07,0x0101);	/* Display Contral */
    LCD_Delay(10);  /* delay 100 ms */		

	LCD_WriteReg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	/* Power Control 1.(0x16b0)	*/
	LCD_WriteReg(0x11,0x0007);								/* Power Control 2 */
	LCD_WriteReg(0x12,(1<<8)|(1<<4)|(0<<0));				/* Power Control 3.(0x0138)	*/
	LCD_WriteReg(0x13,0x0b00);								/* Power Control 4 */
	LCD_WriteReg(0x29,0x0000);								/* Power Control 7 */
	
	LCD_WriteReg(0x2b,(1<<14)|(1<<4));
		
	LCD_WriteReg(0x50,0);       /* Set X Start */
	LCD_WriteReg(0x51,239);	    /* Set X End */
	LCD_WriteReg(0x52,0);	    /* Set Y Start */
	LCD_WriteReg(0x53,319);	    /* Set Y End */
	
	LCD_WriteReg(0x60,0x2700);	/* Driver Output Control */
	LCD_WriteReg(0x61,0x0001);	/* Driver Output Control */
	LCD_WriteReg(0x6a,0x0000);	/* Vertical Srcoll Control */
	
	LCD_WriteReg(0x80,0x0000);	/* Display Position? Partial Display 1 */
	LCD_WriteReg(0x81,0x0000);	/* RAM Address Start? Partial Display 1 */
	LCD_WriteReg(0x82,0x0000);	/* RAM Address End-Partial Display 1 */
	LCD_WriteReg(0x83,0x0000);	/* Displsy Position? Partial Display 2 */
	LCD_WriteReg(0x84,0x0000);	/* RAM Address Start? Partial Display 2 */
	LCD_WriteReg(0x85,0x0000);	/* RAM Address End? Partial Display 2 */
	
    LCD_WriteReg(0x90,(0<<7)|(16<<0));	/* Frame Cycle Contral.(0x0013)	*/
	LCD_WriteReg(0x92,0x0000);	/* Panel Interface Contral 2.(0x0000) */
	LCD_WriteReg(0x93,0x0001);	/* Panel Interface Contral 3. */
    LCD_WriteReg(0x95,0x0110);	/* Frame Cycle Contral.(0x0110)	*/
	LCD_WriteReg(0x97,(0<<8));	
	LCD_WriteReg(0x98,0x0000);	/* Frame Cycle Contral */

    LCD_WriteReg(0x07,0x0173);
  }
  else if(DeviceCode==0x9331)
  {
	LCD_WriteReg(0x00E7, 0x1014);
	LCD_WriteReg(0x0001, 0x0100);   /* set SS and SM bit */
	LCD_WriteReg(0x0002, 0x0200);   /* set 1 line inversion */
	LCD_WriteReg(0x0003, 0x1030);   /* set GRAM write direction and BGR=1 */
	LCD_WriteReg(0x0008, 0x0202);   /* set the back porch and front porch */
    LCD_WriteReg(0x0009, 0x0000);   /* set non-display area refresh cycle ISC[3:0] */
	LCD_WriteReg(0x000A, 0x0000);   /* FMARK function */
	LCD_WriteReg(0x000C, 0x0000);   /* RGB interface setting */
	LCD_WriteReg(0x000D, 0x0000);   /* Frame marker Position */
	LCD_WriteReg(0x000F, 0x0000);   /* RGB interface polarity */
	/* Power On sequence */
	LCD_WriteReg(0x0010, 0x0000);   /* SAP, BT[3:0], AP, DSTB, SLP, STB	*/
	LCD_WriteReg(0x0011, 0x0007);   /* DC1[2:0], DC0[2:0], VC[2:0] */
	LCD_WriteReg(0x0012, 0x0000);   /* VREG1OUT voltage	*/
	LCD_WriteReg(0x0013, 0x0000);   /* VDV[4:0] for VCOM amplitude */
    LCD_Delay(20);                  /* delay 200 ms */		
	LCD_WriteReg(0x0010, 0x1690);   /* SAP, BT[3:0], AP, DSTB, SLP, STB	*/
	LCD_WriteReg(0x0011, 0x0227);   /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_Delay(5);                   /* delay 50 ms */		
	LCD_WriteReg(0x0012, 0x000C);   /* Internal reference voltage= Vci	*/
    LCD_Delay(5);                    /* delay 50 ms */		
	LCD_WriteReg(0x0013, 0x0800);   /* Set VDV[4:0] for VCOM amplitude */
	LCD_WriteReg(0x0029, 0x0011);   /* Set VCM[5:0] for VCOMH */
	LCD_WriteReg(0x002B, 0x000B);   /* Set Frame Rate */
    LCD_Delay(5);                   /* delay 50 ms */		
	LCD_WriteReg(0x0020, 0x0000);   /* GRAM horizontal Address */
	LCD_WriteReg(0x0021, 0x0000);   /* GRAM Vertical Address */
	/* Adjust the Gamma Curve */
	LCD_WriteReg(0x0030, 0x0000);
	LCD_WriteReg(0x0031, 0x0106);
	LCD_WriteReg(0x0032, 0x0000);
	LCD_WriteReg(0x0035, 0x0204);
	LCD_WriteReg(0x0036, 0x160A);
	LCD_WriteReg(0x0037, 0x0707);
	LCD_WriteReg(0x0038, 0x0106);
	LCD_WriteReg(0x0039, 0x0707);
	LCD_WriteReg(0x003C, 0x0402);
	LCD_WriteReg(0x003D, 0x0C0F);
	/* Set GRAM area */
	LCD_WriteReg(0x0050, 0x0000);   /* Horizontal GRAM Start Address */
	LCD_WriteReg(0x0051, 0x00EF);   /* Horizontal GRAM End Address */
	LCD_WriteReg(0x0052, 0x0000);   /* Vertical GRAM Start Address */
	LCD_WriteReg(0x0053, 0x013F);   /* Vertical GRAM Start Address */
	LCD_WriteReg(0x0060, 0x2700);   /* Gate Scan Line */
	LCD_WriteReg(0x0061, 0x0001);   /*  NDL,VLE, REV */
	LCD_WriteReg(0x006A, 0x0000);   /* set scrolling line */
	/* Partial Display Control */
	LCD_WriteReg(0x0080, 0x0000);
	LCD_WriteReg(0x0081, 0x0000);
	LCD_WriteReg(0x0082, 0x0000);
	LCD_WriteReg(0x0083, 0x0000);
	LCD_WriteReg(0x0084, 0x0000);
	LCD_WriteReg(0x0085, 0x0000);
	/* Panel Control */
	LCD_WriteReg(0x0090, 0x0010);
	LCD_WriteReg(0x0092, 0x0600);
	LCD_WriteReg(0x0007,0x0021);		
    LCD_Delay(5);                   /* delay 50 ms */		
	LCD_WriteReg(0x0007,0x0061);
    LCD_Delay(5);                   /* delay 50 ms */		
	LCD_WriteReg(0x0007,0x0133);    /* 262K color and display ON */
    LCD_Delay(5);                   /* delay 50 ms */		
  }
  else if(DeviceCode==0x9919)
  {
    /* POWER ON &RESET DISPLAY OFF */
	LCD_WriteReg(0x28,0x0006);
	LCD_WriteReg(0x00,0x0001);		
	LCD_WriteReg(0x10,0x0000);		
	LCD_WriteReg(0x01,0x72ef);
	LCD_WriteReg(0x02,0x0600);
	LCD_WriteReg(0x03,0x6a38);	
	LCD_WriteReg(0x11,0x6874);
	LCD_WriteReg(0x0f,0x0000);    /* RAM WRITE DATA MASK */
	LCD_WriteReg(0x0b,0x5308);    /* RAM WRITE DATA MASK */
	LCD_WriteReg(0x0c,0x0003);
	LCD_WriteReg(0x0d,0x000a);
	LCD_WriteReg(0x0e,0x2e00);  
	LCD_WriteReg(0x1e,0x00be);
	LCD_WriteReg(0x25,0x8000);
	LCD_WriteReg(0x26,0x7800);
	LCD_WriteReg(0x27,0x0078);
	LCD_WriteReg(0x4e,0x0000);
	LCD_WriteReg(0x4f,0x0000);
	LCD_WriteReg(0x12,0x08d9);
	/* Adjust the Gamma Curve */
	LCD_WriteReg(0x30,0x0000);
	LCD_WriteReg(0x31,0x0104);	 
	LCD_WriteReg(0x32,0x0100);	
    LCD_WriteReg(0x33,0x0305);	
    LCD_WriteReg(0x34,0x0505);	 
	LCD_WriteReg(0x35,0x0305);	
    LCD_WriteReg(0x36,0x0707);	
    LCD_WriteReg(0x37,0x0300);	
	LCD_WriteReg(0x3a,0x1200);	
	LCD_WriteReg(0x3b,0x0800);		 
    LCD_WriteReg(0x07,0x0033);
  }
  else if(DeviceCode==0x1505)
  {
    /* second release on 3/5  ,luminance is acceptable,water wave appear during camera preview */
    LCD_WriteReg(0x0007,0x0000);
    LCD_Delay(5);                   /* delay 50 ms */		
    LCD_WriteReg(0x0012,0x011C);    /* why need to set several times?	*/
    LCD_WriteReg(0x00A4,0x0001);    /* NVM */
    LCD_WriteReg(0x0008,0x000F);
    LCD_WriteReg(0x000A,0x0008);
    LCD_WriteReg(0x000D,0x0008);    
    /* GAMMA CONTROL */
    LCD_WriteReg(0x0030,0x0707);
    LCD_WriteReg(0x0031,0x0007); 
    LCD_WriteReg(0x0032,0x0603); 
    LCD_WriteReg(0x0033,0x0700); 
    LCD_WriteReg(0x0034,0x0202); 
    LCD_WriteReg(0x0035,0x0002); 
    LCD_WriteReg(0x0036,0x1F0F);
    LCD_WriteReg(0x0037,0x0707); 
    LCD_WriteReg(0x0038,0x0000); 
    LCD_WriteReg(0x0039,0x0000); 
    LCD_WriteReg(0x003A,0x0707); 
    LCD_WriteReg(0x003B,0x0000); 
    LCD_WriteReg(0x003C,0x0007); 
    LCD_WriteReg(0x003D,0x0000); 
    LCD_Delay(5);                   /* delay 50 ms */		
    LCD_WriteReg(0x0007,0x0001);
    LCD_WriteReg(0x0017,0x0001);    /* Power supply startup enable */
    LCD_Delay(5);                   /* delay 50 ms */		
    /* power control */
    LCD_WriteReg(0x0010,0x17A0); 
    LCD_WriteReg(0x0011,0x0217);    /* reference voltage VC[2:0]   Vciout = 1.00*Vcivl */
    LCD_WriteReg(0x0012,0x011E);    /* Vreg1out = Vcilvl*1.80   is it the same as Vgama1out ?	*/
    LCD_WriteReg(0x0013,0x0F00);    /* VDV[4:0]-->VCOM Amplitude VcomL = VcomH - Vcom Ampl */
    LCD_WriteReg(0x002A,0x0000);  
    LCD_WriteReg(0x0029,0x000A);    /* Vcomh = VCM1[4:0]*Vreg1out    gate source voltage?? */
    LCD_WriteReg(0x0012,0x013E);    /* power supply on */
    /* Coordinates Control */
    LCD_WriteReg(0x0050,0x0000);
    LCD_WriteReg(0x0051,0x00EF); 
    LCD_WriteReg(0x0052,0x0000); 
    LCD_WriteReg(0x0053,0x013F); 
    /* Pannel Image Control */
    LCD_WriteReg(0x0060,0x2700); 
    LCD_WriteReg(0x0061,0x0001); 
    LCD_WriteReg(0x006A,0x0000); 
    LCD_WriteReg(0x0080,0x0000); 
    /* Partial Image Control */
    LCD_WriteReg(0x0081,0x0000); 
    LCD_WriteReg(0x0082,0x0000); 
    LCD_WriteReg(0x0083,0x0000); 
    LCD_WriteReg(0x0084,0x0000); 
    LCD_WriteReg(0x0085,0x0000); 
    /* Panel Interface Control */
    LCD_WriteReg(0x0090,0x0013);      /* frenqucy */	
    LCD_WriteReg(0x0092,0x0300); 
    LCD_WriteReg(0x0093,0x0005); 
    LCD_WriteReg(0x0095,0x0000); 
    LCD_WriteReg(0x0097,0x0000); 
    LCD_WriteReg(0x0098,0x0000); 
  
    LCD_WriteReg(0x0001,0x0100); 
    LCD_WriteReg(0x0002,0x0700); 
    LCD_WriteReg(0x0003,0x1030); 
    LCD_WriteReg(0x0004,0x0000); 
    LCD_WriteReg(0x000C,0x0000); 
    LCD_WriteReg(0x000F,0x0000); 
    LCD_WriteReg(0x0020,0x0000); 
    LCD_WriteReg(0x0021,0x0000); 
    LCD_WriteReg(0x0007,0x0021); 
    LCD_Delay(20);                   /* delay 200 ms */		
    LCD_WriteReg(0x0007,0x0061); 
    LCD_Delay(20);                   /* delay 200 ms */		
    LCD_WriteReg(0x0007,0x0173); 
    LCD_Delay(20);                   /* delay 200 ms */		
  }							 
  else if(DeviceCode==0x8989) //mój HY32D !!!!!!!!!!!!
  {
  	//--------------------------------
	//  mój HY32D !!!!!!!!!!!!
	//--------------------------------
 /*
   // power supply setting
    // set R07h at 0021h (GON=1,DTE=0,D[1:0]=01)
    LCD_WriteReg(0x0007,0x0021);
    // set R00h at 0001h (OSCEN=1)
    LCD_WriteReg(0x0000,0x0001);
    // set R07h at 0023h (GON=1,DTE=0,D[1:0]=11)
    LCD_WriteReg(0x0007,0x0023);
    // set R10h at 0000h (Exit sleep mode)
    LCD_WriteReg(0x0010,0x0000);
    // Wait 30ms
    LCD_Delay(3);
    // set R07h at 0033h (GON=1,DTE=1,D[1:0]=11)
    LCD_WriteReg(0x0007,0x0033);
    // Entry mode setting (R11h)
    // R11H Entry mode
    // vsmode DFM1 DFM0 TRANS OEDef WMode DMode1 DMode0 TY1 TY0 ID1 ID0 AM LG2 LG2 LG0
    //   0     1    1     0     0     0     0      0     0   1   1   1  *   0   0   0
    LCD_WriteReg(0x0011,0x6070);
    // LCD driver AC setting (R02h)
    LCD_WriteReg(0x0002,0x0600);
    // power control 1
    // DCT3 DCT2 DCT1 DCT0 BT2 BT1 BT0 0 DC3 DC2 DC1 DC0 AP2 AP1 AP0 0
    // 1     0    1    0    1   0   0  0  1   0   1   0   0   1   0  0
    // DCT[3:0] fosc/4 BT[2:0]  DC{3:0] fosc/4
    LCD_WriteReg(0x0003,0x0804);//0xA8A4
    LCD_WriteReg(0x000C,0x0000);//
    LCD_WriteReg(0x000D,0x0808);// 0x080C --> 0x0808
    // power control 4
    // 0 0 VCOMG VDV4 VDV3 VDV2 VDV1 VDV0 0 0 0 0 0 0 0 0
    // 0 0   1    0    1    0    1    1   0 0 0 0 0 0 0 0
    LCD_WriteReg(0x000E,0x2900);
    LCD_WriteReg(0x001E,0x00B8);
 //   LCD_WriteReg(0x0001,0x2B3F);//??????320*240  0x6B3F
   //rotated 180 degree
	LCD_WriteReg(0x0001,(0x2B3F | 0x4000) & (~0x200));   //Driver Output Control (R01h)  320*240 0x2B3F 
    
	LCD_WriteReg(0x0010,0x0000);
    LCD_WriteReg(0x0005,0x0000);
    LCD_WriteReg(0x0006,0x0000);
    LCD_WriteReg(0x0016,0xEF1C);
    LCD_WriteReg(0x0017,0x0003);
    LCD_WriteReg(0x0007,0x0233);//0x0233
    LCD_WriteReg(0x000B,0x0000|(3<<6));
    LCD_WriteReg(0x000F,0x0000);//??????
    LCD_WriteReg(0x0041,0x0000);
    LCD_WriteReg(0x0042,0x0000);
    LCD_WriteReg(0x0048,0x0000);
    LCD_WriteReg(0x0049,0x013F);
    LCD_WriteReg(0x004A,0x0000);
    LCD_WriteReg(0x004B,0x0000);
    LCD_WriteReg(0x0044,0xEF00);
    LCD_WriteReg(0x0045,0x0000);
    LCD_WriteReg(0x0046,0x013F);
    LCD_WriteReg(0x0030,0x0707);
    LCD_WriteReg(0x0031,0x0204);
    LCD_WriteReg(0x0032,0x0204);
    LCD_WriteReg(0x0033,0x0502);
    LCD_WriteReg(0x0034,0x0507);
    LCD_WriteReg(0x0035,0x0204);
    LCD_WriteReg(0x0036,0x0204);
    LCD_WriteReg(0x0037,0x0502);
    LCD_WriteReg(0x003A,0x0302);
    LCD_WriteReg(0x003B,0x0302);
    LCD_WriteReg(0x0023,0x0000);
    LCD_WriteReg(0x0024,0x0000);
    LCD_WriteReg(0x0025,0x8000);   // 65hz
    LCD_WriteReg(0x004f,0);        // ???0
    LCD_WriteReg(0x004e,0);        // ???0
	return; 
*/
    
	LCD_WriteReg(0x0000,0x0001);    LCD_Delay(5);   /* ´ò¿ª¾§Õñ */
    LCD_WriteReg(0x0003,0xA8A4);    LCD_Delay(5);   
    LCD_WriteReg(0x000C,0x0000);    LCD_Delay(5);   
    LCD_WriteReg(0x000D,0x080C);    LCD_Delay(5);   
    LCD_WriteReg(0x000E,0x2B00);    LCD_Delay(5);   
    LCD_WriteReg(0x001E,0x00B0);    LCD_Delay(5);   
	//normal	
    //LCD_WriteReg(0x0001,0x2B3F);    LCD_Delay(5);   //Driver Output Control (R01h)  320*240 0x2B3F */
    //rotated 180 degree
	LCD_WriteReg(0x0001,(0x2B3F | 0x4000) & (~0x200));    LCD_Delay(5);   //Driver Output Control (R01h)  320*240 0x2B3F */
    
	LCD_WriteReg(0x0002,0x0600);    LCD_Delay(5);
    LCD_WriteReg(0x0010,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0011,0x6070);    LCD_Delay(5);   /* ¶¨ÒåÊý¾Ý¸ñÊ½ 16Î»É« ºáÆÁ 0x6070 */
    LCD_WriteReg(0x0005,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0006,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0016,0xEF1C);    LCD_Delay(5);
    LCD_WriteReg(0x0017,0x0003);    LCD_Delay(5);
    LCD_WriteReg(0x0007,0x0133);    LCD_Delay(5);         
    LCD_WriteReg(0x000B,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x000F,0x0000);    LCD_Delay(5);   /* É¨Ãè¿ªÊ¼µØÖ· */
    LCD_WriteReg(0x0041,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0042,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0048,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0049,0x013F);    LCD_Delay(5);
    LCD_WriteReg(0x004A,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x004B,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0044,0xEF00);    LCD_Delay(5);
    LCD_WriteReg(0x0045,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0046,0x013F);    LCD_Delay(5);
    LCD_WriteReg(0x0030,0x0707);    LCD_Delay(5);
    LCD_WriteReg(0x0031,0x0204);    LCD_Delay(5);
    LCD_WriteReg(0x0032,0x0204);    LCD_Delay(5);
    LCD_WriteReg(0x0033,0x0502);    LCD_Delay(5);
    LCD_WriteReg(0x0034,0x0507);    LCD_Delay(5);
    LCD_WriteReg(0x0035,0x0204);    LCD_Delay(5);
    LCD_WriteReg(0x0036,0x0204);    LCD_Delay(5);
    LCD_WriteReg(0x0037,0x0502);    LCD_Delay(5);
    LCD_WriteReg(0x003A,0x0302);    LCD_Delay(5);
    LCD_WriteReg(0x003B,0x0302);    LCD_Delay(5);
    LCD_WriteReg(0x0023,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0024,0x0000);    LCD_Delay(5);
    LCD_WriteReg(0x0025,0x8000);    LCD_Delay(5);
    LCD_WriteReg(0x004f,0);        /* ÐÐÊ×Ö·0 */
    LCD_WriteReg(0x004e,0);        /* ÁÐÊ×Ö·0 */
  }
  LCD_Delay(5);  /* delay 50 ms */		
 //	DeviceCode = LCD_ReadReg(0x0000);		/* ¶ÁÈ¡ÆÁID	*/
}

/******************************************************************************
* Function Name  : LCD_SetWindow
* Description    : Sets Windows Area.
* Input          : - StartX: Row Start Coordinate 
*                  - StartY: Line Start Coordinate  
*				   - xLong:  
*				   - yLong: 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

void LCD_SetWindow(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong)
{
  LCD_SetCursor(xStart,yStart); 
  LCD_WriteReg(0x0044, xStart + ((xStart + xLong-1) << 8) );         
  LCD_WriteReg(0x0045, yStart);         
  LCD_WriteReg(0x0046, yStart+yLong-1); 
}

/*******************************************************************************
* Function Name  : LCD_Clear
* Description    : ½«ÆÁÄ»Ìî³ä³ÉÖ¸¶¨µÄÑÕÉ«£¬ÈçÇåÆÁ£¬ÔòÌî³ä 0xffff
* Input          : - Color: Screen Color
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_Clear(uint16_t Color)
{
  uint32_t index=0;
  LCD_SetCursor(0,0); 
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index=0;index<76800;index++)
   {
     LCD_RAM=Color;
   }
}

void _LCD_ClearRect(uint16_t x, uint16_t y, uint16_t dx, uint16_t dy, uint16_t Color)
{
  	uint32_t i,j;

  	for(j=0; j < dy; j++)
  	{
  		LCD_SetCursor(x, y+j); 
  		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  		for(i=0; i < dx; i++)
	     	LCD_RAM=Color;
   }
}

void LCD_ClearRect(uint16_t x, uint16_t y, uint16_t dx, uint16_t dy, uint16_t Color)
{
	int cnt = dx*dy;
	LCD_SetWindow(x, y, dx, dy);	//set window wrap area
	LCD_WriteRAM_Prepare(); 		// Prepare to write GRAM 

  	while(cnt--)
     	LCD_RAM=Color; 				//write into window area, wrap to next line

	LCD_SetWindow(0, 0, 240, 320); //full screen
}

/******************************************************************************
* Function Name  : LCD_GetPoint
* Description    : »ñÈ¡Ö¸¶¨×ù±êµÄÑÕÉ«Öµ
* Input          : - Xpos: Row Coordinate
*                  - Xpos: Line Coordinate 
* Output         : None
* Return         : Screen Color
* Attention		 : None
*******************************************************************************/
uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos)
{
  LCD_SetCursor(Xpos,Ypos);
  if( DeviceCode==0x7783 || DeviceCode==0x4531 || DeviceCode==0x8989 )
    return ( LCD_ReadRAM() );
  else
    return ( LCD_BGR2RGB(LCD_ReadRAM()) );
}

/******************************************************************************
* Function Name  : LCD_SetPoint
* Description    : ÔÚÖ¸¶¨×ù±ê»­µã
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point)
{
  if ( ( Xpos > 239 ) ||( Ypos > 319 ) ) return;
  LCD_SetCursor(Xpos,Ypos);
  LCD_WriteRAM_Prepare();
  LCD_WriteRAM(point);
}

/******************************************************************************
* Function Name  : LCD_DrawPicture
* Description    : ÔÚÖ¸¶¨×ø±ê·¶Î§ÏÔÊ¾Ò»¸±Í¼Æ¬
* Input          : - StartX: Row Start Coordinate 
*                  - StartY: Line Start Coordinate  
*				   - EndX: Row End Coordinate 
*				   - EndY: Line End Coordinate   
* Output         : None
* Return         : None
* Attention		 : Í¼Æ¬È¡Ä£¸ñÊ½ÎªË®Æ½É¨Ãè£¬16Î»ÑÕÉ«Ä£Ê½
*******************************************************************************/
void LCD_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY,uint16_t *pic)
{
  uint16_t  i;
  //full screen picutre, or set window!!!!
  LCD_SetCursor(StartX,StartY);  
  LCD_WriteRAM_Prepare();
  for (i=0;i<(EndX*EndY);i++)
  {
      LCD_WriteRAM(*pic++);
  }
}


/******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : »­Ò»ÌõÖ±Ïß
* Input          : - x1: ÐÐ×ù±ê¿ªÊ¼
*                  - y1: ÁÐ×ù±ê¿ªÊ¼ 
*				   - x2: ÐÐ×ù±ê½áÊø
*				   - y2: ÁÐ×ù±ê½áÊø  
*				   - bkColor: ±³¾°ÑÕÉ« 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_DrawLine(int x1, int y1, int x2, int y2,uint16_t bkColor)  
{ 
  int x,y,dx,dy,Dx,Dy,e,i; 
  Dx=x2-x1; 
  Dy=y2-y1; 

  dx=fabs(x2-x1); 
  dy=fabs(y2-y1); 
  x=x1; 
  y=y1; 
  if(dy>dx) 
  { 
    e=-dy; 
    for(i=0;i<dy;i++) 
    { 
      LCD_SetPoint(x,y,bkColor); 
      if(Dy>=0) y++;   
      else y--;    
      e+=2*dx; 
      if(e>=0) 
      { 
        if(Dx>=0) x++; 
        else x--;  
        e-=2*dy; 
      } 
    } 
  } 
  else 
  { 
    e=-dx; 
    for(i=0;i<dx;i++) 
    { 
      LCD_SetPoint(x,y,bkColor); 
      if(Dx>=0) x++; 
      else x--; 
      e+=2*dy; 
      if(e>=0) 
      { 
        if(Dy>=0) y++; 
        else y--;
        e-=2*dx;
      } 
    } 
  } 
} 


#if ASCII_LIB > 0 
/******************************************************************************
* Function Name  : PutChar
* Description    : ½«LcdÆÁÉÏÈÎÒâÎ»ÖÃÏÔÊ¾Ò»¸ö×Ö·û
* Input          : - Xpos: Ë®Æ½×ø±ê 
*                  - Ypos: ´¹Ö±×ø±ê  
*				   - c: ÏÔÊ¾µÄ×Ö·û
*				   - charColor: ×Ö·ûÑÕÉ«   
*				   - bkColor: ±³¾°ÑÕÉ« 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
const unsigned char * GetASCIIptr(unsigned char ASCII);

void PutChar(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor,unsigned short bkColor)
{
  unsigned short i;
  unsigned short j;
  unsigned char tmp_char;
  const	unsigned char * buffer	= GetASCIIptr(c);

  for (i=0; i<16; i++)
  {
    tmp_char = *buffer++;
  	LCD_SetCursor(Xpos, Ypos+i);
	LCD_WriteRAM_Prepare();
    for (j=0; j<8; j++)
    {
      	if( (tmp_char & 0x80) != 0 )
			LCD_WriteRAM(charColor);  
        else
          	LCD_WriteRAM(bkColor);  
		tmp_char = tmp_char<<1; 
    }
  }
}


void PutChar2x(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor,unsigned short bkColor)
{
  unsigned short i;
  unsigned short j;
  unsigned char tmp_char;
  const	unsigned char * buffer	= GetASCIIptr(c);

  for (i=0; i<16; i++)
  {
    tmp_char = *buffer++;
	for (j=0; j<8; j++)
    {
	  	LCD_SetCursor(Xpos+j+j, Ypos+i+i);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
		{
			LCD_WriteRAM(charColor);  
			LCD_WriteRAM(charColor);  
		}
		else
		{
          	LCD_WriteRAM(bkColor);  
			LCD_WriteRAM(bkColor);  
		}
	  	LCD_SetCursor(Xpos+j+j, Ypos+i+i+1);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
		{
			LCD_WriteRAM(charColor);  
			LCD_WriteRAM(charColor);  
		}
		else
		{
          	LCD_WriteRAM(bkColor);  
			LCD_WriteRAM(bkColor);  
		}
		tmp_char = tmp_char<<1; 
    }
  }
}
void PutCharOpaque(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor)
{
  unsigned short i;
  unsigned short j;
  unsigned char tmp_char;
  const	unsigned char * buffer	= GetASCIIptr(c);

  for (i=0; i<16; i++)
  {
    tmp_char = *buffer++;
    for (j=0; j<8; j++)
    {
	  	LCD_SetCursor(Xpos+j, Ypos+i);
		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
			LCD_WriteRAM(charColor);
        //else  left background unchanged
		tmp_char = tmp_char<<1;
    }
  }
}




void PutChar6xV(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor,unsigned short bkColor)
{
  unsigned short i;
  unsigned short j;
  unsigned char tmp_char;
  const	unsigned char * buffer	= GetASCIIptr(c);

  for (i=0; i<16; i++)
  {
    tmp_char = *buffer++;
	for (j=0; j<8; j++)
    {
		LCD_SetCursor(Xpos+i*6, Ypos+j*6);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x01) != 0 )
		{
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
		}
		else
		{
          	LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
		}
      	LCD_SetCursor(Xpos+i*6, Ypos+j*6+1);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x01) != 0 )
		{
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
		}
		else
		{
          	LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
		}
      	LCD_SetCursor(Xpos+i*6, Ypos+j*6+2);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x01) != 0 )
		{
      		LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
		}
		else
		{
			LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
		}
      	LCD_SetCursor(Xpos+i*6, Ypos+j*6+3);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x01) != 0 )
		{
      		LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
      					LCD_WriteRAM(charColor);
		}
		else
		{
			LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
						LCD_WriteRAM(bkColor);
		}
      	LCD_SetCursor(Xpos+i*6, Ypos+j*6+4);
      	  		LCD_WriteRAM_Prepare();
      	      	if( (tmp_char & 0x01) != 0 )
      			{
      	      		LCD_WriteRAM(charColor);
      	      					LCD_WriteRAM(charColor);
      	      					LCD_WriteRAM(charColor);
      	      					LCD_WriteRAM(charColor);
      	      					LCD_WriteRAM(charColor);
      	      					LCD_WriteRAM(charColor);
      			}
      			else
      			{
      				LCD_WriteRAM(bkColor);
      							LCD_WriteRAM(bkColor);
      							LCD_WriteRAM(bkColor);
      							LCD_WriteRAM(bkColor);
      							LCD_WriteRAM(bkColor);
      							LCD_WriteRAM(bkColor);
      			}
      	      LCD_SetCursor(Xpos+i*6, Ypos+j*6+5);
      	        		LCD_WriteRAM_Prepare();
      	            	if( (tmp_char & 0x01) != 0 )
      	      		{
      	            		LCD_WriteRAM(charColor);
      	            					LCD_WriteRAM(charColor);
      	            					LCD_WriteRAM(charColor);
      	            					LCD_WriteRAM(charColor);
      	            					LCD_WriteRAM(charColor);
      	            					LCD_WriteRAM(charColor);
      	      		}
      	      		else
      	      		{
      	      			LCD_WriteRAM(bkColor);
      	      						LCD_WriteRAM(bkColor);
      	      						LCD_WriteRAM(bkColor);
      	      						LCD_WriteRAM(bkColor);
      	      						LCD_WriteRAM(bkColor);
      	      						LCD_WriteRAM(bkColor);
      	      		}


		tmp_char = tmp_char>>1;
    }
  }
}
void PutCharV2(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor,unsigned short bkColor)
{
  unsigned short i;
  unsigned short j;
  unsigned char tmp_char;
  const	unsigned char * buffer	= GetASCIIptr(c);

  for (i=0; i<16; i++)
  {
    tmp_char = *buffer++;
	for (j=0; j<8; j++)
    {
	  	LCD_SetCursor(Xpos+i+i, Ypos+j+j);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
		{
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
		}
		else
		{
          	LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
		}
	  	LCD_SetCursor(Xpos+i+i, Ypos+j+j+1);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
		{
			LCD_WriteRAM(charColor);
			LCD_WriteRAM(charColor);
		}
		else
		{
          	LCD_WriteRAM(bkColor);
			LCD_WriteRAM(bkColor);
		}
		tmp_char = tmp_char<<1;
    }
  }
}

void PutChar2H(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor,unsigned short bkColor)
{
  unsigned short i;
  unsigned short j;
  unsigned char tmp_char;
  const	unsigned char * buffer	= GetASCIIptr(c);

  for (i=0; i<16; i++)
  {
    tmp_char = *buffer++;
	for (j=0; j<8; j++)
    {
	  	LCD_SetCursor(Xpos+j, Ypos+i+i);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
			LCD_WriteRAM(charColor);  
		else
			LCD_WriteRAM(bkColor);  
	  	LCD_SetCursor(Xpos+j, Ypos+i+i+1);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
			LCD_WriteRAM(charColor);  
		else
			LCD_WriteRAM(bkColor);  
		tmp_char = tmp_char<<1; 
    }
  }
}
void PutChar2W(unsigned short Xpos,unsigned short Ypos,unsigned char c,unsigned short charColor,unsigned short bkColor)
{
  unsigned short i;
  unsigned short j;
  unsigned char tmp_char;
  const	unsigned char * buffer	= GetASCIIptr(c);

  for (i=0; i<16; i++)
  {
    tmp_char = *buffer++;
	for (j=0; j<8; j++)
    {
	  	LCD_SetCursor(Xpos+j+j, Ypos+i);
  		LCD_WriteRAM_Prepare();
      	if( (tmp_char & 0x80) != 0 )
		{
			LCD_WriteRAM(charColor);  
			LCD_WriteRAM(charColor);  
		}
		else
		{
          	LCD_WriteRAM(bkColor);  
			LCD_WriteRAM(bkColor);  
		}
		tmp_char = tmp_char<<1; 
    }
  }
}

/******************************************************************************
* Function Name  : GUI_Text
* Description    : ÔÚÖ¸¶¨×ù±êÏÔÊ¾×Ö·û´®
* Input          : - Xpos: ÐÐ×ù±ê
*                  - Ypos: ÁÐ×ù±ê 
*				   - str: ×Ö·û´®
*				   - charColor: ×Ö·ûÑÕÉ«   
*				   - bkColor: ±³¾°ÑÕÉ« 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GUI_Text(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color, uint16_t bkColor)
{
 	while (*str!=0)
  	{
	    PutChar(Xpos,Ypos,(uint8_t)*str++,Color,bkColor);    
	    if (Xpos<224)	//240px ekran, zajêlismy w³asnie 8 i musimy miec miejsce na kolejne 8 pixeli
	    {
	      Xpos+=8;
	    } 
	    else if (Ypos<304) //powinno byæ 320-16-16  bo 16 zajêlismy i 16 potrzebujemy na nastepny wiersz
	    {
	      Xpos=0;
	      Ypos+=16;
	    }   
	    else
	    {
	      Xpos=0;
	      Ypos=0;
	    }    
  	}
}
void GUI_TextOpaque(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color)
{
 	while (*str!=0)
  	{
	    PutCharOpaque(Xpos,Ypos,(uint8_t)*str++,Color);    
	    if (Xpos<224)	//240px ekran, zajêlismy w³asnie 8 i musimy miec miejsce na kolejne 8 pixeli
	    {
	      Xpos+=8;
	    } 
	    else if (Ypos<304)
	    {
	      Xpos=0;
	      Ypos+=16;
	    }   
	    else
	    {
	      Xpos=0;
	      Ypos=0;
	    }    
  	}
}

void GUI_Text2x(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color, uint16_t bkColor)
{
 	do
  	{
	    PutChar2x(Xpos,Ypos,(uint8_t)*str++,Color,bkColor);    
	    if (Xpos<224)
	    {
	      Xpos+=16;
	    } 
	    else if (Ypos<288)
	    {
	      Xpos=0;
	      Ypos+=32;
	    }   
	    else
	    {
	      Xpos=0;
	      Ypos=0;
	    }    
  	}
  	while (*str!=0);
}
void GUI_Text6Vx(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color, uint16_t bkColor)
{
 	do
  	{
	    PutChar6xV(Xpos,Ypos,(uint8_t)*str++,Color,bkColor);
	    if (Xpos<224)
	    {
	      Xpos+=16;
	    }
	    else if (Ypos<288)
	    {
	      Xpos=0;
	      Ypos+=32;
	    }
	    else
	    {
	      Xpos=0;
	      Ypos=0;
	    }
  	}
  	while (*str!=0);
}
void GUI_Text2H(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color, uint16_t bkColor)
{
 	do
  	{
	    PutChar2H(Xpos,Ypos,(uint8_t)*str++,Color,bkColor);    
	    if (Xpos<232)
	    {
	      Xpos+=8;
	    } 
	    else if (Ypos<288)
	    {
	      Xpos=0;
	      Ypos+=32;
	    }   
	    else
	    {
	      Xpos=0;
	      Ypos=0;
	    }    
  	}
  	while (*str!=0);
}

void GUI_Text2W(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color, uint16_t bkColor)
{
 	do
  	{
	    PutChar2W(Xpos,Ypos,(uint8_t)*str++,Color,bkColor);    
	    if (Xpos<224)
	    {
	      Xpos+=16;
	    } 
	    else if (Ypos<304)
	    {
	      Xpos=0;
	      Ypos+=16;
	    }   
	    else
	    {
	      Xpos=0;
	      Ypos=0;
	    }    
  	}
  	while (*str!=0);
}


#endif

#if HZ_LIB > 0 
/******************************************************************************
* Function Name  : PutChinese
* Description    : ½«LcdÆÁÉÏÈÎÒâÎ»ÖÃÏÔÊ¾Ò»¸öÖÐÎÄ×Ö
* Input          : - Xpos: Ë®Æ½×ø±ê 
*                  - Ypos: ´¹Ö±×ø±ê  
*				   - str: ÏÔÊ¾µÄÖÐÎÄ×Ö
*				   - Color: ×Ö·ûÑÕÉ«   
*				   - bkColor: ±³¾°ÑÕÉ« 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void PutChinese(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor)
{
  uint8_t i,j;
  uint8_t buffer[32];
  uint16_t tmp_char=0;
 
  GetGBKCode(buffer,str);  /* È¡×ÖÄ£Êý¾Ý */

  for (i=0;i<16;i++)
  {
    tmp_char=buffer[i*2];
	tmp_char=(tmp_char<<8);
	tmp_char|=buffer[2*i+1];
    for (j=0;j<16;j++)
    {
      if ( (tmp_char >> 15-j) & 0x01 == 0x01)
        {
          LCD_SetPoint(Xpos+j,Ypos+i,Color);  /* ×Ö·ûÑÕÉ« */
        }
        else
        {
          LCD_SetPoint(Xpos+j,Ypos+i,bkColor);  /* ±³¾°ÑÕÉ« */
        }
    }
  }
}

/******************************************************************************
* Function Name  : GUI_Chinese
* Description    : ÔÚÖ¸¶¨×ù±êÏÔÊ¾×Ö·û´®
* Input          : - Xpos: ÐÐ×ù±ê
*                  - Ypos: ÁÐ×ù±ê 
*				   - str: ×Ö·û´®
*				   - charColor: ×Ö·ûÑÕÉ«   
*				   - bkColor: ±³¾°ÑÕÉ« 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

void GUI_Chinese(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
  do
  {
    PutChinese(Xpos,Ypos,str++,Color,bkColor);
	str++;
   if (Xpos<224)
    {
      Xpos+=16;
    }
    else if (Ypos<304)
    {
      Xpos=0;
      Ypos+=16;
    }
    else
    {
      Xpos=0;
      Ypos=0;
    }       
  }
  while(*str!=0);
}  
*/
#endif 

/******************************************************************************
* Function Name  : LCD_BGR2RGB
* Description    : RRRRRGGGGGGBBBBB ¸ÄÎª BBBBBGGGGGGRRRRR ¸ñÊ½
* Input          : - color: BRG ÑÕÉ«Öµ  
* Output         : None
* Return         : RGB ÑÕÉ«Öµ
* Attention		 : ÄÚ²¿º¯Êýµ÷ÓÃ
*******************************************************************************/
uint16_t LCD_BGR2RGB(uint16_t color)
{
  uint16_t  r, g, b, rgb;

  b = ( color>>0 )  & 0x1f;
  g = ( color>>5 )  & 0x3f;
  r = ( color>>11 ) & 0x1f;
 
  rgb =  (b<<11) + (g<<5) + (r<<0);

  return( rgb );
}

/******************************************************************************
* Function Name  : LCD_BackLight_Init
* Description    : LCD_BackLight Initializtion 
* Input          : None  
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_BackLight_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 						 
  /*GPIOB Configuration:  PB5(TIM3 CH2) as alternate function push-pull */

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);	      

  /* -----------------------------------------------------------------------
    TIM3CLK = 36 MHz, Prescaler = 35, TIM3 counter clock = 1 MHz
    TIM3 ARR Register = 999 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
    TIM3 Frequency = 1 KHz.
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 
  ----------------------------------------------------------------------- */
   /* ÆµÂÊÎª 1MHz TIM3 counter clock = 1MHz */
  TimerPeriod = (uint16_t) (SystemCoreClock / 1000000) - 1;

  /* Êä³öÆµÂÊ=Ê±ÖÓ/(ARR+1)£¬ËùÒÔ½«Êä³öÒ»¸ö 1Mhz/(999 + 1 )=1kHz ÆµÂÊ	 */
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;  /* Ê¹ÓÃÏµÍ³»ù´¡Ê±ÖÓ */
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseStructure.TIM_Prescaler = TimerPeriod;  //1MHz clock
  TIM_TimeBaseStructure.TIM_Period = 999;   /* TIM3 ARR Register */ 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* TIM_PulseÓÃÀ´¿ØÖÆÕ¼¿Õ±È£¬ËûÊµ¼ÊÓ°ÏìTIMµÄCCR2¼Ä´æÆ÷£¬³ÌÐòÖÐ¿ÉËæÊ±¸ü¸Ä¸Ã¼Ä´æÆ÷µÄÖµ£¬¿ÉËæÊ±¸ü¸ÄÕ¼¿Õ±È¡£Õ¼¿Õ±È=(CCRx/ARR)*100¡£*/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; /* Êä³öÄ£Ê½ */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse*10; /* Õ¼¿Õ±È²ÎÊý */
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);    /* Ê¹ÄÜTIM3ÔÚCCR2µÄÔ¤×°´¢´æÆ÷ */

  TIM_ARRPreloadConfig(TIM3, ENABLE);  /* Ê¹ÄÜ ARR×°ÔØ */

	TIM_ClearFlag(TIM3, TIM_FLAG_Update);							    
	TIM_ITConfig(TIM3,TIM_IT_Update, DISABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE); //output

  TIM_Cmd(TIM3, ENABLE);			   /* Ê¹ÄÜTIM3 */

}
/******************************************************************************
* Function Name  : LCD_BackLight
* Description    : µ÷ÕûÒº¾§±³¹â
* Input          : - percent: ±³¹âÁÁ¶È°Ù·Ö±È 
* Output         : None
* Return         : ·µ»Ø1³É¹¦ ·µ»Ø0Ê§°Ü
* Attention		 : None
*******************************************************************************/
FunctionalState LCD_BackLight( uint8_t percent)
{

  if( percent <= 100)
  {
    Channel2Pulse=percent;
	LCD_BackLight_Init(); 
	return ENABLE;
  } 
  else
    return DISABLE;
}

//Zbig.
void LCD_DrawHLine(uint16_t StartX,uint16_t StartY,uint16_t dX,uint16_t color)
{
  	LCD_SetCursor(StartX, StartY);  
	LCD_WriteRAM_Prepare();
	while(dX-- > 0) 	//poziom
        LCD_WriteRAM(color);
}

void LCD_DrawVLine(uint16_t StartX,uint16_t StartY, uint16_t dY,uint16_t color)
{
  	uint16_t  i;
	for(i=0; i < dY; i++) 	 //pion
	{
	  	LCD_SetCursor(StartX, StartY + i);  
		LCD_WriteRAM_Prepare();
        LCD_WriteRAM(color);
	}
}

//Zbig.
void LCD_DrawRect(uint16_t StartX,uint16_t StartY,uint16_t dX,uint16_t dY,uint16_t color)
{
//zweryfikowane doci¹ganie pixeli
  	uint16_t  i;
	for(i=1; i < dY; i++) 	 //pion
	{
	  	LCD_SetCursor(StartX, StartY + i);  
		LCD_WriteRAM_Prepare();
        LCD_WriteRAM(color);
	  	LCD_SetCursor(StartX+dX-1, StartY + i);  
		LCD_WriteRAM_Prepare();
        LCD_WriteRAM(color);
	}
  	LCD_SetCursor(StartX, StartY);  
	LCD_WriteRAM_Prepare();
	for(i=0; i < dX; i++) 	//poziom góra
	{
        LCD_WriteRAM(color);
	}
  	LCD_SetCursor(StartX, StartY+dY-1);  
	LCD_WriteRAM_Prepare();
	for(i=0; i < dX; i++)  //poziom dó³
	{
        LCD_WriteRAM(color);
	}
}

void LCD_DrawRect2(uint16_t StartX,uint16_t StartY,uint16_t dX,uint16_t dY,uint16_t color)
{
	//2px wide border - zweryfikowane doci¹ganie pixeli
  	uint16_t  i;
	for(i=0; i < dY; i++) 	 //pion
	{
	  	LCD_SetCursor(StartX, StartY + i);  
		LCD_WriteRAM_Prepare();
        LCD_WriteRAM(color);
        LCD_WriteRAM(color);
	  	LCD_SetCursor(StartX+dX-2, StartY + i);  
		LCD_WriteRAM_Prepare();
        LCD_WriteRAM(color);
        LCD_WriteRAM(color);
	}
  	LCD_SetCursor(StartX+2, StartY);  
	LCD_WriteRAM_Prepare();
	for(i=2; i < dX; i++) 	//poziom góra
        LCD_WriteRAM(color);
  	LCD_SetCursor(StartX+2, StartY+1);  
	LCD_WriteRAM_Prepare();
	for(i=2; i < dX; i++) 	//poziom góra
        LCD_WriteRAM(color);

  	LCD_SetCursor(StartX+2, StartY+dY-2);  
	LCD_WriteRAM_Prepare();
	for(i=2; i < dX; i++)  //poziom dó³
        LCD_WriteRAM(color);
  	LCD_SetCursor(StartX+2, StartY+dY-1);  
	LCD_WriteRAM_Prepare();
	for(i=2; i < dX; i++)  //poziom dó³
        LCD_WriteRAM(color);
}

//Zbig.
void LCD_FillRect(uint16_t StartX, uint16_t StartY, uint16_t dX, uint16_t dY, uint16_t color)
{
//zweryfikowane dociaganie pixeli
	int cnt = dX*dY;
	LCD_SetWindow(StartX, StartY, dX, dY);	//set window wrap area
	LCD_WriteRAM_Prepare(); 		// Prepare to write GRAM 

  	while(cnt--)
     	LCD_RAM = color; 				//write into window area, wrap to next line

	LCD_SetWindow(0, 0, 240, 320); //full screen
}



//Zbig.
void LCD_DrawHBar(uint16_t StartX, uint16_t StartY, uint16_t dX,uint16_t dY, int percent, uint16_t barColor, uint16_t bkColor, uint16_t frameColor)
{
	//x,y left top corner, dx-right, dy-down 
	//horizontal, horizontal bar rising from left to right
	if(percent < 0)
		percent = 0;
	if(percent >100)
		percent = 100;

	percent = (dX-2)*percent/100;
	LCD_FillRect(StartX+1, StartY+1, percent, dY-2, barColor);  
	LCD_FillRect(StartX+1+percent, StartY+1, dX-percent, dY-2, bkColor);  
	LCD_DrawRect(StartX, StartY, dX, dY, frameColor);
}
void LCD_DrawHBar2(uint16_t StartX, uint16_t StartY, uint16_t dX,uint16_t dY, int percent, uint16_t barColor, uint16_t bkColor, uint16_t frameColor)
{
	//x,y left top corner, dx-right, dy-down 
	//horizontal, horizontal bar rising from left to right
//	char buf[10];
	if(percent < 0)
		percent = 0;
	if(percent >100)
		percent = 100;
//	sprintf(buf, "%d%%", percent);
	percent = (dX-2)*percent/100;
	LCD_FillRect(StartX+1, StartY+1, percent, dY-2, barColor);  
	LCD_FillRect(StartX+1+percent, StartY+1, dX-percent, dY-2, bkColor);  
	LCD_DrawRect(StartX, StartY, dX, dY, frameColor);
	//vertical line at end of bar, with frame color
	LCD_DrawVLine(StartX+percent, StartY+1, dY-2, frameColor);

	//procenty - tylko problem z wyraŸnym kolorem na tle baru i t³a równoczesnie		
	//GUI_TextOpaque(StartX+2, StartY+3, buf, Blue2);
}

//Zbig.
void LCD_DrawVBar(uint16_t StartX, uint16_t StartY, uint16_t dX,uint16_t dY, int percent, uint16_t barColor, uint16_t bkColor, uint16_t frameColor)
{
	//X,Y - left top corner, dx-right, dy-down
	//vertical bar rising from bottom 
	percent = (dY-2)*percent/100;
	LCD_FillRect(StartX+1, StartY+1, dX-2, dY-2-percent, bkColor);  
	LCD_FillRect(StartX+1, StartY+dY-1-percent, dX-2, percent, barColor);  
	LCD_DrawRect(StartX, StartY, dX, dY, frameColor);
}

void LCD_Circle(uint16_t xc, uint16_t yc, uint16_t r, uint16_t color)
{
	//Circle Bresenham
    int x = 0; 
    int y = r; 
    int p = 3 - 2 * r;
    if (!r) 
		return;     
    while (y >= x) // only formulate 1/8 of circle
    {
        LCD_SetPoint(xc-x, yc-y, color);//upper left left
        LCD_SetPoint(xc-y, yc-x, color);//upper upper left
        LCD_SetPoint(xc+y, yc-x, color);//upper upper right
        LCD_SetPoint(xc+x, yc-y, color);//upper right right
        LCD_SetPoint(xc-x, yc+y, color);//lower left left
        LCD_SetPoint(xc-y, yc+x, color);//lower lower left
        LCD_SetPoint(xc+y, yc+x, color);//lower lower right
        LCD_SetPoint(xc+x, yc+y, color);//lower right right
        if (p < 0) p += 4*x++ + 6; 
              else p += 4*(x++ - y--) + 10; 
     } 
}


void LCD_FillCircle(uint16_t xc, uint16_t yc, uint16_t r, uint16_t color)
{
	//Circle Bresenham
    int x = 0; 
    int y = r; 
    int p = 3 - 2 * r;
    if (!r) 
		return;     
    while (y >= x) // only formulate 1/8 of circle
    {
		//zamiast +1 powinno byæ (x=0 ? 1: 2*x)
		LCD_DrawHLine(xc-x, yc+y, 2*x+1, color); 
		LCD_DrawHLine(xc-x, yc-y, 2*x+1, color);

        LCD_DrawHLine(xc-y, yc+x, 2*y+1, color);
        LCD_DrawHLine(xc-y, yc-x, 2*y+1,color);
        
		if (p < 0) p += 4*x++ + 6; 
              else p += 4*(x++ - y--) + 10; 
     } 
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

