//ee_ram.h
//EE_RAM non volatile "RAM" (EEPROM)
#include <inttypes.h>
#ifndef EE_RAM_H
#define EE_RAM_H

//local definitions, only for this module
#define EE_VAR_SIZE  6    //var ID (halfword) and value (2xhalfword = int32_t)
//-------------------------------------------------------------------------------------------------------
//this two defines can be modified according to chip used, and designated EE_RAM area 
#define EE_PAGESIZE	  0x800			//2kB pagesize
#define EE_START  (0x08080000 - 2*EE_PAGESIZE) //e.g FLASHEND (512k) minus 2 pages 
//-------------------------------------------------------------------------------------------------------
#define EE_PAGE0 (EE_START)
#define EE_PAGE1 (EE_START + EE_PAGESIZE)	//next page
//page status: first halfword on the page
#define EE_EMPTY  0xFFFF
#define EE_ACTIVE 0xAAAA
//any other status means invalid/damaged page
#define EE_STAT      0  //page status offset
#define EE_VAR_NUM ((uint16_t)((EE_PAGESIZE - 2) / EE_VAR_SIZE))	 //max number of "variables" value stored 
#define EE_VAR_FIRST 2   //first "variable" (3xhalfword) offset
#define EE_VAR_LAST  (EE_VAR_FIRST + (EE_VAR_NUM-1)*EE_VAR_SIZE) //last possible "variable" offset
//address is byte-addres, but we can read and write only double word
//and addres for this read/write will be always parity even (b0=0)
#define EE_READ 0
#define EE_WRITE 1
//limit numer of variables, to left space for cyclic usage of FLASH

#define EE_MAX_ID 0xFF  //1-byte variable ID limit variable count to 256, rest of page space will be left free for rewriting new values 

int32_t EE_Read(uint32_t id, int32_t defVal);
void EE_Write(uint32_t id, int32_t val);
void EE_Init(void);

//reserved for another set of EE_RAM variables
#define EE_START2 (EE_START - 2* EE_PAGESIZE)
//user language data 20kB
#define EE_USERLANG_SIZE (10*EE_PAGESIZE)
#define EE_USERLANG_START (EE_START2 - EE_USERLANG_SIZE) 

/*
USAGE:
	//call initialisation routine once at the beginning of main();
	EE_Init(); 
	//choose ID for your EE_RAM variable	
	#define MYVAR_ID  1	//assign any ID in range 0..255
	//declare "cache" RAM variable to speed up code operation, because EE_Read() takes quite long time 
	my_type my_var;		//variable can be of any type, althoug it will be stored in EERAM as int32_t
	MyVar = EE_Read(MYVAR_ID, 10);	//if (MYVAR_ID) was not previously stored, function will return default value=10
	EE_Write(MYVAR_ID, MyVar);		//store any value you need
	EE_Write(MYVAR_ID, 12+1);		//store any value you need, not necessarily from 'MyVar' variable
	MyVar = EE_Read(MYVAR_ID, 10);  //if ((MYVAR_ID) was previously stored, function will return last stored val=13, instead of default value 
*/

/*
//ESTORE - large storage for various data, new data will overlap oldest one
//Always after last written data there is at least one free space in this page (at the bottom of page)
//or in the next (completely empty) page
//it serves as "last data written" mark when data is readed, and it is identifiable place for next writting,
//allowing to identify active page to next write.
#define ES_PAGES   10		//20KB
#define ES_VARS_IN_PAGE		((uint16_t)(EE_PAGESIZE / EE_VAR_SIZE))
#define ES_VARS_IN_STORAGE	(ES_PAGES * ES_VARS_IN_PAGE)  //max number of stored infos
#define ES_START  (EE_START - (2+ES_PAGES)*EE_PAGESIZE)	  //just before EE_RAM minus ES size
#define ES_LAST   ((ES_VARS_IN_PAGE-1) * EE_VAR_SIZE) //offset for last data in page
void ES_Init(void);		//some initialisation will be needed
void ES_Clear(void);	//clears all stored data

//writes value identified by its 'id', returns index of stored value
uint16_t ES_Write32(uint16_t id, int32_t val);				//'id' in range between 0 and  0xFFFE 
//uint16_t ES_Write16(uint16_t id, int16_t v1, int16_t v2);	
//uint16_t ES_Write8(uint16_t id, int8_t v1, int8_t v2, int8_t v3, int8_t v4);	

//reads value form index, returns data id
uint16_t ES_Read32(uint16_t index, int32_t * val);		//'index' in range between 0 and  ES_SIZE 
//uint16_t ES_Read16(uint16_t id, int16_t *v1, int16_t *v2);	
//uint16_t ES_Read8(uint16_t id, int8_t *v1, int8_t *v2, int8_t *v3, int8_t *v4);	
*/

#endif

