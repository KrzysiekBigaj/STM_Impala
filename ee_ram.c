//-------------------------------------------------------------------------------------------
//(c) Zbig 2011
//modifications, use or publishing without written permission of author is stricty prohibited
//-------------------------------------------------------------------------------------------
//ee_ram.c
#include"ee_ram.h"
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include <string.h>


/*
EE_RAM stores max 256 variables of 'int32_t' type.
Each variable is identified by its ID (range from 0 to 255)
in flash each stored variable consists of 3 halfwords:
	var_id			//identifier of stored variable 
	var_value_lo	//low part of value
	var_value_hi	//high part of value

EE_RAM uses 2 FLASH pages, one active for current read/write,
and another empty (filled by oxFF's).

each page is organised as follows:
  page_status
  first_var_id
  first_var_val_lo
  first_var_val_hi
	.....
  last_var_id
  last_var_val_lo
  last_var_val_hi

Writing variable (new variable or new value of previously stored variable) 
means write new (id+val) below all previously stored (id+val) on active page

Reading variables means finding (in active page) last stored ID and getting following (last stored) variable value. 

When there is no free room for new value in current (active) page, last stored values of all variables (IDs)
were moved to another (empy) page (which becomes active page) and current page is erased, 
then new wariable/its value is stored in active page

In case of invalid pages signature, both pages were cleared during EE_init() -without any attempt to recover damaged values
In case of page failure discovered durnig R/W operation, those operation will take no effect (Read will return default value) 
*/

void EE_Rewrite(uint32_t from, uint32_t to)
{
	uint32_t src, dst=0;
	uint16_t id, hv;
	//here store info that variable of certain id already moved
	uint8_t moved[EE_MAX_ID+1];		  
	memset( moved, 0, EE_MAX_ID+1);
	//be sure page status is valid

	FLASH_Unlock();
	//scan page from bottom (last stored value) to top,
	//and store last values into new page starting from top
	dst = to + EE_VAR_FIRST;
	for(src = from + EE_VAR_LAST; src >= from + EE_VAR_FIRST; src -= EE_VAR_SIZE)
	{
		id = (*(__IO uint16_t*)src);
		if( id <= EE_MAX_ID ) //valid variable id, no empty space
		{
			if( moved[id] == 0 ) //not moved yet
			{
				//rewrite variable 
				FLASH_ProgramHalfWord(dst, id); //variable id
				hv = (*(__IO uint16_t*)(src+2));	//first half word
				FLASH_ProgramHalfWord(dst+2, hv);
				hv = (*(__IO uint16_t*)(src+4));	//second half word
				FLASH_ProgramHalfWord((dst+4), hv);
				dst += EE_VAR_SIZE;	
				//mark as moved, so no previous values will be moved again 	
				moved[id] = 1;	
			}
		}
	}
	//erase old page 
	FLASH_ErasePage(from);
	//and make new page active
	FLASH_ProgramHalfWord(to + EE_STAT, EE_ACTIVE);
	FLASH_Lock();
}


uint32_t EE_GetActivePageAddr( uint8_t rw )
{
	uint16_t stat = (*(__IO uint16_t*)(EE_PAGE0 + EE_STAT));
	uint16_t last = (*(__IO uint16_t*)(EE_PAGE0 + EE_VAR_LAST));
	if(stat == EE_ACTIVE)
	{
		if( rw == EE_READ)	
			return EE_PAGE0;	
		//write, we need to check free space for new "variabble" value
		if( last == EE_EMPTY)
			return EE_PAGE0;	
		//first page is full 
		//we need to rewrite active variables
		//from page 0 into page 1
		EE_Rewrite(EE_PAGE0, EE_PAGE1);
		//finally return page1
		return EE_PAGE1;
	}
	//first page empty, chcek second
	stat = (*(__IO uint16_t*)(EE_PAGE1 + EE_STAT));
	last = (*(__IO uint16_t*)(EE_PAGE1 + EE_VAR_LAST));
	if(stat == EE_ACTIVE)
	{
		if( rw == EE_READ)	
			return EE_PAGE1;	
		//write, we need to check free space for new "variabble" value
		if( last == EE_EMPTY)
			return EE_PAGE1;	
		
		//second page is full 
		//we need to rewrite active variables
		//from page 1 into page 0
		EE_Rewrite(EE_PAGE1, EE_PAGE0);
		//finally return page0
		return EE_PAGE0;
	}
	//both pages empty?
	if(stat == EE_EMPTY)
	{
		//make this page active
		FLASH_Unlock();
		FLASH_ProgramHalfWord(EE_PAGE1 + EE_STAT, EE_ACTIVE);
		FLASH_Lock();
		return EE_PAGE1;		
	}
	return 0;
}

int32_t EE_Read(uint32_t id, int32_t defVal)
{
	uint32_t src;
	//find variable in active page	
	uint32_t page = EE_GetActivePageAddr( EE_READ );
	if( !page )	//will never happens when EE_Init() was called and FLASH was not dameged 
		return defVal;
	//scan starting from last (stored) variable searching for last stored value of this id
	for(src = page + EE_VAR_LAST; src >= page + EE_VAR_FIRST; src -= EE_VAR_SIZE)
	{
		uint32_t eeid = (uint32_t)(*(__IO uint16_t*)src);
		if( id == eeid )		//id matched, get value
		{
			uint32_t val = (*(__IO uint32_t*)(src+2));	//4 byte value at once
			return (int32_t)val;
		}
	}
	//variable not stored yet, return default value
	//it's no need to store default value :-)
	return defVal;
}

int32_t EE_IsEqual(uint32_t id, int32_t orgVal)
{
	uint32_t src;
	//find variable in active page	
	uint32_t page = EE_GetActivePageAddr( EE_READ );
	if( !page )	//will never happens when EE_Init() was called and FLASH was not dameged 
		return 0;
	//scan starting from last (stored) variable searching for last stored value of this id
	for(src = page + EE_VAR_LAST; src >= page + EE_VAR_FIRST; src -= EE_VAR_SIZE)
	{
		uint32_t eeid = (uint32_t)(*(__IO uint16_t*)src);
		if( id == eeid )		//id matched, get value
		{
			int val = (*(int*)(src+2));	//4 byte value at once
			return (val == orgVal); //=1 when equal values
		}
	}
	//not exists yet
	return 0;
}

void EE_Write(uint32_t id, int32_t val)
{
	uint32_t dst, page;
	if( EE_IsEqual(id, val))
		return;	//valid value already stored. Avoid re-writing 
	//write variable into active page	
	page = EE_GetActivePageAddr( EE_WRITE );
	if( !page )	//will never happens when EE_Init() was called and FLASH was not dameged 
		return; 
	//search for empty space
	for(dst = page + EE_VAR_FIRST; dst <= page + EE_VAR_LAST; dst += EE_VAR_SIZE)
	{
		uint32_t eeid = (uint32_t)(*(__IO uint16_t*)dst);
		if( eeid == EE_EMPTY )		//empty, unused space
		{
			FLASH_Unlock();
			FLASH_ProgramHalfWord(dst, id); //variable id
			FLASH_ProgramHalfWord(dst+2, (val & 0xFFFF) );
			FLASH_ProgramHalfWord(dst+4, (val >> 16) );
			FLASH_Lock();
			return;
		}
	}
	//something goes wrong. Impossible :-)
}

void EE_Init(void)
{
	uint16_t stat;
	//unlock pages
	FLASH_Unlock();
	//check pages for validity and format if necessary
	stat = (*(__IO uint16_t*)(EE_PAGE0 + EE_STAT));
	if(stat != EE_ACTIVE && stat != EE_EMPTY)
		FLASH_ErasePage(EE_PAGE0);
	stat = (*(__IO uint16_t*)(EE_PAGE1 + EE_STAT));
	if(stat != EE_ACTIVE && stat != EE_EMPTY)
		FLASH_ErasePage(EE_PAGE1);
	FLASH_Lock();
}

/*
EESTORAGE stores a lot of 'int32_t' values (log file)
Each variable is classified by its own ID (range from 0 to 0xFFFE)
In flash each stored value consists of 3 halfwords:
	value_id	//identifier of stored value 
	value_lo	//low part of value
	value_hi	//high part of value

ESTORAGE uses a lot of FLASH pages, 

each page is organised as follows:
  first_val_id
  first_val_lo
  first_val_hi
	.....
  last_val_id
  last_val_lo
  last_val_hi

Writing value means write new (id+val) below all previously stored (id+val) on active page
Reading 'index' variable means finding value stored at particular index strating from 0 (beginning of storage)
When there is no free room for new value in last page, first page is erased, 
then new value is stored in first page (overlaping previously stored data)
*/

/*
uint32_t ES_GetActivePage()				//page with free at least one place for data
{
	uint32_t  addr;
	uint8_t   page;
	//search for free page (check last data location)
	for(addr = ES_START, page = 0; page < ES_PAGES; page++, addr++)
	{
		//page is active when at least one (last on page) space is free
		if( (*(__IO uint32_t*)(addr + ES_LAST)) == EE_EMPTY)
			return addr;
	}
	return 0;	//something wrong, probably whole ES structure is damaged
}

uint16_t ES_Write32(uint16_t id, int32_t val)				//'id' in range between 0 and  0xFFFE 
{
	uint32_t  addr;
	uint32_t  free;
	uint16_t  index;
	uint16_t  page; 
	//look for page with free space for writting
	addr = ES_GetActivePage();
	if(addr == 0)
	{	
		//format and use PAGE0 ?	
		FLASH_ErasePage(ES_START);  //prepare first page fo data - overlap oldest data
		addr = ES_START;
	}
	//in this page is at least one free space for write next data
	//look for first free space 
	for( index=0, free = addr; index < ES_VARS_IN_PAGE; free += EE_VAR_SIZE, index++)
	{
		if( (*(__IO uint32_t*)free) == EE_EMPTY)
		{
			FLASH_ProgramHalfWord(free, id); //variable id
			FLASH_ProgramHalfWord(free+2, (val & 0xFFFF) );
			FLASH_ProgramHalfWord(free+4, (val >> 16) );

			page = (free-ES_START)/ES_VARS_IN_PAGE;
			if(free == addr + ES_LAST)
			{
				//this page is full, make room in next page
				if( (page+1) < ES_PAGES )
					FLASH_ErasePage(page+1);		//prepare next page
				else
					FLASH_ErasePage(ES_START);  //prepare first page for data - overlap oldest data
			}
			//return index - for debug purposes to check validity of R/W operations
			return index + page*ES_VARS_IN_PAGE;
		}
	}
	//never here, until fatal FLASH disaster 
	return 0xFFFF;
}

uint16_t ES_Read32(uint16_t index, int32_t * val)		//'index' in range between 0 and  ES_SIZE 
{
	uint32_t uv;
	uint16_t page;
	uint16_t offs;
	uint32_t addr;
	uint32_t id;

	if( index >= ES_VARS_IN_STORAGE)
		return 0xFFFF;

	page = index / ES_VARS_IN_PAGE;
	offs = (index % ES_VARS_IN_PAGE) * EE_VAR_SIZE;
	addr = ES_START + page*EE_PAGESIZE + (offs* EE_VAR_SIZE);
	id =  (*(__IO uint32_t*)addr);
	
	if( id != 0xFFFF)
	{
		uv = (*(__IO uint32_t*)(addr+2));
		uv += (*(__IO uint32_t*)(addr+4)) << 16;
		*val = (int32_t)uv;
	}
	return id;
}

void ES_Init()
{
	FLASH_Unlock();
	if( !ES_GetActivePage())
	{
		FLASH_ErasePage(ES_START);  //prepare first page for data - overlap oldest data
	}
}

*/

int ProgramWRO4(int addr, int val)
{
	//write once - jednorazowy zapis flash 4B pod podany adres
	//zapisywana komórka musi byc skasowana!
	//komenda przeznaczona do zapisywania np. numeru seryjnego itp.
	//dla bezpieczenstwa adres musi byc w zakresie 30-32k 
	//czyli ostatni sektor przed firmware - miejsce na wszelakie "wynalazki" tego typu
	//lub powyzej 128kB (omija przynajmniej czêœæ firmware)
	FLASH_Unlock();   
	addr = (addr & 0x8FFFFFC); 		//4 bytes boundary
	if( addr < 0x8007800)
		return 0;
	if( addr >= 0x8008000)
		return 0; 
	if( addr < 0x8020000) 
		return 0;
	if( *(int*)addr != 0xFFFFFFFF)	//check unused location
		return 0;
	FLASH_ProgramWord(addr, val);
	FLASH_Lock();
	return 1;
}


/*
////log area:
//start 128KB
//end   FLASHEND-4KB
#define LOG_BEG 0x8020000 //+128KB
#define LOG_END	EE_START2
#define LOG_PAGES ((int)((LOG_END-LOG_BEG)/EE_PAGESIZE))
#define LOG_SIZE  (LOG_PAGES*EE_PAGESIZE)

#define LOG_DATASIZE (6+3+1) //3x2b gyro, 3x1b acc, 1x1b speed
#define LOG_PDATANUM  ((int)((EE_PAGESIZE-4)/LOG_DATASIZE)) //iloœæ paczek danych na stronie

int logId  = -1; 	//not initialised
int logPage   = -1; //first
int logInPage = 0; 		//first 

void LogInit()
{
	logId = (*(__IO int*)(LOG_BEG)) + 1; //next log ID
	logPage = 0;
	logInPage = 0;
}

void Log()
{
	//rosn¹cy logid pozwoli na okreslenie koñca logowanych danych 
	//tzn kolejna strona z innym logid to koniec danych 	
	if(logId < 0 || logPage >= LOG_PAGES)
		return; //not started or full

	if( logInPage >= LOG_PDATANUM)
	{
		//set next page
		if(++logPage >= LOG_PAGES)
			return;
		logInPage = 0; 
	}

	if(logInPage == 0)
	{
		//clear page
		//set logId at the beginning of page
	}	 
	//set addr acording to page index and logInPage
	//write log data
	logInPage++;				
}
*/
