

/*
http://techdocs.altium.com/display/ADRR/NEC+Infrared+Transmission+Protocol

NEC IR
noœna 38kHz
start: 9ms impuls + 4,5ms przerwa
bit0 562,5us impuls + 562,5 us przerwa (total 1,125ms)
bit1 562,5us impuls + 1,685 ms przerwa (total 2,25ms)
end 562,5us impuls
LSB first
ramka:
START (9ms impuls+5ms przerwa)
8 bit adres
8bit negacja adresu
8bit dane
8bit negacja danych
END (562,5us impuls)
total 67ms + impuls STOP
repeat code po 108 ms, jeœli guzior wci¹¿ wciœniêty
9ms pulse
2,25ms space
562,5 pulse end.
implementacja:
wejœcie GPIO/EXTI rising edge interrupt
TIMER:
prescaler 4050, tick = 56,25us (x10)
period = 1300  (78ms total)
timer startujemy na pierwszym zboczu, wy³¹czamy po przekrêceniu.
Na przekrêcenie timera oznaczamy dane gotowe do analizy
na kazde zbocze loguje czas timera
*/

#define	MAX_IR 34
#define	IRREADY 0x100;
volatile int irbuf[34];
volatile int iridx = 0;
	 
void EXTI_Irq()
{
	if( TIMx->CNT == 0) //first time, pierwsze zbocze, timer zablokowany
	{

		Enable(TIMx);
		iridx = 0;
	}
	else //timer dzia³a, jestesmy w czasie analizy impulsów
	{
		if(iridx < MAX_IR)
		{
			irbuf[iridx++] = TIMx->CNT;
		}
	}
	ClearIRQFlag();
}

void TIMx_Irq()
{
	// if( TIMx->SR == OVERFLOW) always
	Disable(TIMx);
	//stop timer
	iridx |= IRREADY;
	//mark ready
	TIMx->CNT = 0;
	ClearIRQFlag();
}

//analiza danych:
int ir_addr;
int	ir_cmd;
int ir_valid;

int Get8(int offs)
{
	int a = 0;
	int c;
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
		if(c > 36 && c < 44)
		{
			a = (a >> 1) | 0x80;
			continue;
		}
		//error
		return -1;
	}
	return a;
}

int	IrReceive()
{
	int i, a=0,b=0, c;
	if( !(iridx & IRREADY) )
		return 0;

	ir_valid = 0;
	//start pulse: nominal 240
	if( irbuf[0] < 220 || irbuf[0] > 260)
	{
		iridx = 0;
		//error
		return 0;
	}
	a = Get8(1);
	b = ~Get8(9); //negacja
	if(a != b || a < 0)
		return 0;

	ir_addr = a;

	a = Get8(17);
	b = ~Get8(25); //negacja
	if(a != b || a < 0)
		return 0;

	ir_data = a;
	return ir_valid = 1; //OK
}
