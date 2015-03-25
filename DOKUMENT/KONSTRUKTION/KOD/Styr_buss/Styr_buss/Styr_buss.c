/*
 *  styrmodul_test1.c
 *	Styrmodul
 *  Author: adnbe196, mansk700
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <stdio.h>

//Setup data direction registers @ ports for out/inputs.
void Styr_InitPortDirections(void)
{
	DDRA = 1<<DDA0 | 1<<DDA1 | 1<<DDA2 | 1<<DDA3 | 1<<DDA6;
	DDRB = 1<<DDB0 | 1<<DDB1 | 1<<DDB2 | 1<<DDB3 | 1<<DDB4 | 1<<DDB5 | 1<<DDB7;
	DDRD = 1<<DDD0 | 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4 | 1<<DDD5 | 1<<DDD6 | 1<<DDD7;
} 

//Setups port values, more specifically puts SS on high.
void Styr_InitPortValues(void)
{
	PORTB = 1<<PORTB3 | 1<<PORTB4;
}

//Configures device as spi master.
void SPI_MasterInit(void)
{
	SPSR = 0<<SPI2X;
	SPCR = 1<<SPIE | 1<<SPE | 1<<DORD | 1<<MSTR | 0<<CPOL | 0<<CPHA | 1<<SPR1 | 1<<SPR0;
	//SPIE: SPI interrupt enable. Set to 1 to allow interrupts
	//SPE: SPI Enable. Set to 1 to allow SPI communication
	//DORD: Data order. Set to 1 to transmit LSB first, MSB last.
	//MSTR: Master select. Set to 1 to set master.
	//CPOL and CPHA: set to 0 to sample on rising edge and setup on falling edge.
	//SPI2X, SPR1, SPR0, set to 0,1,1 to scale clock with f_osc/128.
	//Bus config similar to comm and sensor module, though set 0<<MSTR
	
}

//Initiates commiunication with other modules.
unsigned char SPI_MasterTransmit(unsigned char cData, char *target)
{
	if (!strncmp(target, "komm", 4))
	{
		PORTB = 0<<PORTB4;
	}
	else if (!strncmp(target, "sens", 4))
	{
		PORTB = 0<<PORTB5;
	}
	//Load data into SPI data register.
	SPDR = cData; 
	
	//wait until transmission completes.
	while(!(SPSR & (1<<SPIF)));
	
	PORTB = 1<<PORTB4 | 1<<PORTB5;
	
	return SPDR;
}

//Interrupt method runs when SPI transmission/reception is completed.
ISR(SPI_STC_vect)
{
	
}

int main(void)
{
	unsigned char SPDRrec_ = 0;
	sei();
	sleep_enable();
	Styr_InitPortDirections();
	Styr_InitPortValues();
	SPI_MasterInit();
	SPDR = 0xFE;
	SPDRrec_ = SPI_MasterTransmit(0x0F, "komm");
	SPDRrec_ = SPI_MasterTransmit(0x0F, "komm");
	while(1)
    {
		
	}
}