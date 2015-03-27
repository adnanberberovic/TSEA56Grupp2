/*
 *  Komm_buss.c
 *	Kommunikationsmodul
 *  Author: adnbe196, mansk700
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

char outSPDR[] = "ABCDEFGHIJKLMNOP";
char inSPDR;
int i = 0;
// Setup data direction registers @ ports for out/inputs.
void Komm_InitPortDirections(void)
{
	DDRB = 1<<DDB6;
	DDRD = 1<<DDD1;
}

// Initiate port values.
void Komm_InitPortValues(void)
{
	PORTB = 1<<PORTB4;
}

// Configures device as spi slave.
void SPI_SlaveInit(void)
{
	SPSR = 0<<SPI2X;
	SPCR = 1<<SPIE | 1<<SPE | 1<<DORD | 0<<MSTR | 0<<CPOL | 0<<CPHA | 1<<SPR1 | 1<<SPR0;
	//SPIE: SPI interrupt enable. Set to 1 to allow interrupts
	//SPE: SPI Enable. Set to 1 to allow SPI communication
	//DORD: Data order. Set to 1 to transmit LSB first, MSB last.
	//MSTR: Master select. Set to 1 to set master.
	//CPOL and CPHA: set to 0 to sample on rising edge and setup on falling edge.
	//SPI2X, SPR1, SPR0, set to 0,1,1 to scale clock with f_osc/128.
	//Bus config similar to comm and sensor module, though set 0<<MSTR
	
}


// Bus transmission. Send and receive data from styrmodulen.
void SPI_SlaveTransmit(unsigned char cData)
{
	inSPDR = SPDR;
	//Load data into SPI data register
	SPDR = cData; 
}

// Interrupt method runs when SPI transmission/reception is completed.
ISR(SPI_STC_vect)
{
	i = SPDR;
	SPDR = outSPDR[i];
	
}

int main(void)
{
	inSPDR = 0x00;
	sleep_enable();
	Komm_InitPortDirections();
	Komm_InitPortValues();
	SPI_SlaveInit();
	sei();
	while(1)
	{
		
	}
}