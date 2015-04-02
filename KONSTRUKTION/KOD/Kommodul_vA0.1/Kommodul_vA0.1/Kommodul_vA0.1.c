/*
 *  Komm_buss.c
 *	Kommunikationsmodul
 *  Author: adnbe196, mansk700
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define BuffSize 10
char outSPDR[] = {'1','2','3','4','5','6','7','8','9','0'};
char inSPDR[BuffSize];
int i = 0;
int posBuff = 0;
int posBuff_SPI = 0;
int sendFlag = 0;
char outBT[BuffSize];
char inBT[BuffSize];

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


// Interrupt method runs when SPI transmission/reception is completed.
ISR(SPI_STC_vect)
{
		if (posBuff_SPI < (BuffSize - 1))
		{
			inSPDR[posBuff_SPI] = SPDR; // Save received char in inSPDR-buffer
			SPDR = outSPDR[posBuff_SPI]; //Send sign from outSPDR-buffer
			posBuff_SPI++; // add 1 to bufferpos
		}
		else if (posBuff_SPI == (BuffSize - 1))
		{
			inSPDR[posBuff_SPI] = SPDR; //save received char in inSPDR-buffer
			SPDR = outSPDR[posBuff_SPI]; //Send last sign from outSPDR-buffer
			posBuff_SPI = 0; //Set bufferpos to restart
		}
		else 
		{
			SPDR = 'E';
		}
	
	
}

// Set up and enable Bluetooth
void BT_init(void)
{
	UBRR0H = 0x00; //correct value to change baud rate
	UBRR0L = 0x07;//^^ same ^^ with a 14.7 mhz, scale with 1111 (7)
	
	UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (0<<UCSZ02) | (1<<RXCIE0) | (1<<TXCIE0) | (0<<UDRIE0);
	/* RXCI, TXCI Complete transmission and complete interrupt is enabled
	 * UDRIE0 not set, disabled interrupts due to UDRE0 flag. Data register empty
	 * TXEN, TXEN, transmission and receiver enable 
	 * UCSZ02 sets the third bit, defining framesize
	*/
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
	/* UMSEL0 = 0 setting Asynchronous operation
	 * UPM01:0 = 0, Pairty disabled
	 * USBS0 = 0, 1 stop bit
	 * UCSZ01:0 = 1, char size = 8
	 */
}


// Transmit data via BT.
void BT_transmit(unsigned char outData)
{
	UDR0 = outData;
	// UCSR0B |= (1<<UDRIE0); // enable 'empty buffer interrupt'
}

// Receive complete - triggered by interrupt
ISR(USART0_RX_vect) 
{
	if (posBuff == 0)
	{
		memset(&inBT,' ',10);
	}
	if (posBuff < BuffSize)
	{
		inBT[posBuff] = UDR0; //Load bit 1-9.
		BT_transmit(inBT[posBuff]);
		posBuff++;
	}
	else if (posBuff == BuffSize )
	{
		inBT[posBuff] = UDR0; //Load 10th bit
		BT_transmit(inBT[posBuff]); // Send back incoming
		posBuff = 0;	
	}

}


// Empty dataregister = send next character
ISR(USART0_UDRE_vect)
{
	//if (posBuff < BuffSize)
	//{
		//UDR0 = outBT[posBuff]; //Load bit 1-9.
		//posBuff++;
	//}
	//else
	//{
		//UDR0 = outBT[posBuff]; //Load 10th bit
		//posBuff = 0;
		//UCSR0B &= ~(1<<UDRIE0); // disable 'empty buffer interrupt'
	//}
}

int main(void)
{
	sleep_enable();
	Komm_InitPortDirections();
	Komm_InitPortValues();
	SPI_SlaveInit();
	BT_init();
	sei();
	while(1)
	{
		strncpy(outSPDR, inBT, BuffSize); //copy data from inBT to outbuffer (SLAVE)
	}
}