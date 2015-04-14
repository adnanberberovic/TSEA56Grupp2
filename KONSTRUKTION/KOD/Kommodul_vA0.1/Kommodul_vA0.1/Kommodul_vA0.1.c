/*
 *  Komm_buss.c
 *	Kommunikationsmodul
 *  Author: adnbe196, mansk700
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>

#define BuffSize 100
char outSPDR[BuffSize];
char inSPDR[BuffSize];
int i = 0;
int posBuff_SPI = 0;

int rBTin = 0;
int wBTin = 0;
int rBTout = 0;
int wBTout = 0;
int *readPos_BTin = &rBTin;
int *writePos_BTin = &wBTin; // Add BTout positioners aswell if needed.
int *readPos_BTout = &rBTout;
int *writePos_BTout = &wBTout;
int BT_received_flag = 0;
int BT_sent_flag = 0;

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
			SPDR = outSPDR[posBuff_SPI]; //Sett next bit to send from outSPDR-buffer
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

void Write_Buffer(char buffer[BuffSize], char data, int *position)
{
	if ((*position) == (BuffSize - 1)) // If end of buffer restart from first pos, done with read.
	{
		BT_received_flag = 1;
		(*position) = 0; 
		send_BT_buffer(inBT); //Echo back inBT *****************ONLY FOR TEST******************
	}
	buffer[(*position)] = data; //Add data to correct location
	(*position)++;
}

char Read_Buffer(char buffer[BuffSize], int *pos_read)
{
	char data;
	if ((*pos_read) == (BuffSize - 1)) // End of buffer, reset from start 
	{
		(*pos_read) = 0;
	}
	data = buffer[(*pos_read)]; //return next value in queue
	(*pos_read)++;
	return data;
}


// Receive complete - triggered by interrupt
ISR(USART0_RX_vect) 
{
	BT_received_flag = 0;
	char data = UDR0; //Get received value
	Write_Buffer(inBT, data, writePos_BTin); //Writes data to buffer in order they are received

	UCSR0B |= (1<<UDRIE0); //Enable empty buffer interrupt


}

void send_BT_buffer(char buffer[BuffSize] )
{
	strncpy(outBT, buffer, BuffSize); //Copy buffer to send to outBT
	UCSR0B &= ~(1<<UDRIE0);	//Enable UDRE interrupt flag -> send when empty dataregister
}

// Empty dataregister = send next character
ISR(USART0_UDRE_vect)
{
	if ((*readPos_BTout) == BuffSize) //Read entire buff and sent it
	{
		BT_sent_flag = 1; // done with transmission
		UCSR0B &= ~(1<<UDRIE0); //Disable UDRE interrupt, All data is sent. 
	}
	else
	{
		BT_sent_flag = 0;
		UDR0 = Read_Buffer(outBT, readPos_BTout); //Send back the next value in out-buffer. 														
	}
		
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