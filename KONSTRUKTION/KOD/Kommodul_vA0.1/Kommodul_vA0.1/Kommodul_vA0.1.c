/*
 *  Komm_buss.c
 *	Kommunikationsmodul
 *  Author: adnbe196, mansk700
 */ 
#define F_CPU 20000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <alloca.h>
#include <stdio.h>
#include <ctype.h>


#define BuffSize 100
char outSPDR[BuffSize];
char inSPDR[BuffSize];
char *SPI_queue[BuffSize];

char testbuffer_[] = "All I do is win!";
volatile uint8_t pos_queue = 0;
volatile uint8_t *pos_SPIqueue = &pos_queue;
volatile uint8_t posSPIout = 0;
volatile uint8_t posSPIin = 0;
volatile uint8_t *posBuff_SPIout = &posSPIout;
volatile uint8_t *posBuff_SPIin = &posSPIin;
volatile uint8_t ongoing_SPI_transfer = 0;

char outBT[BuffSize];
char inBT[BuffSize];
volatile uint8_t rBTin = 0;
volatile uint8_t wBTin = 0;
volatile uint8_t rBTout = 0;
volatile uint8_t wBTout = 0;
volatile uint8_t *readPos_BTin = &rBTin;
volatile uint8_t *writePos_BTin = &wBTin; // Add BTout positioners aswell if needed.
volatile uint8_t *readPos_BTout = &rBTout;
volatile uint8_t *writePos_BTout = &wBTout;
volatile uint8_t BT_received_flag = 0;
volatile uint8_t BT_sent_flag = 0;

uint8_t sendFlag = 0;

struct node { // definition of the linked list node
	int val;
	struct node *next;
	};

typedef struct node buffer_; // buffer_ definieras


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

void send_BT_buffer(char buffer[BuffSize] )
{
	strncpy(outBT, buffer, BuffSize); //Copy buffer to send to outBT
	UCSR0B |= (1<<UDRIE0);	//Enable UDRE interrupt flag -> send when empty dataregister
	// maby add while UDRIE0 = 0 here to counter multiple send_BT_buffer in a row
}


void Write_Buffer(char *buffer, char data, volatile uint8_t *position)
{
	if ((*position) == (BuffSize - 2)) // If end of buffer restart from first pos, done with read.
	{
		(*position) = 0; 
		send_BT_buffer(inBT); //Echo back inBT *****************ONLY FOR TEST******************
	}
	buffer[(*position)] = data; //Add data to correct location
	(*position)++;
	
}

char Read_Buffer(char *buffer, volatile uint8_t *pos_read)
{
	char data = buffer[(*pos_read)]; //return next value in queue
	(*pos_read)++;
	return data;
}


// Receive complete - triggered by interrupt
ISR(USART0_RX_vect) 
{
	BT_received_flag = 0;
	char data = UDR0; //Get received value
	Write_Buffer(inBT, data, writePos_BTin); //Writes data to buffer in order they are received
	
}


// Empty dataregister = send next character
ISR(USART0_UDRE_vect)
{
	if ((*readPos_BTout) == BuffSize) //Read entire buff and sent it
	{
		(*readPos_BTout) = 0;
		BT_sent_flag = 1; // done with transmission
		UCSR0B &= ~(1<<UDRIE0); //Disable UDRE interrupt, All data is sent. 
	}
	else
	{
		BT_sent_flag = 0;
		UDR0 = Read_Buffer(outBT, readPos_BTout); //Send back the next value in out-buffer. 														
	}
		
}

void send_SPI_buffer(char *buffer)
{
	
	memset(outSPDR, '\0', BuffSize);
	strncpy(outSPDR, buffer, BuffSize); //Copy what to send into outSPDR
	(*posBuff_SPIout) = 0; // start reading from beginning
	ongoing_SPI_transfer = 1; //something to send.
	while(((ongoing_SPI_transfer == 1) & !(outSPDR[(*posBuff_SPIout)] == '\0'))); //Wait until entire buffer is sent.
	
}

// Interrupt method runs when SPI transmission/reception is completed.
ISR(SPI_STC_vect)
{
	char data = SPDR;
	// Add condition for WCOL to avoid missing datawrite
	
	Write_Buffer(inSPDR, data, posBuff_SPIin); //Write received value to inBT buffer
	
	if (ongoing_SPI_transfer == 1) //something to send
	{
		if ( (*posBuff_SPIout) == BuffSize ) // end of buffer, all sent. sett pointer to beginning.
		{
			(*posBuff_SPIout) = 0;
			ongoing_SPI_transfer = 0;
		}
		
		SPDR = Read_Buffer(outSPDR, posBuff_SPIout); //Add next element in buffer to SPDR.
	}
	else 
	{
		SPDR = '-'; //Return blank.
	}
}

void add_node(buffer_* lst_head, int val)
{
	buffer_ * curr = lst_head;

	while(curr->next != NULL) // step to end of list
	{
		curr = curr->next;
	}
	curr->next = (buffer_ *)malloc(sizeof(buffer_));
	curr->next->val = val;
	curr->next->next = NULL; // Add node last.
}

int main(void)
{
	
	
	buffer_ *head = (buffer_ *)malloc(sizeof(buffer_)); //Define head of list and alloc memory.
	head->next= NULL;
	head->val = 0;
	
	
	
	
	sleep_enable();
	Komm_InitPortDirections();
	Komm_InitPortValues();
	SPI_SlaveInit();
	BT_init();
	sei();
	char testbuffer2_[] = " 9/11 is a lie. ";
	while(1)
	{
		send_SPI_buffer(testbuffer_);
		send_SPI_buffer(testbuffer2_);
		
	}
}

/* Todo:
		- Göra så att läsning sker tills buffern är null. Någon form av stoppbit i buffer, måhända skriva över nyligen läst med \0.
		- Avsluta alla SPI-buffrar med något bra tecken.
*/