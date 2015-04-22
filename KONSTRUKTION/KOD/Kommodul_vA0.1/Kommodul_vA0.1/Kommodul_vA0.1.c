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


#define BuffSize 10
char outSPDR[BuffSize];
char inSPDR[BuffSize];
char *SPI_queue[BuffSize];

int8_t testbuffer3_[] = {'9','8','7','6','5','4','3','2','1','0'};
int8_t testbuffer2_[] = {9,8,7,6,5,4,3,2,1,0};
int8_t testbuffer1_[] = {'1','2','3','4','5','6','7','8','9','0'};	
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

//*********** BUFFER STRUCT ****************

struct node { // definition of the linked list node
	int8_t val;
	struct node *next;
	};

typedef struct node buffer_; // buffer_ definieras

buffer_ *head_SPIout = NULL;
buffer_ *head_SPIin = NULL;

buffer_ *head_BTout = NULL;
buffer_ *head_BTin = NULL;

//_______________________________________________


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

void add_node(buffer_* lst_head, int8_t val)
{
	if (lst_head != NULL)
	{
		buffer_ * curr = lst_head;
		while (curr->next != NULL){ // step to end of list
			curr = curr->next;
		}
		curr->next = (buffer_ *)malloc(sizeof(buffer_));
		curr->next->val = val;
		curr->next->next = NULL; // Add node last.
	}
	else
	{
		lst_head = (buffer_ *)malloc(sizeof(buffer_));
		lst_head->val = val;
		lst_head->next = NULL;
	}

	//buffer_ * curr = lst_head;
	//while(curr->next != NULL) // step to end of list
	//{
	//curr = curr->next;
	//}
	//curr->next =
	//curr->next->val = val;
	//curr->next->next = NULL; // Add node last.
}

void Write_Buffer(char *buffer, char data, volatile uint8_t *position)
{
	if ((*position) == (BuffSize - 2)) // If end of buffer restart from first pos, done with read.
	{
		(*position) = 0; 
		//send_BT_buffer(inBT); //Echo back inBT *****************ONLY FOR TEST******************
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

int8_t pop_node(buffer_ ** lst_head)
{
	buffer_* next_node = NULL;
	int8_t retval = 0;
	
	if ( *lst_head != NULL)
	{
		next_node = (*lst_head)->next;
		retval = (*lst_head)->val;
		free(*lst_head);
		*lst_head = next_node;
	}
	
	return retval;
}

void BT_send(uint8_t val)
{
	add_node(head_BTout, val);
	UCSR0B |= (1<<UDRIE0); // activate interrupt
	//UDR0 = (char)val;
}

void flush_list(buffer_ ** lst_head)
{
	while(*lst_head != NULL)
	{
		pop_node(lst_head);
	}
}

void send_BT_buffer(int8_t buffer[BuffSize] )
{
	//flush_list(&head_BTout);
	//strncpy(outBT, buffer, BuffSize); //Copy buffer to send to outBT
	for(int i = 0; buffer[i]; i++ )
	{
		add_node(head_BTout, buffer[i]);
	}
	UCSR0B |= (1<<UDRIE0);	//Enable UDRE interrupt flag -> send when empty dataregister
	// maby add while UDRIE0 = 0 here to counter multiple send_BT_buffer in a row
}


// Receive complete - triggered by interrupt
ISR(USART0_RX_vect) 
{
	//UCSR0B &= ~(1<<UDRIE0);
	//uint8_t data = (uint8_t)UDR0;
	char data = UDR0;
	if (data == '1')
	{
		send_BT_buffer(testbuffer1_);
		
	}
	else if (data == '2')
	{
		send_BT_buffer(testbuffer2_);
	}
	else if (data == '3')
	{
		send_BT_buffer(testbuffer3_);
	}
	else{
		UDR0 = data;
	}
	//add_node(head_BTin, data); //Saved received data in list 
	//UDR0 = data;
	//BT_send(data); //echo **** CHANGE WHEN NOT TESTING ****
	
	//BT_received_flag = 0;
	//char data = UDR0; //Get received value
	//Write_Buffer(inBT, data, writePos_BTin); //Writes data to buffer in order they are received
	
}


// Empty dataregister = send next character
ISR(USART0_UDRE_vect)
{
	if (head_BTout == NULL) // Nothing more to send, disable interrupt ******* REMEMBER TO ACTIVATE UDRIE0 WHEN TRANSMITTING (auto in BT_send)
	{
		UCSR0B &= ~(1<<UDRIE0);
	}
	else
	{
		UDR0 = (int8_t)pop_node(&head_BTout); 
	}
	
	
	//if ((*readPos_BTout) == BuffSize) //Read entire buff and sent it
	//{
		//(*readPos_BTout) = 0;
		//BT_sent_flag = 1; // done with transmission
		//UCSR0B &= ~(1<<UDRIE0); //Disable UDRE interrupt, All data is sent. 
	//}
	//else
	//{
		//BT_sent_flag = 0;
		//UDR0 = Read_Buffer(outBT, readPos_BTout); //Send back the next value in out-buffer. 														
	//}
		
}

void send_SPI_buffer(char *buffer)
{
	
	memset(outSPDR, '\0', BuffSize);
	strncpy(outSPDR, buffer, BuffSize); //Copy what to send into outSPDR
	(*posBuff_SPIout) = 0; // start reading from beginning
	ongoing_SPI_transfer = 1; //something to send.
	while(((ongoing_SPI_transfer == 1) & !(outSPDR[(*posBuff_SPIout)] == '\0'))); //Wait until entire buffer is sent.
	
}



void SPI_send(int tosend)
{
	add_node(head_SPIout, tosend); //Add node with tosend-value to desired list
}

void SPI_send_arr(int8_t tosend[], int size) // lenght of array = sizeof(array)/sizeof(element in array)
{
	int i = 0;
	while(i < size)// +2 due to continu
	{
		SPI_send(tosend[i]);
		i++;
	}
}

// Interrupt method runs when SPI transmission/reception is completed.
ISR(SPI_STC_vect)
{
	int data = (int)SPDR;
	add_node(head_SPIin, data); // Add received data to in-queue
	if (head_SPIout == NULL)
	{
		int8_t stop_bit = -128; //0b10000000, cant be shown on lcd as -128 due to limits in print func.
		SPDR = stop_bit;
	}
	else
	{
		SPDR = (int8_t)pop_node(&head_SPIout);
	}
}


int main(void)
{

	head_SPIout = (buffer_ *)malloc(sizeof(buffer_)); //Define head of list for SPI- values to send and alloc memory.
	head_SPIout->next= NULL;
	head_SPIout->val = 0;
	

	head_SPIin = (buffer_ *)malloc(sizeof(buffer_)); //Define head of list for SPI- values to receive and alloc memory.
	head_SPIin->next= NULL;
	head_SPIin->val = 0;
	
	head_BTin = (buffer_ *)malloc(sizeof(buffer_));
	head_BTin->next= NULL;
	head_BTin->val = 0;
	head_BTout = (buffer_ *)malloc(sizeof(buffer_));
	head_BTout->next= NULL;
	head_BTout->val = 0;
	
	
	sleep_enable();
	Komm_InitPortDirections();
	Komm_InitPortValues();
	SPI_SlaveInit();
	BT_init();
	sei();
	//flush_list(&head_SPIout);
	//flush_list(&head_SPIin);
	int8_t array[] ={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};
	SPI_send_arr(array, (sizeof(array)/sizeof(array[0]))); // sizeof(array)/sizeof(element in array) = lenght of array
	while(1)
	{
		flush_list(&head_SPIin);
	}
}

/* Todo:
		- Göra så att läsning sker tills buffern är null. Någon form av stoppbit i buffer, måhända skriva över nyligen läst med \0.
		- Avsluta alla SPI-buffrar med något bra tecken.
*/