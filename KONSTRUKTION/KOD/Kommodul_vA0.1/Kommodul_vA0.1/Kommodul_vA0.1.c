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


uint8_t arrSpeed[5] = {0,0,1,1,0}; //Array with current speed (Left right), direction (1 = forward, 0 = backward) and claw. From/to PC/master left
uint8_t arrSpeedout[3] = {10, 10, 1}; //Speed left, speed right, dirleft & right
uint8_t arrMap[3] = {0, 0, 0}; //[0] = {xpos(4) + dir(2)}, [1] = {y-pos(5) + mål(1)}, [2] = {left(2) + ahead(2) + right(2)},

uint8_t sendFlag = 0;
uint8_t Flag_ = 0;
int8_t arrSensor[7] = {8,7,6,5,4,3,2};



uint8_t mapFlag_ = 0;	
uint8_t sensorFlag_ = 0;
uint8_t speedFlag_ = 0;
uint8_t speedoutFlag_ = 0;
uint8_t counter_ = 0;

uint8_t	BTmapFlag_ = 0;
uint8_t BTsensorFlag_ = 0;
uint8_t BTspeedFlag_ = 0;
uint8_t BTspeedoutFlag_ = 0;
uint8_t BTcounter_ = 0;
//*********** BUFFER STRUCT ****************

struct node { // definition of the linked list node
	uint8_t val;
	struct node *next;
	};

typedef struct node buffer_; // buffer_ definieras

buffer_* head_SPIout = NULL;
buffer_* head_SPIin = NULL;

buffer_* head_BTout = NULL;
buffer_* head_BTin = NULL;
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
	
	UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (0<<UCSZ02) | (1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0);
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
void add_node(buffer_** lst_head, uint8_t val)
{
	if (*lst_head == 0)
	{

		(*lst_head) = (buffer_*)malloc(sizeof(buffer_));
		(*lst_head)->next = 0;
		(*lst_head)->val = val;
	}
	
	//UDR0 = val;

	buffer_ * curr = *lst_head;
	while (curr->next != 0){ // step to end of list
		curr = curr->next;
	}
	curr->next = (buffer_ *)malloc(sizeof(buffer_));
	curr->val = val;
	curr->next->next = 0; // Add node last.

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
uint8_t pop_node(buffer_ ** lst_head)
{
	buffer_* next_node = NULL;
	uint8_t retval = 3;
	
	if ( *lst_head != NULL)
	{
		next_node = (*lst_head)->next;
		retval = (*lst_head)->val;
		free(*lst_head);
		*lst_head = next_node;
		return retval;
	}

	return retval;
	
}
void BT_send(uint8_t val)
{
	add_node(&head_BTout, val);
	UCSR0B |= (1<<UDRIE0); // activate interrupt
}
void flush_list(buffer_ ** lst_head)
{
	if (*lst_head == NULL)
	{
		return;
	}
	
	buffer_* curr_node = *lst_head;
	*lst_head = NULL;
	
	if (curr_node->next != NULL)
	{
		flush_list(&(curr_node->next));
	}
	free(curr_node);
}
void send_BT_buffer(uint8_t buffer[], int size)
{
	flush_list(&head_BTout);
	int i = 0;
	while(i < size)
	{
		add_node(&head_BTout, buffer[i]);
		i++;
	}

	UCSR0B |= (1<<UDRIE0);	//Enable UDRE interrupt flag -> send when empty dataregister
	
	// maby add while UDRIE0 = 0 here to counter multiple send_BT_buffer in a row
}
void BT_StartBitCheck(uint8_t in_)
{
	switch (in_){
		
		case 1:
		BTspeedFlag_ = 1;
		BTsensorFlag_ = 0;
		BTspeedoutFlag_ = 0;
		BTcounter_ = 0;
		BTmapFlag_ = 0;
		break;
		
		case 87:
		BTspeedoutFlag_ = 0;
		BTsensorFlag_ = 1;
		BTspeedFlag_ = 0;
		BTmapFlag_ = 0;
		break;
		
		case 89:
		BTspeedoutFlag_ = 1;
		BTsensorFlag_ = 0;
		BTspeedFlag_ = 0;
		BTmapFlag_ = 0;
		
		case 69:
		BTmapFlag_ = 1;
		BTcounter_ = 0;
		BTspeedoutFlag_ = 0;
		BTsensorFlag_ = 0;
		BTspeedFlag_ = 0;
		
	}
}
// Receive complete - triggered by interrupt
ISR(USART0_RX_vect) 
{
	uint8_t data = UDR0;
	if ((BTspeedFlag_ == 0) && (BTsensorFlag_ == 0) && (BTspeedoutFlag_ == 0)) //First time check if it's starbit
	{
		BT_StartBitCheck(data);
	}
	
	else if ((BTspeedFlag_ == 1) && (data != 87)) // Speed incoming
	{
		arrSpeed[BTcounter_] = data; //Load value into speed array
		BTcounter_++;
		if (BTcounter_ == (sizeof(arrSpeed)/sizeof(arrSpeed[0]))) // all values red
		{
			BTspeedFlag_ = 0;
			BTcounter_ = 0;
		}
	}
	if (BTsensorFlag_ == 1) //Send out sensordata
	{
		send_BT_buffer(arrSensor, (sizeof(arrSensor)/sizeof(arrSensor[0])));
		BTsensorFlag_ = 0;
	}
	if(BTspeedoutFlag_ == 1)
	{
		send_BT_buffer(arrSpeedout, (sizeof(arrSpeedout)/sizeof(arrSpeedout[0])));
		BTspeedoutFlag_ = 0;
	}
	if (BTmapFlag_ == 1)
	{
		send_BT_buffer(arrMap, (sizeof(arrMap)/sizeof(arrMap[0])));
		BTmapFlag_ = 0;
	}
	
	
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
		UDR0 = pop_node(&head_BTout); 
	}
}

void send_SPI_buffer(char *buffer)
{
	memset(outSPDR, '\0', BuffSize);
	strncpy(outSPDR, buffer, BuffSize); //Copy what to send into outSPDR
	(*posBuff_SPIout) = 0; // start reading from beginning
	ongoing_SPI_transfer = 1; //something to send.
	while(((ongoing_SPI_transfer == 1) && !(outSPDR[(*posBuff_SPIout)] == '\0'))); //Wait until entire buffer is sent.
	
}
void SPI_send(uint8_t tosend)
{
	add_node(&head_SPIout, tosend); //Add node with tosend-value to desired list
}
void SPI_send_arr(uint8_t tosend[], int size) // lenght of array = sizeof(array)/sizeof(element in array)
{
	int i = 0;
	while(i < size)
	{
		SPI_send(tosend[i]);
		i++;
	}
}
void SPI_StartBitCheck(uint8_t in_)
{
	switch (in_){
		
		case 1: 
		speedFlag_ = 1;
		speedoutFlag_ = 0;
		sensorFlag_ = 0;
		mapFlag_ = 0;
		counter_ = 0;
		break;
		
		case 255: 
		sensorFlag_ = 1;
		speedoutFlag_ = 0;
		speedFlag_ = 0;
		mapFlag_ = 0;		
		counter_ = 0;
		break; 
		
		case 254:
		speedoutFlag_ = 1;
		sensorFlag_ = 0;
		speedFlag_ = 0;
		mapFlag_ = 0;		
		counter_ = 0;
		break;
		
		case 69:
		mapFlag_ = 1;
		speedoutFlag_ = 0;
		sensorFlag_ = 0;
		speedFlag_ = 0;
		counter_ = 0;
		break;
		
		default: 
		mapFlag_ = 0;
		speedoutFlag_ = 0;
		sensorFlag_ = 0;
		speedFlag_ = 0;
		counter_ = 0;
	}
}
// Interrupt method runs when SPI transmission/reception is completed.
ISR(SPI_STC_vect)
{
	uint8_t data = SPDR;
	
	if ( (speedFlag_ == 0) && (sensorFlag_ == 0) && (speedoutFlag_ == 0)){
		SPI_StartBitCheck(data);
	}
	
	else if (sensorFlag_ == 1){
	arrSensor[counter_] = data; //Load into correct pos of array 0-6
	counter_++;
		if (counter_ == (sizeof(arrSensor)/sizeof(arrSensor[0])) ){  //all values in.
				counter_ = 0;
				sensorFlag_ = 0;
		}
	}
	else if (speedoutFlag_ == 1){
		arrSpeedout[counter_] = data; //Load into correct pos of array 0-3
		counter_++;
		if (counter_ == (sizeof(arrSpeedout)/sizeof(arrSpeedout[0])) ){  //all values in.
			counter_ = 0;
			speedoutFlag_ = 0;
		}
	}
	
	// Speed is to be sent.
	if (speedFlag_ == 1){
		SPI_send_arr(arrSpeed,(sizeof(arrSpeed)/sizeof(arrSpeed[0])));
		SPDR =  pop_node(&head_SPIout);
		speedFlag_ = 0;
		return;
	}
	

	if (head_SPIout == NULL){ //Sendback function will always be performed when something is to be sent.
		uint8_t stop_bit = 255;
		SPDR = stop_bit;
	}
	else{
		SPDR = pop_node(&head_SPIout);
	}
	
}



int main(void)
{	
	Flag_ = 0;
	sleep_enable();
	Komm_InitPortDirections();
	Komm_InitPortValues();
	SPI_SlaveInit();
	BT_init();
	sei();
	while(1)
	{
	}
}

/* Todo:
		- Göra så att läsning sker tills buffern är null. Någon form av stoppbit i buffer, måhända skriva över nyligen läst med \0.
		- Avsluta alla SPI-buffrar med något bra tecken.
*/