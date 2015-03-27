/*
 * Bluetooth_.c
 *
 * Created: 2015-03-26 13:29:42
 *  Author: Måns
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

unsigned char outSPDR;
unsigned char inSPDR;
unsigned char outBT;
unsigned char inBT;
unsigned char BTbuffer_[128];
int bufferPos = 0;

//Setup data direction registers @ ports for out/inputs.
void Komm_InitPortDirections(void)
{
	DDRB = 1<<DDB6;
	DDRD = 1<<DDD1;
}

//Initiate port values.
void Komm_InitPortValues(void)
{
	PORTB = 1<<PORTB4;
}

//Configures device as spi slave.
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


//Bus transmission. Send and receive data from styrmodulen.
void SPI_SlaveTransmit(unsigned char cData)
{
	inSPDR = SPDR;
	//Load data into SPI data register
	SPDR = cData;
}

//Interrupt method runs when SPI transmission/reception is completed.
ISR(SPI_STC_vect)
{
	inSPDR = SPDR;
	SPDR = inSPDR -1;
}


// Set up and enable Bluetooth
void BT_init(void)
{
	UBRR0H = 0x00; //correct value to change baud rate
	UBRR0L = 0x07;//^^ same ^^ with a 14.7 mhz, scale with 1111 (7)
	
	UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (0<<UCSZ02) | (1<<RXCIE0) | (1<<TXCIE0);
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

// Receive data via BT.
unsigned char BT_receive(void)
{

	return UDR0;
}

// Transmit data via BT.
void BT_transmit(char data)
{
	while(!( UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	SPDR = data;
	inSPDR = data;
}

// Receive complete
ISR(USART0_RX_vect) 
{
	inBT = UDR0;
	//inBT = BT_receive();
	BT_transmit(inBT); // Send back incoming
}

// Transmission complete
ISR(USART0_TX_vect) 
{

}


int main(void)
{
	outSPDR = 0x01;
	inSPDR = 0x00;
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