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
#include <util/delay.h>
#include <math.h>
#include <avr/pgmspace.h>
//#define F_CPU 20000000UL
//Setup data direction registers @ ports for out/inputs.
void Styr_InitPortDirections(void)
{
	DDRA = 1<<DDA0 | 1<<DDA1 | 1<<DDA2 | 1<<DDA3 | 1<<DDA6;
	DDRB = 1<<DDB0 | 1<<DDB1 | 1<<DDB2 | 1<<DDB3 | 1<<DDB4 | 1<<DDB5 | 1<<DDB7;
	DDRC = 1<<DDC0;
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
unsigned char SPI_MasterTransmit(unsigned char cData, char target)
{
	if (target == 'k') //k as in kommunikation
	{
		PORTB = 0<<PORTB4;
	}
	else if (target == 's')	//s as in sensor
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

ISR(SPI_STC_vect)
{
	
}

/// en del funktioner kan deklareras i en enskild *.h fil ///
// void LCD_busy();
// void LCD_init();
// void LCD_SendCommand(char cmd);
//

int LCD_busy()
{
	DDRD = 0; //sätter port D till ingång
	PORTB = 1 << 1; //read busy flag
	PORTB &= ~(1 << 0); // clear bit 0 in PORTB to 0 (sätter register select instruction "RS=0")
	PORTB = (1 << 2); // aktiverar LCD:en (enable pinne på LCD)
	_delay_us(2);
	char instr = PIND;
	PORTB &= ~(1 << 2); // clear bit 2 in PORTB to 0
	PORTB &= ~(1 << 1); // clear bit 1 in PORTB to 0
	DDRD = 0xff; //sätter port D till utgång
	return instr >> 7; // MSB is the busy flag
}

void LCD_SendCommand(char cmd) {
	while (LCD_busy())
	{
		_delay_ms(1);
		
	}
		PORTB &= ~(1 << 0); // clear bit 0 in PORTB to 0 (sätter Register Select till instruction "RS=0")
		PORTD = cmd;
	
	
	PORTB = 1 << 2;
	
	if (cmd == 0b01 || cmd == 0b10) // if clear or return home instruction
	{
		_delay_ms(1.5);				    // then execution time is longer
	}
	else
	{
		_delay_us(50);
	}

	PORTB &= ~(1 << 2);
}

//Sends a number to the LCD display
void LCD_SendNumber(uint8_t number)
{
	PORTD = number | 0b00110000 // compare to addition with 30_hex, 48_dec.
}

//Sends a capital letter to the LCD display
void LCD_SendCapitalLetter(uint8_t number)
{
	PORTD = number | 0b01000000 // number is a number between 1 and 26, for each letter in the alphabet in order.
}

//Sends a lower case letter to the LCD display.
void LCD_SendSmallLetter(uint8_t number)
{
	PORTD = number | 0b0110000 // number is a numbner between 1 and 26, for each letter in the alphabet in order.
}
//Sends a letter or symbol to the LCD display
void LCD_SendChar(char symbol)
{	
	// // Numbers
	// if(tecken == '0') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//00110000
	// else if (tecken == '1') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//00110001
	// else if (tecken == '2') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//00110010
	// else if (tecken == '3') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//00110011
	// else if (tecken == '4') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//00110100
	// else if (tecken == '5') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//00110101
	// else if (tecken == '6') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//00110110
	// else if (tecken == '7') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//00110111
	// else if (tecken == '8') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//00111000
	// else if (tecken == '9') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//00111001
	// // Capital letters
	// else if (symbol == 'A') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01000001
	// else if (symbol == 'B') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01000010
	// else if (symbol == 'C') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01000011
	// else if (symbol == 'D') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01000100
	// else if (symbol == 'E') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01000101
	// else if (symbol == 'F') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01000110
	// else if (symbol == 'G') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01000111
	// else if (symbol == 'H') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01001000
	// else if (symbol == 'I') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01001001
	// else if (symbol == 'J') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01001010
	// else if (symbol == 'K') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01001011
	// else if (symbol == 'L') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01001100
	// else if (symbol == 'M') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01001101
	// else if (symbol == 'N') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01001110
	// else if (symbol == 'O') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01001111
	// else if (symbol == 'P') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01010000
	// else if (symbol == 'Q') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01010001
	// else if (symbol == 'R') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01010010
	// else if (symbol == 'S') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01010011
	// else if (symbol == 'T') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01010100
	// else if (symbol == 'U') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01010101
	// else if (symbol == 'V') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01010110
	// else if (symbol == 'W') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01010111
	// else if (symbol == 'X') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01011000
	// else if (symbol == 'Y') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01011001
	// else if (symbol == 'Z') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01011010
	// // lower case letters
	// else if (symbol == 'a') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01100001
	// else if (symbol == 'b') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01100010
	// else if (symbol == 'c') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01100011
	// else if (symbol == 'd') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01100100
	// else if (symbol == 'e') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01100101
	// else if (symbol == 'f') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01100110
	// else if (symbol == 'g') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01100111
	// else if (symbol == 'h') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01101000
	// else if (symbol == 'i') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01101001
	// else if (symbol == 'j') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01101010
	// else if (symbol == 'k') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01101011
	// else if (symbol == 'l') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01101100
	// else if (symbol == 'm') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01101101
	// else if (symbol == 'n') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01101110
	// else if (symbol == 'o') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01101111
	// else if (symbol == 'p') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01110000
	// else if (symbol == 'q') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01110001
	// else if (symbol == 'r') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01110010
	// else if (symbol == 's') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01110011
	// else if (symbol == 't') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01110100
	// else if (symbol == 'u') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01110101
	// else if (symbol == 'v') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01110110
	// else if (symbol == 'w') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01110111
	// else if (symbol == 'x') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01111000
	// else if (symbol == 'y')  PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01111001
	// else if (symbol == 'z') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01111010
	// Other symbols
	//else if (symbol == '/n') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//00100000
	if (symbol == '!') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//00100001
	else if (symbol == '"') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//00100010
	else if (symbol == '#') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//00100011
	else if (symbol == '$') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//00100100
	else if (symbol == '%') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//00100101
	else if (symbol == '&') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//00100110
	//else if (symbol == ''') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//00100111
	else if (symbol == '(') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//00101000
	else if (symbol == ')') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//00101001
	else if (symbol == '*') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//00101010
	else if (symbol == '+') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//00101011
	else if (symbol == ',') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//00101100
	else if (symbol == '-') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//00101101
	else if (symbol == '.') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//00101110
	else if (symbol == '/') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//00101111
	else if (symbol == ':') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//00111010
	else if (symbol == ';') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//00111011
	else if (symbol == '<') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//00111100
	else if (symbol == '=') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//00111101
	else if (symbol == '>') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//00111110
	else if (symbol == '?') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//00111111
	else if (symbol == '{') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01111011
	else if (symbol == '|') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01111100
	else if (symbol == '}') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01111101
	//else if (symbol == '->') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01111110
	//else if (symbol == '<-') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01111111
	else if (symbol == '^') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01011110
	else if (symbol == '_') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01011111
	else PORTD = (1 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0); //full box
}


// initieringen av LCD kontrollern, enligt Initializing Flowchart(Condition fosc=270KHz) i databladet
void LCD_init()
{
	_delay_ms(30);
	// konfigurera LCD:en för 8 bitar, 2 linjer, 5x8 pixlar (dots) dvs LCD:en får instruktion 00 0011 1000
	LCD_SendCommand(0b00111000);
	_delay_us(39);

	// display,cursor and blinking off, instruktion 00 0000 1000
	LCD_SendCommand(0b00001000);
	_delay_us(39);

	// Clear display, instruktion 00 0000 0001
	LCD_SendCommand(0b00000001);
	_delay_ms(1.53);

	// cursor moving direction: left-to-right, skifta ej displayen (dvs shift disabled), instruktion 00 0000 0110
	LCD_SendCommand(0b00000110);

	// display on, cursor and blinking still off, instruktion 00 0000 1100
	LCD_SendCommand(0b00001100);
		
		
	PORTB = (1 << 0)|(0 << 1); // för att börja skicka data
}

int main(void)
{
	//unsigned char SPDRrec_ = 0;
	sei();
	sleep_enable();
	Styr_InitPortDirections();
	Styr_InitPortValues();
	SPI_MasterInit();
	LCD_init();
	//SPDRrec_ = SPI_MasterTransmit(0x01, 'k');
	//_delay_us(10);
	//SPDRrec_ = SPI_MasterTransmit(0x1F, 'k');
	//if (SPDRrec_ == 0x0F)
	//{
	//	PORTC = (1<<PORTC0);
	//}
	//if (SPDRrec_ == 0x00)
	//{
	//	PORTC = (1<<PORTC0);
	//}
	//_delay_us(10);
	//SPDRrec_ = SPI_MasterTransmit(0x0F, 'k');
	//if (SPDRrec_ == 0x1E)
	//{
		//PORTC = (1<<PORTC0);
	//}
	//_delay_us(10);
	//SPDRrec_ = SPI_MasterTransmit(0x1F, 'k');
	//if (SPDRrec_ == 0x0E)
	//{
		//PORTC = (1<<PORTC0);
	//}
	//if (SPDRrec_ == 0x0F)
	//{
		//PORTC = (1<<PORTC0);
	//}
	LCD_SendChar('S');
	LCD_SendChar('T');
	LCD_SendChar('D');
	
	while(1)
    {		
	}
}