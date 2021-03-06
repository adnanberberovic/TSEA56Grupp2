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
//#include <avr/pgmspace.h>
//#define F_CPU 20000000UL

// Setup data direction registers @ ports for out/inputs.
void Styr_InitPortDirections(void)
{
	DDRA = 1<<DDA0 | 1<<DDA1 | 1<<DDA2 | 1<<DDA3 | 1<<DDA6;
	DDRB = 1<<DDB0 | 1<<DDB1 | 1<<DDB2 | 1<<DDB3 | 1<<DDB4 | 1<<DDB5 | 1<<DDB7;
	DDRC = 1<<DDC0;
	DDRD = 1<<DDD0 | 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4 | 1<<DDD5 | 1<<DDD6 | 1<<DDD7;
} 

// Setups port values, more specifically puts SS on high.
void Styr_InitPortValues(void)
{
	PORTB = 1<<PORTB3 | 1<<PORTB4;
}

// Configures device as spi master.
void SPI_MasterInit(void)
{
	SPSR = 0<<SPI2X;
	SPCR = 1<<SPIE | 1<<SPE | 1<<DORD | 1<<MSTR | 0<<CPOL | 0<<CPHA | 1<<SPR1 | 1<<SPR0;
	// SPIE: SPI interrupt enable. Set to 1 to allow interrupts
	// SPE: SPI Enable. Set to 1 to allow SPI communication
	// DORD: Data order. Set to 1 to transmit LSB first, MSB last.
	// MSTR: Master select. Set to 1 to set master.
	// CPOL and CPHA: set to 0 to sample on rising edge and setup on falling edge.
	// SPI2X, SPR1, SPR0, set to 0,1,1 to scale clock with f_osc/128.
	// Bus config similar to comm and sensor module, though set 0<<MSTR
	
}

// Initiates communication with other modules.
unsigned char SPI_MasterTransmit(unsigned char cData, char target)
{
	if (target == 'k') // K as in kommunikation
	{
		PORTB = 0<<PORTB4;
	}
	else if (target == 's')	// S as in sensor
	{
		PORTB = 0<<PORTB5;
	}
	// Load data into SPI data register.
	SPDR = cData; 
	
	// Wait until transmission completes.
	while(!(SPSR & (1<<SPIF)));
	
	PORTB = 1<<PORTB4 | 1<<PORTB5;
	
	return SPDR;
}

ISR(SPI_STC_vect)
{
	
}

int LCD_Busy()
{
	DDRD = 0; // Set PORTD as input
	PORTB |= 1 << 1; // Read busy flag
	PORTB &= ~(1 << 0); // Clear bit 0 in PORTB to 0 (Set register select to instruction "RS=0")
	PORTB |= (1 << 2); // Activates the LCD (enable pin on LCD)
	_delay_us(2);
	char instr = PIND;
	PORTB &= ~(1 << 2); // Clear bit 2 in PORTB
	PORTB &= ~(1 << 1); // Clear bit 1 in PORTB
	DDRD = 0xff; // Set PORTD as output
	return instr >> 7; // MSB is the busy flag
}

void LCD_SendCommand(char cmd) {
	while (LCD_Busy())
	{
		_delay_ms(1);
	}
	
	PORTB &= ~(1 << 0); // Clear bit 0 in PORTB to 0 (Set Register Select to instruction "RS=0")
	PORTD = cmd;
	PORTB |= 1 << 2; // Set R/W to 1.
	
	if (cmd == 0b01 || cmd == 0b10) // If clear or return home instruction
	{
		_delay_ms(1.5);				    // Then execution time is longer
	}
	else
	{
		_delay_us(50);
	}

	PORTB &= ~(1 << 2);
}

void LCD_SetRow(int row)
{
	while(LCD_Busy())
	{
		_delay_ms(1);
	}
	
	PORTB |= (0 << 0)|(0 << 1); // Set RS and R/W to 0 so that the following commands can be run
	
	if (row == 1)
	{
		LCD_SendCommand(0b00000010); // Set the cursor on the first of rows
	}
	else if (row == 2)
	{
		LCD_SendCommand(0b11000000); // Set the cursor on the second of rows
	}
}

void LCD_SendCharacter(char symbol)
{
	while(LCD_Busy())
	{
		_delay_ms(1);
	}
	PORTB |= (1 << 0)|(0 << 1); // Set RS and clear R/W
	
	//uint8_t tempNum = (int)symbol;
	//PORTD = tempNum;
	
	// If the following doesn't work, delete it and uncomment the two lines above.
	PORTD = (int)symbol;
	
	PORTB |= 1 << 2; // Set Enable
	_delay_us(50); // 50us is the controller execution time of the LCD.
	PORTB &= ~(1 << 2); // Pull Enable.
}

void LCD_WelcomeScreen(void)
{
	//Rad 1
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter('R');
	LCD_SendCharacter('e');
	LCD_SendCharacter('s');
	LCD_SendCharacter('Q');
	LCD_SendCharacter('.');
	LCD_SendCharacter('P');
	LCD_SendCharacter('L');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	
	LCD_SetRow(2); //byt rad
	
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter('M');
	LCD_SendCharacter('a');
	LCD_SendCharacter('s');
	LCD_SendCharacter('t');
	LCD_SendCharacter('e');
	LCD_SendCharacter('r');
	LCD_SendCharacter(' ');
	LCD_SendCharacter('R');
	LCD_SendCharacter('a');
	LCD_SendCharacter('c');
	LCD_SendCharacter('e');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
	LCD_SendCharacter(' ');
}

// Initiatazion of the LCD, according to Initializing Flowchart(Condition fosc=270KHz) in the data sheet.
void LCD_Init()
{
	_delay_ms(30);
	// Configure the LCD for 8 bits, 2 lines, 5x8 pixlex (dots) send instruction 00 0011 1000
	LCD_SendCommand(0b00111000);
	_delay_us(39);

	// Display, cursor and blinking off, instruction 00 0000 1000
	LCD_SendCommand(0b00001000);
	_delay_us(39);

	// Clear display, instruction 00 0000 0001
	LCD_SendCommand(0b00000001);
	_delay_ms(1.53);

	// Cursor moving direction: left-to-right, do not shift he display (shift disabled), instruction 00 0000 0110
	LCD_SendCommand(0b00000110);

	// Display on, cursor ON, blinking on, instruction 00 0000 1110
	LCD_SendCommand(0b00001100);
}

int main(void)
{
	//unsigned char SPDRrec_ = 0;
	sei();	// Enable global interrupts
	sleep_enable();	// Enable sleep instruction
	Styr_InitPortDirections();	// Initiate Port directions for the styrmodul.
	Styr_InitPortValues();	// Initiate Port Values for the styrmodul.
	SPI_MasterInit();	// Initiate the styrmodul as the SPI master.
	LCD_Init(); // Initiate the LCD.
	LCD_WelcomeScreen(); // Welcomes the user with a nice message ;-)
	
	
	
	while(1)
    {		
	}
}