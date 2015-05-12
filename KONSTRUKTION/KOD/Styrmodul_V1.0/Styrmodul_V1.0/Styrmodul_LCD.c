/*
 * LCD
 *
 * Created: 4/22/2015 10:49:11 PM
 *  Author: frefr166
 */ 

#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>


int LCD_Counter;

int LCD_Busy()
{
	DDRA = 0; // Set PORTA as input
	PORTB |= 1 << 1; // Read busy flag
	PORTB &= ~(1 << 0); // Clear bit 0 in PORTB to 0 (Set register select to instruction "RS=0")
	PORTB |= (1 << 2); // Activates the LCD (enable pin on LCD)
	_delay_us(2);
	char instr = PINA;
	PORTB &= ~(1 << 2); // Clear bit 2 in PORTB
	PORTB &= ~(1 << 1); // Clear bit 1 in PORTB
	DDRA = 0xff; // Set PORTA as output
	return instr >> 7; // MSB is the busy flag
}

void LCD_SendCommand(char cmd) {
	while (LCD_Busy())
	{
		_delay_ms(1);
	}
	
	PORTB &= ~(1 << 0); // Clear bit 0 in PORTB to 0 (Set Register Select to instruction "RS=0")
	PORTA = cmd;
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

// Sets row on the LCD. Do not use this yourself, rather use LCD_SetPosition instead.
void LCD_SetRow(int row)
{
	while(LCD_Busy())
	{
		_delay_ms(1);
	}
	
	PORTB &= ~(1 << 0); // Clear RS and
	PORTB &= ~(1 << 1); // clear R/W bits so that the following commands can be run
	
	if (row == 1)
	{
		LCD_SendCommand(0b10000000); // Set the cursor on the first row, first char
	}
	else if (row == 2)
	{
		LCD_SendCommand(0b11000000); // Set the cursor on the second row, first char
	}
}

// Sets position for cursor on LCD. Argument should be a number in the range of 0-31.
void LCD_SetPosition(uint8_t pos)
{
	LCD_Counter =(int) pos - 1;
	while(LCD_Busy())
	{
		_delay_ms(1);
	}
	PORTB &= ~(1 << 0); // Clear RS and
	PORTB &= ~(1 << 1); // clear R/W bits so that the following commands can be run
	
	if (pos < 16)
	{
		LCD_SendCommand(128+pos);
	}
	else if (pos < 32)
	{
		LCD_SendCommand(128+64-16+pos);
	}
	else LCD_SendCommand(0b10000000);
	
}

void LCD_SendCharacter(char symbol)
{
	LCD_Counter++;
	
	if(LCD_Counter == 16)
	{
		LCD_SetRow(2);
	}
	else if(LCD_Counter == 32)
	{
		LCD_Counter = 0;
		LCD_SetRow(1);
	}
	
	while(LCD_Busy())
	{
		_delay_ms(1);
	}
	PORTB |= (1 << 0); // Set RS
	PORTB &= ~(1 << 1); // Clear R/W
	
	PORTA = (int)symbol;
	
	PORTB |= 1 << 2; // Set Enable
	_delay_us(50); // 50us is the controller execution time of the LCD.
	PORTB &= ~(1 << 2); // Pull Enable.
}

void LCD_SendString(char *text)
{
	while(*text)
	{
		LCD_SendCharacter(*text++);
	}
}

void LCD_WelcomeScreen(void)
{
	LCD_SendString("     ResQ.Pl    ");
	LCD_SetRow(2);
	LCD_SendString("   Master Race  ");
}

// Display an unsigned 8 bit value as up to three decimal characters
void LCD_display_uint8(uint8_t val) {
	unsigned char buf[3];
	int8_t ptr;
	for(ptr=0;ptr<3;++ptr) {
		buf[ptr] = (val % 10) + '0';
		val /= 10;
	}
	for(ptr=2;ptr>0;--ptr) {
		if (buf[ptr] != '0') break;
	}
	for(;ptr>=0;--ptr) {
		LCD_SendCharacter(buf[ptr]);
	}
}

 //Display a signed 8 bit value as a possible minus character followed by up to three decimal characters
void LCD_display_int8(int8_t val) {
	 unsigned char buf[3];
	 int8_t ptr;
	 if (val < 0) {
		 LCD_SendCharacter('-');
		 val *= -1;
	 }
	 for(ptr=0;ptr<3;++ptr) {
		 buf[ptr] = (val % 10) + '0';
		 val /= 10;
	 }
	 for(ptr=2;ptr>0;--ptr) {
		 if (buf[ptr] != '0') break;
	 }
	 for(;ptr>=0;--ptr) {
		 LCD_SendCharacter(buf[ptr]);
	 }
 }

void LCD_display_int16(int16_t val) {
	unsigned char buf[5];
	if (val < 0) {
	  LCD_SendCharacter('-');
	  val *= -1;
	}
	int8_t ptr;
	for(ptr=0;ptr<5;++ptr) {
		buf[ptr] = (val % 10) + '0';
		val /= 10;
	}
	for(ptr=4;ptr>0;--ptr) {
		if (buf[ptr] != '0') break;
	}
	for(;ptr>=0;--ptr) {
		LCD_SendCharacter(buf[ptr]);
	}
}

// Display an unsigned 16 bit value as up to five decimal characters
void LCD_display_uint16(uint16_t val) {
	unsigned char buf[5];
	int8_t ptr;
	for(ptr=0;ptr<5;++ptr) {
		buf[ptr] = (val % 10) + '0';
		val /= 10;
	}
	for(ptr=4;ptr>0;--ptr) {
		if (buf[ptr] != '0') break;
	}
	for(;ptr>=0;--ptr) {
		LCD_SendCharacter(buf[ptr]);
	}
}

void LCD_Clear()
{
	LCD_SendCommand(0b00000001);
	_delay_ms(1.53);
}

// Initiatazion of the LCD, according to Initializing Flowchart(Condition fosc=270KHz) in the data sheet.
void LCD_Init()
{
	LCD_Counter = 0;
	_delay_ms(30);
	// Configure the LCD for 8 bits, 2 lines, 5x8 pixlex (dots) send instruction 00 0011 1000
	LCD_SendCommand(0b00111000);
	_delay_us(39);

	// Display, cursor and blinking off, instruction 00 0000 1000
	LCD_SendCommand(0b00001000);
	_delay_us(39);

	// Clear display, instruction 00 0000 0001
	LCD_Clear();

	// Cursor moving direction: left-to-right, do not shift he display (shift disabled), instruction 00 0000 0110
	LCD_SendCommand(0b00000110);

	// Display on, cursor ON, blinking on, instruction 00 0000 1110
	LCD_SendCommand(0b00001110);
}
