/*
 *  styrmodul_test1.c
 *	Styrmodul
 *  Author: adnbe196, mansk700, nikag669
 */ 

//#include <avr/pgmspace.h>
#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>



// GLOBAL VARIABLES

int LCD_Counter;

// Setup data direction registers @ ports for out/inputs.
void Styr_InitPortDirections(void)
{
	DDRA = 1<<DDA0 | 1<<DDA1 | 1<<DDA2 | 1<<DDA3 | 1<<DDA4 | 1<<DDA5 | 1<<DDA6 | 1<<DDA7;
	DDRB = 1<<DDB0 | 1<<DDB1 | 1<<DDB2 | 1<<DDB3 | 1<<DDB4 | 1<<DDB5 | 1<<DDB7;
	DDRC = 1<<DDC0;
	DDRD = 1<<DDD0 | 1<<DDD1 | 0<<DDD2 | 1<<DDD3 | 1<<DDD4 | 1<<DDD5 | 1<<DDD6 | 1<<DDD7;
} 

// Setups port values, more specifically puts SS on high.
void Styr_InitPortValues(void)
{
	PORTB |= 1<<PORTB3 | 1<<PORTB4;
	PORTC |= 1<<PORTC0;
}

// Configures device as spi master.
void SPI_MasterInit(void)
{
	SPSR = 0<<SPI2X;
	SPCR = 0<<SPIE | 1<<SPE | 1<<DORD | 1<<MSTR | 0<<CPOL | 0<<CPHA | 1<<SPR1 | 1<<SPR0;
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
		PORTB &= ~(1<<PORTB4);
	}
	else if (target == 's')	// S as in sensor
	{
		PORTB &= ~(1<<PORTB3);
	}
	else if (target == 'g') // G as in gyro
	{
		PORTC &= ~(1<<PORTC0);
// 		SPDR = cData; 
// 		while(!(SPSR & (1<<SPIF)));
// 		SPDR = cData; 
// 		while(!(SPSR & (1<<SPIF)));
		
	}
	// Load data into SPI data register.
	SPDR = cData; 
	
	// Wait until transmission completes.
	while(!(SPSR & (1<<SPIF)));
	
	PORTB |= 1<<PORTB3 | 1<<PORTB4;
	//PORTC |= 1<<PORTC0;
	
	return SPDR;
}

ISR(SPI_STC_vect)
{
	PORTB |= 1<<PORTB3 | 1<<PORTB4;
	PORTC |= 1<<PORTC0;
}


//----------------------------PWM------------------------------------

void PWM_Init(void)
{
	// TODO:::::sätt riktning på databitar
	
	// Configuration of the PWM setting for OC2A and OC2B
	// TCCn1:0 = 3 equals phase correct PWM mode
	// WGM22:0 = 5 equals phase correct PWM mode with compare from OCRn
	// CS22:0 = 3 sets the division factor to 32
	TCCR2A = 1<<COM2A1 | 1<<COM2A0 | 1<<COM2B1 | 1<<COM2B0 | 0<<WGM21 | 1<<WGM20;
	TCCR2B = 1<<WGM22 | 0<<CS22 | 1<<CS21 | 1<<CS20;
	
	// Set the compare values for the PWM, 0 == 0% and 
	OCR2A = 0;
	OCR2B = 0;
}

void PWM_SetSpeedRight(int speed)
{
	if (speed >= 0 && speed <= 255)
	{
		OCR2B = speed;
	}
}

void PWM_SetSpeedLeft(int speed)
{
	if (speed >= 0 && speed <= 255)
	{
		OCR2A = speed;
	}
}

void PWM_SetDirRight(int dir)
{
	if (dir == 0)
	{
		PORTD &= ~(1 << PORTD0);
	} 
	//&= ~(1 << PORTD0)
	else if (dir == 1)
	{
		PORTD |= 1 << PORTD0;
	}
}

void PWM_SetDirLeft(int dir)
{
	if (dir == 0)
	{
		PORTD &= ~(1 << PORTD1);
	}
	else if (dir == 1)
	{
		PORTD |= 1 << PORTD1;
	}
}

void PWM_Test(void)
{
	for (int i=0; i < 300; i++)
	{
		_delay_ms(250);
	}
	PWM_SetSpeedLeft(50);
	//PWM_SetSpeedRight(0);
	for (int i=0; i < 300; i++)
	{
		_delay_ms(250);
	}
	PWM_SetDirLeft(1);
	//PWM_SetDirRight(0);
	for (int i=0; i < 300; i++)
	{
		_delay_ms(250);
	}
	PWM_SetSpeedLeft(0);
	//PWM_SetSpeedRight(150);
	for (int i=0; i < 300; i++)
	{
		_delay_ms(250);
	}
	//PWM_SetDirRight(1);
	for (int i=0; i < 300; i++)
	{
		_delay_ms(250);
	}
}


//----------------------------PWM------------------------------------

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
	
	//uint8_t tempNum = (int)symbol;
	//PORTA = tempNum;
	
	// If the following doesn't work, delete it and uncomment the two lines above.
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
	LCD_SendString("    ResQ.Pl heh");
	LCD_SetRow(2);
	LCD_SendString("  Master Race  ");
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
	for(ptr=2;ptr>=0;--ptr) {
		LCD_SendCharacter(buf[ptr]);
	}
}

// Display a signed 8 bit value as a possible minus character followed by up to three decimal characters
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

//----------------------------Gyro------------------------------------
void activateGyroADC()
{
	int dataH;
	SPI_MasterTransmit(0b10010100,'g');
	// check if instruction received
	// by sending two dummy bytes:
	dataH = SPI_MasterTransmit(0x00,'g');
	_delay_us(200);
	SPI_MasterTransmit(0x00,'g');
	_delay_us(200);
	
	//LCD_SendCharacter(dataH); // displays "d" (0b 0100 0110)
				
 	if (dataH & 0b10000000) {
 		LCD_SendString("not accepted");
 	}
// 	else LCD_SendString("ADC activated");
}

void sleepGyroADC()
{
	unsigned int dataH;
	SPI_MasterTransmit(0b10010000,'g');
	// check if instruction received
	// by sending two dummy bytes:
	dataH = SPI_MasterTransmit(0x00,'g');
	_delay_us(200);
	SPI_MasterTransmit(0x00,'g');
	_delay_us(200);

	
// 	if (dataH & 0b10000000) {
// 		LCD_SendString("not accepted");
// 	}
// 	else LCD_SendString("sleep");
}

unsigned int getGyroADC() // gets the angular rate value (in mV) from ADC
{
	uint16_t result=0, dataH, dataL; // angular rate is written in two parts 8 bits each
	SPI_MasterTransmit(0b10010100,'g'); // send SPI ADCC for angular rate
	// check if instruction received
	// by sending two dummy bytes:
	dataH = SPI_MasterTransmit(0x00,'g');
	_delay_us(200);
	SPI_MasterTransmit(0x00,'g');
	_delay_us(200);
	
	
	if (dataH & 0b10000000) {
		LCD_SendString("not accepted");
	}
	
	SPI_MasterTransmit(0b10000000,'g'); // conversion start instruction (send SPI ADCR instr)
		
	dataH = SPI_MasterTransmit(0x00, 'g');
	_delay_ms(200);
	if (!(dataH & 0b00100000)) {
		LCD_SendString("EOC not set");
	}
// 	while(!(dataH & 0b00100000)) //Delay until EOC = 1;
// 	{
// 		dataH = SPI_MasterTransmit(0x00,'g');
// 	}
	dataL = SPI_MasterTransmit(0x00,'g');
	_delay_us(200);

	//PORTC |= 1<<PORTC0;
	LCD_SendString("H|L=");
	LCD_display_uint16(dataH); // "D" (0b 0100 0100)
	LCD_SendCharacter('|');
	LCD_display_uint16(dataL); // "beta" (0b 1110 0010)
	
	if(!(dataH & 0b10000000))
	{
		result = ((dataH & 0x0f) << 7) + (dataL >> 1);
	}
	return result;
}

// converts the adc reading to angles per second
int adcToAngularRate(unsigned int adcValue)
{
	int AngularRate = (adcValue * 25/12)+400;  // in mV
	// from the data sheet, R2 gyroscope sensor version is 26,67 mV/deg
	// for gyroscope with max angular rate 300 deg/s
	return (AngularRate - 2500)/26.67;
}

int main(void)
{
//	char SPDRrec_ = '0';
	sei();	// Enable global interrupts
	sleep_enable();	// Enable sleep instruction
	Styr_InitPortDirections();	// Initiate Port directions for the styrmodul.
	Styr_InitPortValues();	// Initiate Port Values for the styrmodul.
	SPI_MasterInit();	// Initiate the styrmodul as the SPI master.
	LCD_Init(); // Initiate the LCD.
//	PWM_Init(); // Initiate PWM for motör
//	LCD_WelcomeScreen(); // Welcomes the user with a nice message ;-)




	/* logik test */
 	uint8_t dataH, dataL, result;
 	SPI_MasterTransmit(0b10010100,'g'); // activation
 	_delay_us(50); // to separate packets
	dataH = SPI_MasterTransmit(0x00,'g'); // dummy
	_delay_us(50); // to separate packets
	dataL = SPI_MasterTransmit(0x00,'g'); // dummy
	_delay_us(200); // wait for EOC to be set / separate instructions
	PORTC |= 1<<PORTC0;
	
	while(1) {
 	SPI_MasterTransmit(0b10010100,'g'); // select angular rate channel
 	_delay_us(50); // to separate packets
	dataH = SPI_MasterTransmit(0x00,'g'); // dummy
	_delay_us(50); // to separate packets
	dataL = SPI_MasterTransmit(0x00,'g'); // dummy
	_delay_us(200); // to separate instructions
	PORTC |= 1<<PORTC0;

	do {
		SPI_MasterTransmit(0b10000000,'g'); // start conversion
		_delay_us(50); // to separate packets
		dataH = SPI_MasterTransmit(0x00, 'g'); // dummy
		_delay_us(50); // to separate packets
		dataL = SPI_MasterTransmit(0x00,'g'); // dummy
		_delay_us(200);
		PORTC |= 1<<PORTC0;
	} while(!(dataH & 0b00100000)); // read adc until EOC bit is set
	
	
// 	LCD_display_uint16(dataL);
// 	LCD_SendCharacter(' ');
// 	_delay_ms(500);
	}
// while(1) {
// 	
// 		LCD_Clear();
// 		LCD_SetPosition(0);
//   	dataH = dataH & 0x0f;
// 		dataH = dataH << 7;
// 		dataL = dataL >> 1;
// 		result = dataH | dataL;
// 		result = ((dataH & 0x0f) << 7) + (dataL >> 1);
// 		result = adcToAngularRate(result);
//  		LCD_display_uint16(dataH);
// 		 _delay_ms(500);
// 		 LCD_SendString(".......");
// 	
// 	SPI_MasterTransmit(0b10010100,'g'); // select angular rate channel
// 	_delay_us(50); // to separate packets
// 	dataH = SPI_MasterTransmit(0x00,'g'); // dummy
// 	_delay_us(50); // to separate packets
// 	dataL = SPI_MasterTransmit(0x00,'g'); // dummy
// 	_delay_us(200); // to separate instructions
// 
// 	SPI_MasterTransmit(0b10000000,'g'); // start conversion
// 	_delay_us(50); // to separate packets
// 	dataH = SPI_MasterTransmit(0x00, 'g'); // dummy
// 	_delay_us(50); // to separate packets
// 	dataL = SPI_MasterTransmit(0x00,'g'); // dummy
// 	_delay_ms(200);
// 	
// }
		
		
		
//  	uint16_t gyro_adc, angular_rate;
// 	
// 	activateGyroADC();
// 	_delay_ms(400);
// 	while(1)
//     {		
//  			LCD_Clear();
//  			LCD_SetPosition(0);
// 			
//  			gyro_adc = getGyroADC();
// 			LCD_display_uint8(gyro_adc);
// 			_delay_ms(700);
			
//  			angular_rate = adcToAngularRate(gyro_adc);
//  			LCD_SendCharacter(' ');
//  			LCD_display_uint8(angular_rate);
//  			LCD_SendString(" deg/s");
// 			
// 			_delay_ms(2000);
// 			LCD_Clear();
// 			LCD_SetPosition(0);
// 			
//  			sleepGyroADC();
// 			 _delay_ms(500);
//	}
}
