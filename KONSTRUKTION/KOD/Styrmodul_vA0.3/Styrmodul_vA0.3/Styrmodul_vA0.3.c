/*
 * pwmpls.c
 *
 * Created: 4/13/2015 12:39:10 PM
 *  Author: robop806
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

//#include <avr/pgmspace.h>

// GLOBAL VARIABLES

int LCD_Counter;
// One overflow corresponds to 13.1 ms if the F_CPU is 20 MHz and the defined prescaler.
int TIMER_overflows;
// The following overflow storage corresponds to 131ms.
int TIMER_overflows_deci;

// Setup data direction registers @ ports for out/inputs.
void Styr_InitPortDirections(void)
{
	DDRA = 1<<DDA0 | 1<<DDA1 | 1<<DDA2 | 1<<DDA3 | 1<<DDA4 | 1<<DDA5 | 1<<DDA6 | 1<<DDA7;
	DDRB = 1<<DDB0 | 1<<DDB1 | 1<<DDB2 | 1<<DDB3 | 1<<DDB4 | 1<<DDB5 | 1<<DDB7;
	DDRC = 1<<DDC0;
	DDRD = 1<<DDD0 | 1<<DDD1 | 0<<DDD2 | 1<<DDD3 | 1<<DDD4 | 1<<DDD5 | 1<<DDD6 | 1<<DDD7;
	//D2 ingång för reflexsensor
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
	}
	// Load data into SPI data register.
	SPDR = cData;
	
	// Wait until transmission completes.
	while(!(SPSR & (1<<SPIF)));
	
	// Reset SS.
	PORTB |= 1<<PORTB3 | 1<<PORTB4;
	PORTC |= 1<<PORTC0;
	
	return SPDR;
}


//----------------------------PWM------------------------------------

void PWM_Init(void)
{	
	// Configuration of the PWM setting for OC2A and OC2B
	// COM2n1:0 = 2 equals clear OC2A/B on compare match, and set at bottom
	// WGM22:0 = 3 equals fast PWM mode
	// CS22:0 = 4 sets the division factor to 64
	TCCR2A = 1<<COM2A1 | 0<<COM2A0 | 1<<COM2B1 | 0<<COM2B0 | 1<<WGM21 | 1<<WGM20;
	TCCR2B = 0<<WGM22 | 1<<CS22 | 0<<CS21 | 0<<CS20;	
	
	// Set the compare values for the PWM, 0 == 0% and 255 for 100%
	OCR2A = 0;
	OCR2B = 0;
	
	// Configuration of the PWM setting for OC1A and OC1B
	// COM1n1:0 = 2 equals clear OC1A/B on compare match, and set at bottom
	// WGM12:0 = 5 equals 8-bit fast PWM mode
	// CS12:0 = 3 sets division factor to 64
	TCCR1A = 1<<COM1A1 | 0<<COM1A0 | 1<<COM1B1 | 0<<COM1B0 | 1<<WGM11 | 0<<WGM10;
	TCCR1B = 1<<WGM12 | 1<<CS12 | 0<<CS11 | 1<<CS10;
	
	// Set the compare values for the PWM, 0 == 0% and 255 for 100%
	OCR1A = 0;
	OCR1B = 0;
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

void PWM_SetDirLeft(int dir)
{
	if (dir == 0)
	{
		PORTD &= ~(1 << PORTD0);
	}
	else if (dir == 1)
	{
		PORTD |= 1 << PORTD0;
	}
}

void PWM_SetDirRight(int dir)
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

// Setup a timer. Used by the D regulator.
void TIMER_init()
{
	TCCR0B = 1<<CS00 | 0<<CS01 | 1<<CS02; // Prescaler set to 1024
	TCNT0 = 0; // Initialize cunter
	TIMSK0 = 1<<TOIE0; // Enable timer interrupts.
}

// testa värden mellan 18 och 38 för sänk/höj-läge, så att den inte gnäller i maxlägena
void SERVO_SetSpeedVertical(int speed)
{
	if (speed >= 0 && speed <= 255)
	{
		OCR1A = speed;
	}
}

// speed = 44 ger bra grip, 18 eller ev mindre ger bra släpp
void SERVO_SetSpeedGrip(int speed)
{
	if (speed >= 0 && speed <= 255)
	{
		OCR1B = speed;
	}
}

void SERVO_SetGrip()
{
	OCR1B = 44;
}

void SERVO_ReleaseGrip()
{
	OCR1B = 18;	
}

void SERVO_LevelHigh()
{
	OCR1A = 35;
}

void SERVO_LevelLow()
{
	OCR1A = 24;
}

void SERVO_LevelMid()
{
	OCR1A = 28;
}

//----------------------------PWM----END-----------------------------

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
	LCD_SendString("    ResQ.Pl    ");
	LCD_SetRow(2);
	LCD_SendString("  Master Race  ");
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

int8_t sensor_value(int8_t val)
{
	SPI_MasterTransmit(val,'s');
	uint8_t temp = (uint8_t)SPI_MasterTransmit(0,'s');
	return temp;
}

ISR(TIMER0_OVF_vect)
{
	TIMER_overflows++;
	if (TIMER_overflows >= 10)
	{
		TIMER_overflows = 0;
		TIMER_overflows_deci++; 
		//LCD_display_int8(TIMER_overflows_deci);
		//LCD_SendCharacter(' ');
	}
	if (TIMER_overflows_deci >= 229)
	{
		TIMER_overflows_deci = 0;
		LCD_SetPosition(1);
		LCD_SendString("ALLAHU AKBAR!");
	}
}

void init_all()
{
	sleep_enable();	// Enable sleep instruction
	Styr_InitPortDirections();	// Initiate Port directions for the styrmodul.
	Styr_InitPortValues();	// Initiate Port Values for the styrmodul.
	SPI_MasterInit();	// Initiate the styrmodul as the SPI master.
	LCD_Init(); // Initiate the LCD.
	PWM_Init(); // Initiate PWM for motör
	LCD_WelcomeScreen(); // Welcomes the user with a nice message ;^)
	TIMER_init();
	sei();	// Enable global interrupts
}

//Returns speed in millimeters/milliseconds = meters/second
int speed_calculator()
{
	int start_time = TIMER_overflows_deci; //Read start time
	int wheel_circuit = 204; //Wheel circuit is 204 mm
	int wheel_marker_counter = 0; //Hjulet är uppdelat i 8 svarta och vita 8 sektioner. Öka antal?
 
	while(wheel_marker_counter < 8)
	{
		if(PIND & 0b00000100) //When PIND2 == 1 (reflex sensor) - increase counter. 
			wheel_marker_counter++;
	}
	
	return wheel_circuit/(TIMER_overflows_deci - start_time)*131; //speed = distance/(finish_time - start_time)
}

int main(void)
{
	init_all();
	//int8_t sensor_data[4];
	_delay_ms(250);		
	
	
	while(1)
	{	
		// Forever I shall repeat!		
	}
}