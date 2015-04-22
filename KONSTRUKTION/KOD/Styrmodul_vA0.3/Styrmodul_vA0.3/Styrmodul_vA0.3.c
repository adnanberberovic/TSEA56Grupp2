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
#include "Styrmodul_vA.0.3_LCD.c"
#include "Styrmodul_vA.0.3_Servo.c"

//#include <avr/pgmspace.h>

// GLOBAL VARIABLES

// One overflow corresponds to 13.1 ms if the F_CPU is 20 MHz and the defined prescaler.
// This timer will reset every time it reaches 10.
int TIMER_overflows;

// The following overflow storage corresponds to 131ms. This timer will never reset.
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
	//PORTC |= 1<<PORTC0; //Send more messages
	
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

// 0 for forward, 1 for backward
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

// 1 for forward, 0 for backward
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

void MOTOR_Forward(int speed)
{
	PWM_SetDirRight(1);
	PWM_SetDirLeft(0);
	PWM_SetSpeedLeft(speed);
	PWM_SetSpeedRight(speed);
}

void MOTOR_Backward(int speed)
{
	PWM_SetDirRight(0);
	PWM_SetDirLeft(1);
	PWM_SetSpeedLeft(speed);
	PWM_SetSpeedRight(speed);
}

void MOTOR_RotateLeft()
{
	PWM_SetDirRight(1);
	PWM_SetDirLeft(1);
	PWM_SetSpeedLeft(100);
	PWM_SetSpeedRight(100);
	for(int i = 0; i <= 8; i++)
	{
		_delay_ms(250);
	}
}

void MOTOR_RotateRight()
{
	PWM_SetDirRight(0);
	PWM_SetDirLeft(0);	
	PWM_SetSpeedLeft(100);
	PWM_SetSpeedRight(100);
	for(int i = 0; i <= 8; i++)
	{
		_delay_ms(250);
	}
}

void MOTOR_Stop()
{
	PWM_SetSpeedRight(0);
	PWM_SetSpeedLeft(0);
}

// Setup a timer. Used by the D regulator.
void TIMER_init()
{
	TCCR0B = 1<<CS00 | 0<<CS01 | 1<<CS02; // Prescaler set to 1024
	TCNT0 = 0; // Initialize cunter
	TIMSK0 = 1<<TOIE0; // Enable timer interrupts.
}

//----------------------------PWM----END-----------------------------

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
	}
	//if (TIMER_overflows_deci >= 229)
	//{
		//TIMER_overflows_deci = 0;
		//LCD_SetPosition(1);
		//LCD_SendString("ALLAHU AKBAR!");
	//}
}
//----------------------------GYRO------------------------------------
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

//----------------------------GYRO----END-----------------------------


void init_all()
{
	sleep_enable();	// Enable sleep instruction
	Styr_InitPortDirections();	// Initiate Port directions for the styrmodul.
	Styr_InitPortValues();	// Initiate Port Values for the styrmodul.
	SPI_MasterInit();	// Initiate the styrmodul as the SPI master.
	LCD_Init(); // Initiate the LCD.
	PWM_Init(); // Initiate PWM for motor
	TIMER_init(); // Initiate Timer settings
	LCD_WelcomeScreen(); // Welcomes the user with a nice message ;^)
	sei();	// Enable global interrupts
}

//Returns speed in millimeters/milliseconds = meters/second
int speed_calculator()
{
	int start_time = TIMER_overflows_deci; //Read start time
	int wheel_circumference = 204; //Wheel circumference is 204 mm
	int wheel_marker_counter = 0; //Hjulet är uppdelat i 8 svarta och vita 8 sektioner. Öka antal?
 
	while(wheel_marker_counter < 8)
	{
		if(PIND & 0b00000100) //When PIND2 == 1 (reflex sensor) - increase counter. 
			wheel_marker_counter++;
	}
	
	return wheel_circumference/(TIMER_overflows_deci - start_time)*131; //speed = distance/(finish_time - start_time)
}

void Drive_test()
{
	MOTOR_Forward(80);
	SERVO_LevelHigh();
	for(int i = 0; i <= 15; i++)
	{
		_delay_ms(250);
	}
	MOTOR_RotateRight();
	MOTOR_Stop();
	SERVO_SetGrip();
	for(int i = 0; i <= 5; i++)
	{
		_delay_ms(250);
	}
	SERVO_LevelMid();
	MOTOR_Backward(80);
	for(int i = 0; i <= 15; i++)
	{
		_delay_ms(250);
	}
	SERVO_ReleaseGrip();
	MOTOR_RotateLeft();
	MOTOR_Stop();
	for(int i = 0; i <= 5; i++)
	{
		_delay_ms(250);
	}
	SERVO_LevelLow();
	MOTOR_Backward(80);
	for(int i = 0; i <= 32; i++)
	{
		_delay_ms(250);
	}
	SERVO_LevelMid();
	SERVO_SetGrip();
	MOTOR_Stop();
	for(int i = 0; i <= 5; i++)
	{
		_delay_ms(250);
	}
	SERVO_ReleaseGrip();
}

int main(void)
{
	init_all();
	//int8_t sensor_data[4];
	_delay_ms(250);		
	
	
	/* logik test */
	uint8_t dataH, dataL, result;
	SPI_MasterTransmit(0b10010100,'g'); // activation BRA
	//_delay_us(50); // to separate packets
	dataH = SPI_MasterTransmit(0x00,'g'); // dummy
	//_delay_us(50); // to separate packets
	dataL = SPI_MasterTransmit(0x00,'g'); // dummy
	_delay_us(200); // wait for EOC to be set / separate instructions
	LCD_display_int8(dataH & 0b10000000);
	_delay_ms(200);
	LCD_display_int8(dataH & 0b00100000);
	_delay_ms(200);
	PORTC |= 1<<PORTC0;
	
	while(1)
	{
		LCD_Clear();
		SPI_MasterTransmit(0b10010100,'g'); // select angular rate channel
		//_delay_us(50); // to separate packets
		dataH = SPI_MasterTransmit(0x00,'g'); // dummy
		//_delay_us(50); // to separate packets
		dataL = SPI_MasterTransmit(0x00,'g'); // dummy
		//_delay_us(200); // to separate instructions
		PORTC |= 1<<PORTC0;
 		LCD_display_int8(dataH & 0b10000000);
 		_delay_ms(1000);

		do
			{
				LCD_Clear();
				SPI_MasterTransmit(0b10000000,'g'); // Start conversion Gyro sänder till SPI
				//_delay_us(50); // to separate packets
				dataH = SPI_MasterTransmit(0x00, 'g'); // dummy
				//_delay_us(50); // to separate packets
				dataL = SPI_MasterTransmit(0x00,'g'); // dummy
				_delay_us(400);
				PORTC |= 1<<PORTC0; //Gyro sänder inte längre till SPI
 				LCD_display_int8(dataH & 0b10000000);
 				_delay_ms(400);
 				LCD_display_int8(dataH & 0b00100000);
				_delay_ms(400);
 			} while(!(dataH & 0b00100100)); // read ADC until EOC bit is set
// 	LCD_display_int8(dataH & 0b10000000);
// 	_delay_ms(400);
// 	LCD_display_int8(dataH & 0b00100000);
// 	_delay_ms(400);
// 	
	}
	while(1)
	{	
		// Forever I shall repeat!
		//Drive_test();
		LCD_SendCharacter(SPI_MasterTransmit('0','k'));
		_delay_ms(250);
	}
}