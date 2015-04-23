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
#include "Styrmodul_LCD.c"
#include "Styrmodul_Servo.c"



//#include <avr/pgmspace.h>

// GLOBAL VARIABLES

// Sets position for cursor on LCD. Argument should be a number in the range of 0-31.
//void LCD_SetPosition(uint8_t pos)
//{
	//LCD_Counter =(int) pos - 1;
	//while(LCD_Busy())
	//{
		//_delay_ms(1);
	//}
	//PORTB &= ~(1 << 0); // Clear RS and
	//PORTB &= ~(1 << 1); // clear R/W bits so that the following commands can be run
	//
	//if (pos < 16)
	//{
		//LCD_SendCommand(128+pos);
	//}
	//else if (pos < 32)
	//{
		//LCD_SendCommand(128+64-16+pos);
	//}
	//else LCD_SendCommand(0b10000000);
	//
//}

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
	SPCR = 0<<SPIE | 1<<SPE | 0<<DORD | 1<<MSTR | 0<<CPOL | 0<<CPHA | 1<<SPR1 | 1<<SPR0;
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
	//PORTC |= 1<<PORTC0;
		
	return SPDR;
}

void set_GyroSS_Low()
{
	PORTC &= ~(1<<PORTC0);
}
void set_GyroSS_High()
{
	PORTC |= (1<<PORTC0);
}

unsigned char SPI_MasterTransmit_Gyro(unsigned char cData)
{
	// Load data into SPI data register.
	SPDR = cData;
	
	// Wait until transmission completes.
	while(!(SPSR & (1<<SPIF)));
	
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
/*
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
*/

void Gyro_Init()
{
	int8_t first_byte1;
	do{
		set_GyroSS_Low();
		SPI_MasterTransmit_Gyro(0b10010100); //Activate adc
		first_byte1 = SPI_MasterTransmit_Gyro(0x00); //Byte with EOC and Accepted instr. bit
		SPI_MasterTransmit_Gyro(0x00); //Last byte
		set_GyroSS_High();
	} while ( (first_byte1 & 0b10000000) & !(first_byte1 & 0b00100000)); // IF EOC = 0 and acc.instr. = 1 we continue
}

void Gyro_StartConversion()
{
	int8_t first_byte2;
	do{
		set_GyroSS_Low();
		SPI_MasterTransmit_Gyro(0b10010100); //Activate adc with angular rate signal
		first_byte2 = SPI_MasterTransmit_Gyro(0x00); //Byte with EOC and Accepted instr. bit
		SPI_MasterTransmit_Gyro(0x00); //Last byte
		set_GyroSS_High();
	} while (first_byte2 & 0b10000000); // IF  acc.instr. = 1 we continue
}

int16_t Gyro_PollResult()
{
	int8_t first_byte3;
	int8_t second_byte;
	int16_t return_val;
	
		do{
			set_GyroSS_Low();
			SPI_MasterTransmit_Gyro(0b10000000); //Activate adc
			first_byte3 = SPI_MasterTransmit_Gyro(0x00); //Byte with EOC and Accepted instr. bit
			second_byte = SPI_MasterTransmit_Gyro(0x00); //Last byte
			set_GyroSS_High();
		} while ( (first_byte3 & 0b10000000) & !(first_byte3 & 0b00100000)); // IF EOC = 0 and acc.instr. = 1 we continue
		
	return_val = ((first_byte3 & 0x00ff) << 8) | (second_byte);
	return return_val;
}

int16_t Get_ADC_value(int16_t inval_)
{
	int16_t result_;
	result_ = inval_ & 0x0fff;
	result_ = result_ >> 1;

	return result_;
} 

//----------------------------GYRO----END-----------------------------



//Returns speed in millimeters/milliseconds = meters/second
int speed_calculator()
{
	int start_time = TIMER_overflows_deci; //Read start time
	int wheel_circumference = 204; //Wheel circumference is 204 mm
	int wheel_marker_counter = 0; //Hjulet är uppdelat i 8 svarta och vita 8 sektioner. Öka antal?
 
	while(wheel_marker_counter < 8)
	{
		if(
		) //When PIND2 == 1 (reflex sensor) - increase counter. 
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

//DORD: Data order. Set to 1 to transmit LSB first, MSB last
//Spegla det som skkickas !

void Gyro_test()
{
	while (1)
	{
		LCD_Clear();
		LCD_SetPosition(0);
		Gyro_StartConversion();
		result = Gyro_PollResult();
		result = Get_ADC_value(result);
		//LCD_display_uint16(adcToAngularRate( result ));
		LCD_display_int16(adcToAngularRate( result ));
		_delay_ms(250);
	}
	
	return;
}


void Speed_test()
{
	LCD_display_int8(PIND2);
}

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
	Gyro_Init();
	sei();	// Enable global interrupts
}

int main(void)
{
	init_all();
	//int8_t sensor_data[4];
	_delay_ms(250);		
	
	int16_t result = 0;
	int8_t dataH, dataL;
	Gyro_ActivateADC();
	
	while (1)
	{
		//Gyro_test();
		//Drive_test();
		Speed_test();
	}
	
}
