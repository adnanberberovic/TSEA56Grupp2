/*
 * Servo
 *
 * Created: 4/22/2015 10:39:10 AM
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

// testa v�rden mellan 18 och 38 f�r s�nk/h�j-l�ge, s� att den inte gn�ller i maxl�gena
void SERVO_SetSpeedVertical(int speed)
{
	if (speed >= 0 && speed <= 255)
	{
		OCR1A = speed;
	}
}

// speed = 44 ger bra grip, 18 eller ev mindre ger bra sl�pp
void SERVO_SetSpeedGrip(int speed)
{
	if (speed >= 0 && speed <= 255)
	{
		OCR1B = speed;
	}
}

void SERVO_SetGrip()
{
	OCR1B = 48; //�ka f�r att greppa b�ttre! V�gar inte skicka in f�r stora v�rden.
}

void SERVO_ReleaseGrip()
{
	OCR1B = 18;
}

void SERVO_LevelHigh()
{
	OCR1A = 34;
}

void SERVO_LevelLow()
{
	OCR1A = 24;
}

void SERVO_LevelMid()
{
	OCR1A = 28;
}
