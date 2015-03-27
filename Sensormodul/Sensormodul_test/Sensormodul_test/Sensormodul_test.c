/*
 * Sensormodul_test.c 
 *
 * Created: 3/23/2015 12:46:10 PM
 *  Author: frefr166
 */ 


#include<avr/io.h>
#include<avr/interrupt.h>
#include<avr/sleep.h>
#include<math.h> 
#include<stdlib.h>
#include<stdio.h>
#include<util/delay.h>
#include"H:/Documents/GitHub/TSEA56Grupp2/Sensormodul/distance_table.h"
//H:\Documents\GitHub\TSEA56Grupp2\Sensormodul\distance_table.h

int sensor_data[8]; //Skapa en array med 8 element.
int distance_table[255]; //Hur f�r vi ut den till c eller h-filen?

void distance_table_generator()
{
	for (int i = 0; i < 38; i++)
	{
		distance_table[i] = 255;
	}
	
	distance_table[38] = 170;
	distance_table[39] = 168;
	distance_table[40] = 164;
	distance_table[41] = 160;
	distance_table[42] = 156;
	distance_table[43] = 150;
	distance_table[44] = 144;
	
	int k = 0;
	
	for (int i = 45; i < 65; i++)
	{
		distance_table[i] = 140 - k*2;
		k = k + 1;
	}
	
	k = 0;
	for(int i = 65; i < 105; i++)
	{
		distance_table[i] = 100 - k;
		k = k + 1;
	}
	
	k = 0;
	
	for (int i = 105; i < 150; i++)
	{
		distance_table[i] = 60 - k;
		
		if ((i/2)*2 == i){
			k = k + 1;
		}
	}
	
	for (int i = 150; i < 255; i++)
	{
		distance_table[i] = 0;
	}
	
	return;
	
}
int admuxcounter = 0; 
//int value_converter(int meter)

int main(void)
{ 
	distance_table_generator();
	
	DDRB = 255;//S�tt Port B till utg�ng
	DDRA = 0; //S�tt Port A till ing�ng (default)
	sei(); //Aktiverar globala avbrott
	ADCSRA = 143; //Aktivera ADC, ADC-interrupt, S�tt division factor till 128. 20 MHz/128 = 156,25 kHz
	ADMUX = 32; //S�tt ADLAR - v�nsterjustera resultatet.
			
	while(1)
	{
		ADCSRA |= (1<<6); //B�rja ADC
		while(ADCSRA & 1<<ADSC)
		{
		} //Delay:ar s� l�nge vi omvandlar!
		
		//OBS voltage reference may be externally decoupled for better noise performance.
		
		//Gibberish-operationer f�r att testa. H�r borde det st� omvandlingsfunktioner!
		PORTB = 2; 
		PORTB = 1;
		PORTB = 0;
	}
		
// 	for (int i = 0; i < 5; i++) //L�s av alla 8 sensorer. Ev ha de tv� l�nga sist och l�s av v�rden vid behov?
// 		{
//   			ADMUX = i+32; //Olika v�rden ger olika ADC input. L�gg till 32 f�r att s�tta adlar = v�nsterjustera.
// 			ADCSRA |= (1<<6); //Starta omvandling
// 			//SLEEP_MODE_ADC;
// 			_delay_ms(1000);
// 
// 			sensor_data[i] = ADCH;
// 				
// 			//PORTB = 0;
// 			//PORTB = sensor_data[i];
// 			//Kolla ADCH mot sp�nning-avst�nd-tabell?
// 			//om vi �r f�r l�ngt f�r kortavst�nd - l�s av de tv� sista v�rdena 
// 		}

			
}


ISR(ADC_vect)
{
	PORTB = ADCH;
	PORTB = 0;
	sensor_data[admuxcounter]= distance_table[ADCH];

	if (admuxcounter < 7)
		{
			ADMUX += 1;
			admuxcounter += 1;
		}
	else
		{
			ADMUX = 32;
			admuxcounter = 0;
		}
}


//ADMUX 0 = Kort
//ADMUX 1 = Kort
//ADMUX 2 = Kort
//ADMUX 3 = Kort
//ADMUX 4 = Kort
//ADMUX 5 = Reflex
//ADMUX 6 = L�ng
//ADMUX 7 = L�ng
//Jobbigt att fixa till s� att vi hoppar till l�nga sensorn om vi inte ser n�got p� korta. Vi tar de mikrosekunderna!
