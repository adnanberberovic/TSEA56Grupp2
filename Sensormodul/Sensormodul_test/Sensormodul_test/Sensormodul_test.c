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
int distance_table[255]; //Hur får vi ut den till c eller h-filen?

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
	
	DDRB = 255;//Sätt Port B till utgång
	DDRA = 0; //Sätt Port A till ingång (default)
	sei(); //Aktiverar globala avbrott
	ADCSRA = 143; //Aktivera ADC, ADC-interrupt, Sätt division factor till 128. 20 MHz/128 = 156,25 kHz
	ADMUX = 32; //Sätt ADLAR - vänsterjustera resultatet.
			
	while(1)
	{
		ADCSRA |= (1<<6); //Börja ADC
		while(ADCSRA & 1<<ADSC)
		{
		} //Delay:ar så länge vi omvandlar!
		
		//OBS voltage reference may be externally decoupled for better noise performance.
		
		//Gibberish-operationer för att testa. Här borde det stå omvandlingsfunktioner!
		PORTB = 2; 
		PORTB = 1;
		PORTB = 0;
	}
		
// 	for (int i = 0; i < 5; i++) //Läs av alla 8 sensorer. Ev ha de två långa sist och läs av värden vid behov?
// 		{
//   			ADMUX = i+32; //Olika värden ger olika ADC input. Lägg till 32 för att sätta adlar = vänsterjustera.
// 			ADCSRA |= (1<<6); //Starta omvandling
// 			//SLEEP_MODE_ADC;
// 			_delay_ms(1000);
// 
// 			sensor_data[i] = ADCH;
// 				
// 			//PORTB = 0;
// 			//PORTB = sensor_data[i];
// 			//Kolla ADCH mot spänning-avstånd-tabell?
// 			//om vi är för långt för kortavstånd - läs av de två sista värdena 
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
//ADMUX 6 = Lång
//ADMUX 7 = Lång
//Jobbigt att fixa till så att vi hoppar till långa sensorn om vi inte ser något på korta. Vi tar de mikrosekunderna!
