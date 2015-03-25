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


double sensor_data[8]; //Skapa en array med 8 element.

int main(void)
{
	DDRB = 1; 
	DDRB= 255;
	DDRB |= (1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0); //Sätt Port B till utgång
	DDRA |= (0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0); //Sätt Port A till ingång (default)
		
	sei(); //Aktiverar globala avbrott
	ADCSRA |= (1<<7); //Aktivera ADC
	ADCSRA |= (1<<3); //Aktivera ADC-interrupt
	ADCSRA |= (1<<0)|(1<<1)|(1<<2); //Sätt division factor till 128. 20 MHz/128 = 156,25 kHz
	
	//OBS voltage reference may be externally decoupled for better noise performance.		
		
			
	for (int i = 0; i < 5; i++) //Läs av alla 8 sensorer. Ev ha de två långa sist och läs av värden vid behov?
		{
  			ADMUX = i+32; //Olika värden ger olika ADC input. Lägg till 32 för att sätta adlar = vänsterjustera.
			ADCSRA |= (1<<6); //Starta omvandling
			//SLEEP_MODE_ADC;
			_delay_ms(1000);

			//sensor_data[i] = ADCH;
				
			//PORTB = 0;
			//PORTB = sensor_data[i];
			//Kolla ADCH mot spänning-avstånd-tabell?
			//om vi är för långt för kortavstånd - läs av de två sista värdena 
		}
			PORTB = 0;
			PORTB = sensor_data[0];
			PORTB = sensor_data[1];
			PORTB = sensor_data[2];
			PORTB = sensor_data[3];
			PORTB = sensor_data[4];
			PORTB = sensor_data[5];
		//ADC tar 13 CC (OBS - ADC har annan klockfrekvens. Vi använder Single Conversion.
		//Därför omvandlar vi varje signal och omvandlar det förra resultatet till meter under tiden.
		
		//Kolla värdet i minnet. Omvandla till meter eller görs det i styrmodulen?
			
		//Avbrottsnivån behöver ej höjas som i designspec: ADC tar hand om det.
		//This ensures that the channels and reference selection only takes place at a safe point during the conversion.
	
	
	
}


ISR(ADC_vect)
{
	//Vilken läsning har skett? Använd ADMUX för att bestämma!
	//int admuxcounter = ADMUX-32;
	
	sensor_data[ADMUX-32] = ADCH;
	PORTB = ADCH; //Skickar alla 8 bitar till PortB som kopplas till dioder
	PORTB = 0;
	PORTB = sensor_data[ADMUX-32];
	PORTB = 0;
}