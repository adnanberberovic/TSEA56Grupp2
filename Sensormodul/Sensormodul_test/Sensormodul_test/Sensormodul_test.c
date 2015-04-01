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

int sensor_data [8]; //Skapa en array med 8 element.
int send_buffer [4];
int distance_table[255]; //Hur får vi ut den till c eller h-filen?
int buffer_flag = 0;


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

int angle_generator(int back, int front)
{
	int angle;
	double length_quotient;
	double length_difference;
	
	length_difference = front - back;
	length_quotient= length_difference/130; //130 är avståndet mellan sensorerna
	angle = atan(length_quotient) * 180/M_PI;
	return angle;
}

int offset_generator(int angle, int back, int front)
{
	int hyp;
	int cath;
	
	hyp = (front + back)/2 + 100; //100 är avståndet mellan sensorerna
	cath = hyp * cos(angle*M_PI/180);
	
	return cath/10;
}

int main(void)
{ 
	int angle;
	int offset;
	int front_sensor;
	int wall_reflex_information;
	int reflex_bool;
	int left_wall_counter;
	int right_wall_counter;
	
	distance_table_generator(); //Skapa avståndstabellen
	
	DDRB = 255;//Sätt Port B till utgång
	DDRA = 0; //Sätt Port A till ingång (default)
	sei(); //Aktiverar globala avbrott
	ADCSRA = 143; //Aktivera ADC, ADC-interrupt, Sätt division factor till 128. 20 MHz/128 = 156,25 kHz
		
	while(1)
	{
		
//_________________________________________Avläsning________________________________________
		for(int i = 0; i < 7; i++)
		{
			ADMUX = 32 + i; //Öka admux, sätt ADLAR (bit 5 = 32)
			ADCSRA |= (1<<6); //Börja ADC
			
		while(ADCSRA & 1<<ADSC)
			{
			} //Delay så att inte ADMUX-inläsningarna hamnar i oordning
		}


//_______________________________________Offset och angle______________________________________
		//Välj den sida som är närmast väggen - den är mest noggrann!
		//Är höger < vänster? Använd isf höger och vice versa.
		if((sensor_data[3] + sensor_data[4]) < (sensor_data[0]+sensor_data[1])) 
			{
				angle= angle_generator(sensor_data[3],sensor_data[4]);
				offset = 40 - offset_generator(angle,sensor_data[3],sensor_data[4]);
			}
		else
			{
				angle= angle_generator(sensor_data[0],sensor_data[1]);
				offset = offset_generator(angle,sensor_data[0],sensor_data[1]);
			}
		
//_________________________________________Frontsensor________________________________________
		//Dividera resultatet med 10 för att det ska bli centimeter
		//Detta är för att vi bara kan beskriva högst 127 mm annars blir det tvåkomplement
		front_sensor = sensor_data[6]/10;
				
//_________________________________________Reflexsensor________________________________________
		if((sensor_data[7] > 127))
			reflex_bool = 1;
		else
			reflex_bool = 0;
					
//_________________________________________Lång sensor________________________________________
		//Om kort inte detekterar ett avstånd - läs av lång sensor.
		//Skicka ut hur många väggar vi ser från tabell.
		if((sensor_data[0] == 0) || (sensor_data[1] == 0))
			{
			if(sensor_data[2] > 51) 
				left_wall_counter = 0;
			else if(50 > sensor_data[2] && sensor_data[2] > 40) 
				left_wall_counter = 1;
			else if (40 > sensor_data[2] && sensor_data[2] > 22)
				left_wall_counter = 2;
			else
				left_wall_counter = 3;
			}
		else
			//Skicka ut hur många väggar vi ser med den korta
			{
				left_wall_counter = 0;
			}
		
		if((sensor_data[3] == 0) || (sensor_data[4] == 0))
		{
			if(sensor_data[5] > 51)
			right_wall_counter = 0;
			else if(50 > sensor_data[5] && sensor_data[5] > 40)
			right_wall_counter = 1;
			else if (40 > sensor_data[5] && sensor_data[5] > 22)
			right_wall_counter = 2;
			else
			right_wall_counter = 3;
		}
		else
		{
			right_wall_counter = 0;
		}
		
		
//_________________________________________Uppdatera buffer________________________________________
		//Samla ihop väggarna och reflexen i en binär talföljd. Lägg reflex_bool på 7 biten
		//Lägg vänster vägg på 4 och 5 biten, lägg höger vägg på 1 och 2 biten.
		//För avläsning - and:a bort de ointressanta bitarna och dividera med faktorn nedan.
		
		wall_reflex_information = reflex_bool * 64 + left_wall_counter * 8 + right_wall_counter;
			
		//Förhindra avbrott under uppdateringen - höj avbrottsnivån så inga bussavbrott kommer.
		cli();
		send_buffer[0] = angle;
		send_buffer[1] = offset;
		send_buffer[2] = front_sensor;
		send_buffer[3] = wall_reflex_information;
		sei();


//Varför skickar inte reflexen ut sina värden?
//Borde ligga på sensor_data[7] men gör inte det.
//Ska sensor_data ha 7 element istället för 8?
//Börjar den på 0 eller 1?

		PORTB = 0;
		PORTB = sensor_data[7];
		PORTB = 0;
		PORTB = wall_reflex_information;
		PORTB = send_buffer[0];
		PORTB = send_buffer[1];
		PORTB = send_buffer[2];
		PORTB = send_buffer[3];		
	}			
}

ISR(ADC_vect)
{
	
	//Försökte hitta reflexsensorns värde!
	PORTB = ADCH;
	PORTB = sensor_data[7];
	PORTB = sensor_data[8];
	//_delay_ms(1000); Varför har vi en delay här?
	sensor_data[ADMUX-32] = distance_table[ADCH];
}

/*
ISR(SPI_STC_vect)
{
	if(buffer_flag = 0)
	{
		skicka send_buffer_2;
	}
	else
	{
		skicka send_buffer_1;
	}
	PORTB = ADCH;
	PORTB = 0;
	sensor_data[ADMUX-32] = distance_table[ADCH];
}

*/

/*
ADMUX 0 = Kort vänster bak
ADMUX 1 = Kort vänster fram
ADMUX 2 = Lång vänster
ADMUX 3 = Kort höger bak
ADMUX 4 = Kort höger fram
ADMUX 5 = Lång höger
ADMUX 6 = Kort framåt
ADMUX 7 = Reflex
*/