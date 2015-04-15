/*
 * Sensormodul_test.c 
 *
 * Created: 3/23/2015 12:46:10 PM
 *  Author: frefr166
 */ 
#define F_CPU 20000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<avr/sleep.h>
#include<math.h> 
#include<stdlib.h>
#include<stdio.h>
#include<util/delay.h>

int sensor_data [8]; //Skapa en array med 8 element.
int send_buffer [4];
int distance_table[255]; 
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
	length_quotient= length_difference/130; //130 �r avst�ndet mellan sensorerna
	angle = atan(length_quotient) * 180/M_PI;
	return angle;
}

int offset_generator(int angle, int back, int front)
{
	int hyp;
	int cath;
	
	hyp = (front + back)/2 + 80; //100 �r avst�ndet mellan sensorerna
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
	
	distance_table_generator(); //Skapa avst�ndstabellen
	
	DDRB = 255;//S�tt Port B till utg�ng
	DDRA = 0; //S�tt Port A till ing�ng (default)
	sei(); //Aktiverar globala avbrott
	ADCSRA = 143; //Aktivera ADC, ADC-interrupt, S�tt division factor till 128. 20 MHz/128 = 156,25 kHz
		
	while(1)
	{
		
//_________________________________________Avl�sning________________________________________
		for(int i = 0; i < 8; i++)
		{
			ADMUX = 32 + i; //�ka admux, s�tt ADLAR (bit 5 = 32)
			ADCSRA |= (1<<6); //B�rja ADC
			
		while(ADCSRA & 1<<ADSC)
			{
			} //Delay s� att inte ADMUX-inl�sningarna hamnar i oordning
		}


//_______________________________________Offset och angle______________________________________
		//V�lj den sida som �r n�rmast v�ggen - den �r mest noggrann!
		//�r h�ger < v�nster? Anv�nd isf h�ger och vice versa.
		
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
		//Dividera resultatet med 10 f�r att det ska bli centimeter
		//Detta �r f�r att vi bara kan beskriva h�gst 127 mm annars blir det tv�komplement
		//NEJ! VI SKICKAR I MILLIMETER VI SKITER I J�VLA 2KOMP 
		front_sensor = sensor_data[6];
				
//_________________________________________Reflexsensor________________________________________
		if((sensor_data[7] > 127))
			reflex_bool = 1;
		else
			reflex_bool = 0;

//_________________________________________L�ng sensor________________________________________
		//Om kort inte detekterar ett avst�nd - l�s av l�ng sensor.
		//Skicka ut hur m�nga v�ggar vi ser fr�n tabell.

		
		PORTB = sensor_data[0];
		PORTB = sensor_data[1];
		PORTB = sensor_data[2];
			//V�nster l�ngsensor
			if (sensor_data[0] > 170|| sensor_data[1] > 170)
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
				{
					left_wall_counter = 0;
				}

		PORTB = left_wall_counter;
		PORTB = sensor_data[3];
		PORTB = sensor_data[4];
		PORTB = sensor_data[5];
			if(sensor_data[3] > 170|| sensor_data[4] > 170){
				
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
			
		PORTB = right_wall_counter;
		PORTB = sensor_data[6];
		
		
		
		
//_________________________________________Uppdatera buffer________________________________________
		//Samla ihop v�ggarna och reflexen i en bin�r talf�ljd. L�gg reflex_bool p� 7 biten
		//L�gg v�nster v�gg p� 4 och 5 biten, l�gg h�ger v�gg p� 1 och 2 biten.
		//F�r avl�sning - and:a bort de ointressanta bitarna och dividera med r�tt faktor.
		
		wall_reflex_information = ( (reflex_bool * 64) + (left_wall_counter * 8) + (right_wall_counter) );
			
		//F�rhindra avbrott under uppdateringen - h�j avbrottsniv�n s� inga bussavbrott kommer.
		cli();
		send_buffer[0] = angle;
		send_buffer[1] = offset;
		send_buffer[2] = front_sensor;
		send_buffer[3] = wall_reflex_information;
		//L�gg send buffer p� r�tt st�lle f�r M�ns funktion
		sei();
		
		PORTB=send_buffer[0];	
		PORTB=send_buffer[1];
		PORTB=send_buffer[2];
		PORTB=send_buffer[3];
		//_________________________________________TEST________________________________________
		_delay_ms(1000);
	}			
}

ISR(ADC_vect)
{
	if ( (ADMUX-32 == 7) || (ADMUX-32 == 2) || (ADMUX-32 == 5) ){
	sensor_data[ADMUX-32] = ADCH; // Ifall det �r l�nga sensorn eller reflex ska den inte konverteras.
	}
	else
	{
	sensor_data[ADMUX-32] = distance_table[ADCH];
	}	
		
}


ISR(SPI_STC_vect)
{
	//M�ns funktion startas
}



/*
ADMUX 0 = Kort v�nster bak
ADMUX 1 = Kort v�nster fram
ADMUX 2 = L�ng v�nster
ADMUX 3 = Kort h�ger bak
ADMUX 4 = Kort h�ger fram
ADMUX 5 = L�ng h�ger
ADMUX 6 = Kort fram�t

�r riktningen OK?!
*/