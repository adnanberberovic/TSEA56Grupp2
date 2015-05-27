/*
 * Sensor module Version 1.0 
 *
 * Created: 3/23/2015 12:46:10 PM
 * Author: Grupp 2 TSEA56 2015 
 */ 
#define F_CPU 20000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<avr/sleep.h>
#include<math.h> 
#include<stdlib.h>
#include<stdio.h>
#include<util/delay.h>

volatile int sensor_data [8]; //Skapa en array med 8 element.
int8_t send_buffer [7];
int distance_table[255]; 
int buffer_flag = 0;

int SPI_counter = 3;

void Sensor_InitPortDirections()
{
	DDRB = 1<<DDB6;
	DDRA = 0;
	DDRD = 1<<DDD5 | 1<<DDD6 | 1<<DDD7;
	// 0 = input
	// 1 = output
}
void Sensor_InitPortValues()
{
	PORTB = 1<<PORTB4;
}

void SPI_SlaveInit()
{
	SPSR = 0<<SPI2X;
	SPCR = 1<<SPIE | 1<<SPE | 1<<DORD | 0<<MSTR | 0<<CPOL | 0<<CPHA | 1<<SPR1 | 1<<SPR0;
	//SPIE: SPI interrupt enable. Set to 1 to allow interrupst
	
	//SPE: SPI Enable. Set to 1 to allow SPI communication
	//DORD: Data order. Set to 1 to transmit LSB first, MSB last.
	//MSTR: Master select. Set to 1 to set master.
	//CPOL and CPHA: set to 0 to sample on rising edge and setup on falling edge.
	//SPI2X, SPR1, SPR0, set to 0,1,1 to scale clock with f_osc/128.
	//Bus config similar to comm and sensor module, though set 0<<MSTR
}

void distance_table_generator()
{
	for (int i = 0; i < 38; i++)
	{
		distance_table[i] = 210;
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

int8_t angle_generator(int back, int front)
{
	float angle;
	double length_quotient;
	double length_difference;
	
	length_difference = front - back;
	length_quotient= length_difference/130; //130 är avståndet mellan sensorerna
	angle = atan(length_quotient) * 180/M_PI;
	return (int8_t)angle;
}

int8_t offset_generator(int angle, int back, int front)
{
	float hyp;
	float cath;
	
	hyp = (front + back)/2 + 160/2; //160 är avståndet mellan sensorerna
	cath = hyp * cos(angle*M_PI/180);
	
	return (int8_t)(cath/10);
}

int main(void)
{ 
	int8_t angle_left;
	int8_t offset_left;
	int8_t angle_right;
	int8_t offset_right;
	uint8_t front_sensor;
	int8_t wall_reflex_information;
	int8_t can_see_information;
	int reflex_value_floor;
	int reflex_bool;
	int8_t left_wall_counter;
	int8_t right_wall_counter;
	int left_path_both;
	int right_path_both;
	int left_path_one;
	int right_path_one;
	
	distance_table_generator(); //Skapa avståndstabellen
	
	Sensor_InitPortValues();
	Sensor_InitPortDirections();
	SPI_SlaveInit();
	
	sei(); //Aktiverar globala avbrott
	ADCSRA = 0x87; //Aktivera ADC, Sätt division factor till 128. 20 MHz/128 = 156,25 kHz
		

	ADMUX = 32 + 7; //Öka admux, sätt ADLAR (bit 5 = 32)
	ADCSRA |= (1<<6); //Börja ADC

	while(ADCSRA & 1<<ADSC)
	{
		_delay_us(1);
	} //Delay så att inte ADMUX-inläsningarna hamnar i oordning


	reflex_value_floor= ADCH+40;
		
	while(1)
	{
		
//_________________________________________Avläsning________________________________________
	
	for(int i = 0; i < 8; i++) // Addera in värden i arrayen
		{
			ADMUX = 32 + i; //Öka admux, sätt ADLAR (bit 5 = 32)
			ADCSRA |= (1<<6); //Börja ADC
			
			while(ADCSRA & 1<<ADSC)
				{
				} //Delay så att inte ADMUX-inläsningarna hamnar i oordning
				
			if (
			 (i == 7) || (i == 2) || (i == 5) || (i == 6))
			{
				sensor_data[i] = ADCH;
			}
			else
			{
				sensor_data[i] = distance_table[ADCH];
			}	
		}
		

//_______________________________________Offset och angle______________________________________
		//Välj den sida som är närmast väggen - den är mest noggrann!
		//Är höger < vänster? Använd isf höger och vice versa.
		
		angle_right= angle_generator(sensor_data[3],sensor_data[4]);
		offset_right = 40 - offset_generator(angle_right,sensor_data[3],sensor_data[4]);
		
		angle_left= angle_generator(sensor_data[0],sensor_data[1]);
		offset_left = offset_generator(angle_left,sensor_data[0],sensor_data[1]);
					
//_________________________________________Frontsensor________________________________________

		//Skickar spänning till styrmodulen. Gör om?
		front_sensor = (int8_t)(sensor_data[6]);
		
		

//_________________________________________Reflexsensor________________________________________
	
		if((sensor_data[7] > reflex_value_floor))
			reflex_bool = 1;
		else
			reflex_bool = 0;

//_________________________________________Lång sensor________________________________________
		//Om kort inte detekterar ett avstånd - läs av lång sensor.
		//Skicka ut hur många väggar vi ser från tabell.

			//Vänster långsensor
			if (sensor_data[0] > 170 || sensor_data[1] > 170)
			{			
				if(sensor_data[2] >= 85) 
					left_wall_counter = 0;
				else if(85 > sensor_data[2] && sensor_data[2] >= 45) 
					left_wall_counter = 1;
				else if (45 > sensor_data[2] && sensor_data[2] >= 32)
					left_wall_counter = 2;
				else
					left_wall_counter = 3;	
			}
			else
			{
				left_wall_counter = 0;
			}
			
			//Höger långsensor
			if (sensor_data[3] > 170 || sensor_data[4] > 170)
			{
				if(sensor_data[5] >= 90)
					right_wall_counter = 0;
				else if(90 > sensor_data[5] && sensor_data[5] >= 50)
					right_wall_counter = 1;
				else if (50 > sensor_data[5] && sensor_data[5] >= 37)
					right_wall_counter = 2;
				else
					right_wall_counter = 3;
			}
			else
			{
				right_wall_counter = 0;
			}
			
					
//_________________________________________Skapa bools för styrbeslut______________________________________
			if((sensor_data[0] > 200) && (sensor_data[1] > 200)
				&& left_wall_counter > 0)
			{
				left_path_both = 1;
			}
			else
			{
				left_path_both = 0;
			}
					
			if((sensor_data[3] > 200) && (sensor_data[4] > 200)
				&& right_wall_counter > 0)
			{
				right_path_both = 1;
			}
			else
			{
				right_path_both = 0;
			}
					
			if((sensor_data[0] > 200) || (sensor_data[1] > 200))
			{
				left_path_one = 1;
			}
			else
			{
				left_path_one = 0;
			}
					
			if((sensor_data[3] > 200) || (sensor_data[4] > 200))
			{
				right_path_one = 1;
			}
			else
			{
				right_path_one = 0;
			}
		
//_________________________________________Uppdatera buffer________________________________________
		//Samla ihop väggarna och reflexen i en binär talföljd. Lägg reflex_bool på 7 biten
		//Lägg vänster vägg på 4 och 5 biten, lägg höger vägg på 1 och 2 biten.
		//För avläsning - and:a bort de ointressanta bitarna och dividera med rätt faktor.
				
		can_see_information = ( (left_path_one * 8) + (right_path_one * 4) +
								(left_path_both * 2) + right_path_both );
		
		//	CSI	=	x		x		x	  x	  LP1	RP1	LPB	RPB
		//			  128	64	32  16	8	  4	  2 	1
		
		wall_reflex_information = ( (reflex_bool * 64) + (left_wall_counter * 4) + (right_wall_counter) );
		
		//	WRI =	!   !	      REFLEX	x	  LW1 LW0 RW1 RW0
		//			  128	64	    32	    16	8	  4	  2	  1
		
		//Förhindra avbrott under uppdateringen - höj avbrottsnivån så inga bussavbrott kommer.
		cli();
	
		send_buffer[0] = angle_right;
		send_buffer[1] = offset_right;
		send_buffer[2] = angle_left;
		send_buffer[3] = offset_left;
		send_buffer[4] = front_sensor;
		send_buffer[5] = can_see_information;
		send_buffer[6] = wall_reflex_information;
		
		sei();
		
		//Skicka till Styrmodul via SPI
	}			
}


ISR(SPI_STC_vect)
{
	int8_t inval_ = (int8_t)SPDR;
	
	SPDR = send_buffer[inval_];
}



/*
ADMUX 0 = Kort vänster bak
ADMUX 1 = Kort vänster fram
ADMUX 2 = Lång vänster
ADMUX 3 = Kort höger bak
ADMUX 4 = Kort höger fram
ADMUX 5 = Lång höger
ADMUX 6 = Lång framåt
ADMUX 7 = Reflex
*/
