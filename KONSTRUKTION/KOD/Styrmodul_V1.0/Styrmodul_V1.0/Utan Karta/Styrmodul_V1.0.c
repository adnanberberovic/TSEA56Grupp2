/*
 * Styrmodul_0.4.c
 * Date: 2015-12-05
 * Author: Grupp 2 TSEA56 2015
 */ 

#define F_CPU 20000000UL

#include <stdlib.h>
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
//#include "Map.h"
//#include <avr/pgmspace.h>

#define AUTONOM_MODE (((PIND) & (1<<PIND3)) >> 3)
#define MANUAL_MODE (!AUTONOM_MODE)


// GLOBAL VARIABLES --------------------------------------------------------------------
uint8_t arrSpeed[] = {0,0,1,1,0}; //speedLeft, SpeedRight, DirLeft, DirRight, grip
int8_t arrSensor[] = {-1,0,0,0,0,0,0}; //angle, offset, front_sensor, can_see_information, wall_reflex_information
uint8_t arrSpeedout[] = {0,0,0}; //Speedleft, speedRight, dirleft/right
// One overflow corresponds to 13.1 ms if the F_CPU is 20 MHz and the defined prescaler.
// This timer will reset every time it reaches 10.
uint8_t TIMER_overflows;
// This timer will NOT reset every time it reaches 10.
int16_t TIMER_wheel, TIMER_gyro;
int16_t TIMER_PD = 0;
	// The following overflow storage corresponds to 131ms. This timer will never reset.
uint8_t TIMER_overflows_deci;
// This variable is used to store robots angle in degrees
float rotation_angle = 0.00;
// Checks if flags are correctly set in manual/auto loop.
int Manual_Flag = 0;
int speed_var = 200;
// --------------------------------------------------------------------------------------

// Setup data direction registers @ ports for out/inputs.
void Styr_InitPortDirections(void)
{
	DDRA = 1<<DDA0 | 1<<DDA1 | 1<<DDA2 | 1<<DDA3 | 1<<DDA4 | 1<<DDA5 | 1<<DDA6 | 1<<DDA7;
	DDRB = 1<<DDB0 | 1<<DDB1 | 1<<DDB2 | 1<<DDB3 | 1<<DDB4 | 1<<DDB5 | 1<<DDB7;
	DDRC = 1<<DDC0 | 1<<DDC7; 
	DDRD = 1<<DDD0 | 1<<DDD1 | 0<<DDD2 | 0<<DDD3 | 1<<DDD4 | 1<<DDD5 | 1<<DDD6 | 1<<DDD7;
	DDRD &= ~(1 << DDD3);
	// 0 = input
	// 1 = output
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
unsigned char SPI_MasterTransmit(uint8_t cData, char target)
{
	if (target == 'k') // K as in kommunikation
	{
		PORTB &= ~(1<<PORTB4);
	}
	else if (target == 's')	// S as in sensor
	{
		PORTB &= ~(1<<PORTB3);
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
		arrSpeedout[1] = (uint8_t)speed;
		OCR2B = speed;
	}
}
void PWM_SetSpeedLeft(int speed)
{
	if (speed >= 0 && speed <= 255)
	{
		arrSpeedout[0] = (uint8_t)speed;
		OCR2A = speed;
	}
}
// 1 for forward, 0 for backward
void PWM_SetDirLeft(int dir)
{
	arrSpeedout[2] &= 0b00000010;
	arrSpeedout[2] += (uint8_t)dir;
	if (dir == 1)
	{
		PORTD &= ~(1 << PORTD0);
	}
	else if (dir == 0)
	{
		PORTD |= 1 << PORTD0;
	}
}
// 1 for forward, 0 for backward
void PWM_SetDirRight(int dir)
{
	
	arrSpeedout[2] &= 0b00000001;
	arrSpeedout[2] += 2 * (uint8_t)dir;
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
	TCCR0B = 1<<CS00 | 1<<CS01 | 0<<CS02; // Prescaler set to 64 --> 1 interrupt =  0.81875 ms
	TCNT0 = 0; // Initialize counter
	TIMSK0 = 1<<TOIE0; // Enable timer interrupts.
}

//----------------------------PWM----END-----------------------------

void Get_sensor_values() //Load all sensor values into sensor-array
{
	SPI_MasterTransmit(0,'s'); //value to be returned
	for (int8_t i = 1; i < 8; i++)
	{
		arrSensor[i - 1] = SPI_MasterTransmit(i,'s'); //load all 8 sensor values into sensor position [0-7]
	}
}
void Send_sensor_values() // Can combine with Get speed when in manual mode
{
	SPI_MasterTransmit(255,'k');
	for (int8_t i = 0; i < 6; i++)
	{
		SPI_MasterTransmit(arrSensor[i],'k');
	}
	if (MANUAL_MODE)
	{
		SPI_MasterTransmit((arrSensor[6] - 128),'k');
	}
	else SPI_MasterTransmit(arrSensor[6],'k');
}
void Get_speed_value()
{
	SPI_MasterTransmit(0b00000001, 'k');
	_delay_us(200);
	arrSpeed[0] = SPI_MasterTransmit(0x00,'k'); //Get left speed
	_delay_us(20);
	arrSpeed[1] = SPI_MasterTransmit(0x00,'k'); //Get right speed
	_delay_us(20);
	arrSpeed[2] = SPI_MasterTransmit(0x00,'k'); //Get left dir
	_delay_us(20);
	arrSpeed[3] = SPI_MasterTransmit(0x00,'k'); //Get right dir
	_delay_us(20);
	arrSpeed[4] = SPI_MasterTransmit(0x00,'k'); //Get gripclaw
}
void Send_speed_value()
{
	SPI_MasterTransmit(254,'k');
	for (int8_t i = 0; i < 3; i++)
	{
		SPI_MasterTransmit(arrSpeedout[i],'k');
	}
}
ISR(TIMER0_OVF_vect)
{
	TIMER_overflows++;
	TIMER_wheel++;
	TIMER_gyro++;
	TIMER_PD++;
	
	if (TIMER_overflows >= 10)
	{
		TIMER_overflows = 0;
		TIMER_overflows_deci++; 
	}
}

//----------------------------GYRO------------------------------------
unsigned char SPI_MasterTransmit_Gyro(unsigned char cData)
{
	// Load data into SPI data register.
	SPDR = cData;
	
	// Wait until transmission completes.
	while(!(SPSR & (1<<SPIF)));
	
	return SPDR;
}

void set_GyroSS_Low() // connect gyro
{
	PORTC &= ~(1<<PORTC0);
}
void set_GyroSS_High() // disconnect gyro
{
	PORTC |= (1<<PORTC0);
}
void Gyro_Init()
{

	SPCR &= ~(1<<DORD);
	int8_t high_byte;
	do{
		set_GyroSS_Low();
		SPI_MasterTransmit_Gyro(0b10010100); //Activate adc
		high_byte = SPI_MasterTransmit_Gyro(0x00); //Byte with EOC and Accepted instr. bit
		SPI_MasterTransmit_Gyro(0x00); //low byte
		set_GyroSS_High();
	} while ( (high_byte & 0b10000000) & !(high_byte& 0b00100000)); // IF EOC = 0 and acc.instr. = 1 we continue
	SPCR |= (1<<DORD);

}
void Gyro_StartConversion()
{
	int8_t high_byte;	
	SPCR &= ~(1<<DORD);
	do{
		set_GyroSS_Low();
		SPI_MasterTransmit_Gyro(0b10010100); //Activate adc, select angular rate 
		high_byte = SPI_MasterTransmit_Gyro(0x00); //Byte with EOC and Accepted instr. bit
		SPI_MasterTransmit_Gyro(0x00); //low byte
		set_GyroSS_High();

	} while (high_byte & 0b10000000); // IF  acc.instr. = 1 we continue
	SPCR |= (1<<DORD);

}
int16_t Gyro_PollResult()
{
	uint8_t high_byte, low_byte;
	uint16_t return_val;

	SPCR &= ~(1<<DORD);
	do{
		set_GyroSS_Low();
		SPI_MasterTransmit_Gyro(0b10000000); //Activate adc
		high_byte = SPI_MasterTransmit_Gyro(0x00); //Byte with EOC and Accepted instr. bit
		low_byte = SPI_MasterTransmit_Gyro(0x00); //low byte
		set_GyroSS_High();
	} while ((high_byte & 0b10000000) & !(high_byte & 0b00100000)); // IF EOC = 0 and acc.instr. = 1 we continue
	SPCR |= (1<<DORD);
	return_val = ((high_byte & 0x000f) << 8);
	return_val = return_val + low_byte;
	return_val = return_val >> 1;
	return return_val;
}

// Converts the adc reading to angles per second
int16_t adcToAngularRate(uint16_t adcValue)
{
	double AngularRate = (adcValue * 25/12)+400;  // in mV
	// from the data sheet, R2 gyroscope sensor version is 6,67 mV/deg
	// for gyroscope with max angular rate 300 deg/s
	//LCD_SetPosition(16);
	//LCD_display_uint16((uint16_t)AngularRate);
	int16_t retval_= (AngularRate - 2500)/6.67;
	return retval_;
}
void Gyro_Sleep()
{
	SPCR &= ~(1<<DORD);
	int8_t dataH;
	do {
		set_GyroSS_Low();
		SPI_MasterTransmit_Gyro(0b10010000);
		dataH = SPI_MasterTransmit_Gyro(0x00);
		SPI_MasterTransmit_Gyro(0x00);
		set_GyroSS_High();
	} while (dataH & 0b10000000);
	SPCR |= (1<<DORD);
}
int16_t Gyro_sequence()
{
	int16_t result = 0;
	Gyro_StartConversion();
	result = Gyro_PollResult();
	result = adcToAngularRate(result);
	return result;
}

// use in function MOTOR_RotateLeft()
// delay untill 90 degrees reached;
void checkLeftAngle(float target_angle)
{
	int16_t result=0, sum;
	float interval, medel;
	int speed_var_local = speed_var;
	do {
		sum=0;
		//start_time = TIMER_gyro;
		TIMER_gyro = 0; // ** starta om tiden
		
		for (int i=0; i<20; i++)  //for (int i=1; i<21; i++) 
		{
			result = Gyro_sequence();
			//if (result > 0) // ignore positive results
			//{
				//result = 0;
				//i--;
			//}
			sum += result;
			//medel = sum / i; // *****  Läggas utanför för att undvika float-fel? *****
		}
		
		medel = sum / 20;
		interval = (float)(TIMER_gyro)/1220.7; //(float)(TIMER_gyro - start_time)/1200;
		rotation_angle += medel*interval;
		if ((rotation_angle > ((target_angle - 60)*(255/speed_var))) && (speed_var_local > 70))
		{
			speed_var_local = speed_var_local * 0.95; //** eller liknande, för att minska hastigheten gradvis när den närmar sig färdig
			PWM_SetSpeedLeft(speed_var_local);
			PWM_SetSpeedRight(speed_var_local);
		}
	}while (rotation_angle < (target_angle - 2));
	rotation_angle = 0.00; //reset
	//TIMER_gyro = 0;
	//for (int i=0; i<100; i++)
	//{
		//result = Gyro_sequence();
		//sum += result;
	//}
	//medel = sum / 100;
	//interval = (float)(TIMER_gyro)/1220.7; //(float)(TIMER_gyro - start_time)/1200;
	//rotation_angle += medel*interval;
	//LCD_Clear();
	//LCD_SetPosition(0);
	//LCD_display_int16((int16_t)rotation_angle);

}

// use in function MOTOR_RotateRight()
// delay untill 90 degrees reached;
void checkRightAngle(float target_angle)
{
	int16_t result=0, sum;
	float interval, medel;
	int speed_var_local = speed_var;
	do {
		sum=0;
		//start_time = TIMER_gyro;
		TIMER_gyro = 0; // ** starta om tiden
		
		for (int i=0; i<20; i++)  //for (int i=1; i<21; i++) 
		{
			result = Gyro_sequence();
			//if (result > 0) // ignore positive results
			//{
				//result = 0;
				//i--;
			//}
			sum += result;
			//medel = sum / i; // *****  Läggas utanför för att undvika float-fel? *****
		}
		
		medel = sum / 20;
		interval = (float)(TIMER_gyro)/1220.7; //(float)(TIMER_gyro - start_time)/1200;
		rotation_angle += medel*interval;
		if ((rotation_angle < (-(target_angle - 60)*(255/speed_var))) && (speed_var_local > 70))
		{
			speed_var_local = speed_var_local * 0.95; //** eller liknande, för att minska hastigheten gradvis när den närmar sig färdig
			PWM_SetSpeedLeft(speed_var_local);
			PWM_SetSpeedRight(speed_var_local);
		}
	} while (rotation_angle > -(target_angle - 6)); // -6 due to delay from stop of loop until wheels stop and slideing
	
	rotation_angle = 0; //reset
}

//----------------------------GYRO----END-----------------------------

//__________________________SPEEDOMETER_______________________________
uint16_t speed_send;
uint16_t speed_send_new;
uint16_t ss1;
uint16_t ss2;
float current_speed;
float  wheel_circumference = 20.4/4; //Wheel circumference is 204 mm. 4 black sectors
float time_difference;
uint8_t wheel_counter;


void Speed_Interrupt_Init()
{
	EICRA = 1<< ISC00 | 0 <<ISC01; //INT0 genererar avbrott p? båda flanker
	EIMSK = 1<< INT0;
}

ISR(INT0_vect)
{
	if ((PIND & 0b00000100) == 4)
	{
		PORTC &= ~(1 <<PORTC7);
	}
	else
	{
		PORTC |= 1<<PORTC7; //Increase reference voltage
		
// 		ss2 = ss1;
// 		ss1 = speed_send;
// 		//Decrease reference voltage
// 		time_difference = (float)TIMER_wheel / 0.8175 /1000;
// 		// div by 0.8175 -> time in ms
// 		if (TIMER_wheel > 20)
// 		{
// 			current_speed = wheel_circumference / time_difference;
// 			speed_send = (uint16_t)current_speed;
//			speed_send_new = (ss1+ss2+2*speed_send)/4;
// 			TIMER_wheel = 0;
// 		}
		wheel_counter++;
	}	
}
//__________________________SPEEDOMETER END____________________________

//--MOTOR start
void MOTOR_Forward(int speed)
{
	PWM_SetDirRight(1);
	PWM_SetDirLeft(1);
	PWM_SetSpeedLeft(speed);
	PWM_SetSpeedRight(speed);
}
void MOTOR_Backward(int speed)
{
	PWM_SetDirRight(0);
	PWM_SetDirLeft(0);
	PWM_SetSpeedLeft(speed);
	PWM_SetSpeedRight(speed);
}

//--MOTOR stop
void MOTOR_Stop()
{
	PWM_SetSpeedRight(0);
	PWM_SetSpeedLeft(0);
}

//--MOTOR rotate
void MOTOR_RotateLeft(float angle)
{
	PWM_SetDirRight(1);
	PWM_SetDirLeft(0);
	PWM_SetSpeedLeft(speed_var);
	PWM_SetSpeedRight(speed_var);
	checkLeftAngle(angle);
	MOTOR_Stop();
}
void MOTOR_RotateRight(float angle)
{
	PWM_SetDirRight(0);
	PWM_SetDirLeft(1);
	PWM_SetSpeedRight(speed_var);
	PWM_SetSpeedLeft(speed_var);
	checkRightAngle(angle);
	MOTOR_Stop();
}

//--Called to enable manual driving.
uint8_t offset__;
void MANUAL_DRIVE()
{
	Get_speed_value();
	Get_sensor_values();
	offset__ = arrSensor[1];
	PWM_SetSpeedLeft(arrSpeed[0]);
	PWM_SetSpeedRight(arrSpeed[1]);
	PWM_SetDirLeft(arrSpeed[2]);
	PWM_SetDirRight(arrSpeed[3]);
	if (arrSpeed[4] == 1)
	{
		SERVO_SetGrip();
	}
	else
	{
		SERVO_ReleaseGrip();
	}
	Send_sensor_values();
}

// ????
void autonom_get_send()
{
	Get_sensor_values();
	Send_sensor_values();
	Send_speed_value();
}

//_______________________REFLEX SENSOR AND WALL COUNTER_____________________________

uint8_t REFLEX_GetMarker()
{
	Get_sensor_values();
	uint8_t marker_ = ( (arrSensor[6]/64) && (0b00000001) );
	return marker_;
}

uint8_t FRONT_SENSOR_VALUE()
{
	Get_sensor_values();
	return arrSensor[4];
}

uint8_t PATH_AHEAD()
{
	if (FRONT_SENSOR_VALUE() < 40)
	{
		return 1;
	}
	else
	{
		return 0;	
	}
}

uint8_t WALL_AHEAD()
{
	if (FRONT_SENSOR_VALUE() > 80)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t WALL_CLOSE_AHEAD()
{
	if (FRONT_SENSOR_VALUE() > 100)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t LEFTPATHONE()
{
	Get_sensor_values();
	uint8_t canseewall_left = ((arrSensor[5])/8 & (0b00000001));
	return canseewall_left;
}

uint8_t RIGHTPATHONE()
{
	Get_sensor_values();
	uint8_t canseewall_right = ((arrSensor[5])/4 & (0b00000001));
	return canseewall_right;
}

uint8_t LEFTPATHBOTH()
{
	Get_sensor_values();
	uint8_t canseepath_left = ((arrSensor[5]/2) & (0b00000001));
	return canseepath_left;
}

uint8_t RIGHTPATHBOTH()
{
	Get_sensor_values();
	uint8_t canseepath_right = ((arrSensor[5]) & (0b00000001));
	return canseepath_right;
}

uint8_t PATHCOUNT_Left()
{
	Get_sensor_values();
	uint8_t path_left = ((arrSensor[6]/4) & (0b00000011));
	return path_left;
}

uint8_t PATHCOUNT_Right()
{
	Get_sensor_values();
	uint8_t path_right = (arrSensor[6] & (0b00000011));
	return path_right;
}


//_____________________REFLEX SENSOR AND WALL COUNTER END____________________________

//________________________________AUTOMATIC CONTROL_____________________________________

// Y = PD*G/(1+PD*G) * R

// Reference value = 0
// Error value e = r-y.
// We want to regulate the robot to drive in the middle of the corridor,
// So the reference value r should correspond to 20 cm.

int reference_ = 20;// Reference value for where we want to place the robot.
int offset_;		// Compared to reference.
int current_error_;	// Current error
int prev_error_ = 0;// Used to compare the errors when computing the "derivative".
int time_diff_;		// Useful for comparing time, used for the "derivative".
int time_new_ = 0;	// Useful for comparing time, used for the "derivative".
int prev_time_ = 0;	// Useful for comparing time, used for the "derivative".
int angle_ = 0;			// The angle of the robot with respect to the corridor.
int K_d = 1;		// K (constant) for D regulation.
int K_p = 1;		// K (proportion) for P regulation.
int standard_speed_ = 150;	// Keeps track of standard speed.
char control_mode = 'r';	/*if control_mode == r, then the robot will make a rapid angle
							 change in angle to steer itself towards the middle lane.
							 if control_mode == c, then the robot will carefully
							 steer itself in a forward direction within the middle lane.
							 The middle lane is around 4 cm wide according to us.
							*/
char discovery_mode = 'l';	/*if discovery_mode == r, then the robot will follow the 
							"right hand on the wall" rule, i.e. always make right turns
							whenever possible in junctions.
							for discovery_mode == l, the same goes as above but left 
							instead of right.
							for discovery_mode == f, the robot will just keep moving
							forward.
							if discovery_mode == ?, then the robot will take a random
							path through a junction.
							Priority order: r -> f -> l.
							*/
char resque_mode = 'd'; /*if resque_mode == d, then the robot will search according to
						discovery_mode. 
						if resque_mode == q, then the robot has found the mark and will
						start resquing.
						*/

// P-control function
int P_Control()
{
	int newSignal_P = K_p*(current_error_); // Calculate regulated value
	return newSignal_P;
}
// D-control function
int D_control()
{
	time_new_ = TIMER_PD; // Retrieve the current time
	int newSignal_D = 0;
	time_diff_ = time_new_ - prev_time_;
	newSignal_D = K_d*(current_error_ - prev_error_)/time_diff_;
	TIMER_PD = 0;
	
	if(prev_error_ != current_error_)
	{
		prev_error_ = current_error_; // Save the current error for next computation
		prev_time_ = time_new_; // Save the current time for next use
	}
	return newSignal_D;   
	
}

int Front_Control()
{

		if(FRONT_SENSOR_VALUE() > 65 )
		{
			return 60;
		}
		else if(FRONT_SENSOR_VALUE() > 40)
		{
			return 80;
		}
		else 
		{
			return standard_speed_;
		}
}

int Side_Control()
{
	if (LEFTPATHONE() || RIGHTPATHONE())
	{
		return 100;
	}
	return standard_speed_;
}

// Total control.
int PD_Control()
{
	
	int newSignal;
	
	if(control_mode == 'r')
	{
		standard_speed_ = 100;
		
		if(FRONT_SENSOR_VALUE() > 45) //Slow down when approaching wall
		{
			standard_speed_ = Front_Control();
		}
		
		K_p = 6;
		K_d = 5;
		current_error_ = reference_ - offset_;
		if(offset_- 20 > 0)
		{
			newSignal = P_Control()+D_control()+angle_;
		}
		else
		{
			newSignal = P_Control()+D_control()-angle_;	
		}
		if(angle_ > 0) // On the right path to the middle.
		{
			newSignal=0;
		}
	}
	else if(control_mode == 'c')
	{
		standard_speed_ = 100;
		standard_speed_ = Side_Control();
		if(FRONT_SENSOR_VALUE() > 45) //Slow down when approaching wall
		{
		standard_speed_ = Front_Control();
		}
		
		K_p = 3;
		K_d = 2;
		if(offset_-20 > 0)
		{
			current_error_ = angle_;
		}
		else
		{
			current_error_ = -angle_;	
		}
		newSignal = P_Control() + D_control();
	}
	else
	{
		current_error_ = 0;
		newSignal = 0;
	}

	
	
	return newSignal;
}

// Performs a 180 degree turn in the event of a dead end.
void DEAD_END()
{	
	MOTOR_Stop();
	_delay_ms(100);
	
	if (offset_ - 20 > 0)
	{
		//Get_sensor_values();
		//int angle_left =  arrSensor[2];
		//MOTOR_RotateLeft(180- angle_left);
		MOTOR_RotateLeft(180 - angle_);
	}
	else
	{
		MOTOR_RotateRight(180 - angle_);
	}
	
	Get_sensor_values();
	if ( ( (arrSensor[1]  + arrSensor[3]) / 2) - 20 > 0)
	{
		if (arrSensor[0] < 0)
		{
			MOTOR_RotateLeft(- arrSensor[0]);
		}
		else
		{
			MOTOR_RotateRight(arrSensor[0]);
		}
	}
	else
	{
		if (arrSensor[2] < 0)
		{
			MOTOR_RotateRight(- arrSensor[2]);
		}
		else
		{
			MOTOR_RotateLeft(arrSensor[2]);
		}
	}
	
	_delay_ms(100);
	MOTOR_Forward(standard_speed_);
}

void JUNCTION_delay(int delay)
{
	wheel_counter = 0;
	while (wheel_counter < delay)
	{
		_delay_us(1);
	}
	
}

// Stops and rotates left 90 degrees.
void TURN_Right(int mode)
{
	MOTOR_Stop();
	_delay_ms(100);
	
	if (mode == 0)
	{
		Get_sensor_values();
		int angle_left =  arrSensor[2];
		MOTOR_RotateRight(90 - angle_left);
		_delay_ms(100);
		
		Get_sensor_values();
		angle_left =  arrSensor[2];
		
		if (angle_left < 0)
		{
			MOTOR_RotateRight(-angle_left);
		}
		else
		{
			MOTOR_RotateLeft(angle_left);
		}
		
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
		
	}
	else if ( mode == 1)
	{
		Get_sensor_values();
		int angle_left =  arrSensor[2];
		
		MOTOR_RotateRight(90 - angle_left);
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
	}
	else if ( mode == 2)
	{
		if (abs(angle_) < 10)
		{
			angle_ = angle_ / 2;
		}
		
		if( offset_ > 20)
		{
			MOTOR_RotateRight(90 - angle_);
		}
		else
		{
			MOTOR_RotateRight(90 + angle_);
		}
		
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
	}
		
	while((PATHCOUNT_Left() > 0) || (PATHCOUNT_Right() > 0))
		{
			_delay_us(1);
			LCD_SetPosition(1);
			LCD_SendString("Turn right");
			//Wait until robot reaches walls again
		}
	JUNCTION_delay(3);
}

// Stops and rotates right 90 degrees.
void TURN_Left(int mode)
{
	MOTOR_Stop();
	_delay_ms(100);
	
	if (mode == 0)
	{
		
		Get_sensor_values();
		int angle_right =  arrSensor[0];
		MOTOR_RotateLeft(90 - angle_right);
		_delay_ms(100);

		Get_sensor_values();
		angle_right =  arrSensor[0];
		
		if (angle_right < 0)
		{
			MOTOR_RotateLeft(-angle_right);
		}
		else
		{
			MOTOR_RotateRight(angle_right);
		}
	
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
	} 
	
	else if ( mode == 1)
	{
		Get_sensor_values();
		int angle_right =  arrSensor[0];
	
		MOTOR_RotateLeft(90 - angle_right);	
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
	}
	else if ( mode == 2)
	{
		
		if( offset_ > 20)
			{
				MOTOR_RotateLeft(90 - angle_);
			}
			else
			{
				MOTOR_RotateLeft(90 + angle_);
			}
				
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
	}
	
	while((PATHCOUNT_Left() > 0) || (PATHCOUNT_Right() > 0))
	{
		_delay_us(1);
		LCD_SetPosition(1);
		LCD_SendString("Turn left");
		//Wait until robot reaches walls again
	}
	JUNCTION_delay(3);
}

// Sets a random discovery mode to actually decide which way to go.
// Remember to reset the discovery mode back to '?' after calling this
// function and deciding on a turn.
void DISCOVERY_SetRandom()
{
	uint8_t random_mode = rand() % 3;
	if (random_mode == 0)
	{
		discovery_mode = 'r';
	}
	if (random_mode == 1)
	{
		discovery_mode = 'l';
	}
	else
	{
		discovery_mode = 'f';
	}
}


// One of the following three methods are called in the event of a 
// three way junction described by the ASCII art above each method.
/*  |
  ->--- ONE is used in junctions of this type.
*/
void JUNCTION_ThreeWayONE()
{
	if ((discovery_mode == 'r') || (discovery_mode == 'f'))
	{
	
		
		// Keep going forward
		while(PATHCOUNT_Left() > 0 )
		{
			Get_sensor_values();
			angle_ =  arrSensor[0];
			offset_ =  arrSensor[1];

			if(abs(offset_-20) <= 2)
			{
				control_mode = 'c';
				LCD_Clear();
				LCD_SetPosition(8);
				LCD_SendString("Mode: c");
			}
			// Puts the automatic control in rapid mode, push the robot to the middle lane.
			else
			{
				control_mode = 'r';
				LCD_Clear();
				LCD_SetPosition(8);
				LCD_SendString("Mode: r");
			}
		
			int new_speed_ = PD_Control();

			// Makes sure that the motors don't burn out (i.e go on max velocity)
			if(new_speed_ > (254-standard_speed_))
			{
				new_speed_ = 254 - standard_speed_;
			}
			else
			{
				PWM_SetSpeedRight(standard_speed_ + new_speed_);
				PWM_SetSpeedLeft(standard_speed_ - new_speed_);		
			}
				_delay_us(1);
				LCD_SetPosition(1);
				LCD_SendString("Threeway One");
			}
			JUNCTION_delay(3);
	}
	else if (discovery_mode == 'l')
	{
		// Turn left
		TURN_Left(1);
	}
	else if (discovery_mode == '?')
	{
		DISCOVERY_SetRandom();
		JUNCTION_ThreeWayONE();
		discovery_mode = '?';
	}
}

/*  v
  ----- TWO is used in junctions of this type.
*/
void JUNCTION_ThreeWayTWO()
{
	if ((discovery_mode == 'l') || (discovery_mode == 'f'))
	{
		//Turn Left. Forward becomes left due to right-forward-left cycle.
		//JUNCTION_delay(3);
		TURN_Left(0);
	}
	else if (discovery_mode == 'r')
	{
		// Turn right
		while(!WALL_CLOSE_AHEAD())
		{
			_delay_us(1);
		}
		TURN_Right(0);
	}
	else if (discovery_mode == '?')
	{
		DISCOVERY_SetRandom();
		JUNCTION_ThreeWayTWO();
		discovery_mode = '?';
	}
}
/*  |
  ---<- THREE is used in junctions of this type.
*/
void JUNCTION_ThreeWayTHREE()
{	
	if (discovery_mode == 'r')
	{
		// Turn right
		//JUNCTION_delay(3);
		TURN_Right(1);
	}
	else if ((discovery_mode == 'f') || (discovery_mode == 'l'))
	{
		// Keep going forward
		while(PATHCOUNT_Right() > 0 )
			{
				Get_sensor_values();
				angle_ =  arrSensor[2];
				offset_ =  arrSensor[3];

				if(abs(offset_-20) <= 2)
				{
					control_mode = 'c';
					LCD_Clear();
					LCD_SetPosition(8);
					LCD_SendString("Mode: c");
				}
				// Puts the automatic control in rapid mode, push the robot to the middle lane.
				else
				{
					control_mode = 'r';
					LCD_Clear();
					LCD_SetPosition(8);
					LCD_SendString("Mode: r");
				}
				
				int new_speed_ = PD_Control();

				// Makes sure that the motors don't burn out (i.e go on max velocity)
				if(new_speed_ > (254-standard_speed_))
				{
					new_speed_ = 254 - standard_speed_;
				}
				else
				{
					PWM_SetSpeedRight(standard_speed_ + new_speed_);
					PWM_SetSpeedLeft(standard_speed_ - new_speed_);
				}
				_delay_us(1);
				LCD_SetPosition(1);
				LCD_SendString("Threeway Three");
			}
		JUNCTION_delay(3);
	}
		
	else if (discovery_mode == '?')
	{
		DISCOVERY_SetRandom();
		JUNCTION_ThreeWayTHREE();
		discovery_mode = '?';
	}	
}

// Called in the event of a four way junction.
void JUNCTION_FourWay()
{
	if(discovery_mode == 'r')
	{
		//turn right
		//JUNCTION_delay(3);
		TURN_Right(2);
	}
	else if(discovery_mode == 'l')
	{
		//turn left
		//JUNCTION_delay(3);
		TURN_Left(2);
	}
	else if(discovery_mode == 'f')
	{
		//keep going forward
		while ((LEFTPATHONE() || RIGHTPATHONE())
				 || ((PATHCOUNT_Left() > 0) && (PATHCOUNT_Right() > 0)))
		{
			_delay_us(1);
			LCD_SetPosition(1);
			LCD_SendString("Fourway");
		}
	}
	else if(discovery_mode == '?')
	{
		DISCOVERY_SetRandom();
		JUNCTION_FourWay();
		discovery_mode = '?';
	}
}

// Call this function to perform automatic control.

/*
void AutomaticControl()
{
	TIMER_PD = 0;
	Get_sensor_values();
	
	angle_ = arrSensor[0];
	offset_ = arrSensor[1];
	 
	// Junction, turn, and dead end handling.
	
	// Turn 180 deg in a dead end.
	if( !(LEFTPATHBOTH() || RIGHTPATHBOTH()) && WALL_AHEAD() && (PATHCOUNT_Left() == 0) && (PATHCOUNT_Right() == 0) )
		{
			DEAD_END();
		}
		
	else if ((LEFTPATHBOTH() || RIGHTPATHBOTH()) && WALL_AHEAD())
		{
			if( ((LEFTPATHBOTH() && RIGHTPATHONE()) || (RIGHTPATHBOTH() && LEFTPATHONE())) 
				&& (PATHCOUNT_Left() > 0) && (PATHCOUNT_Right() > 0) )
				//Denna if-sats ser till att vi förstår att det är en trevägs även om robot är sned
				{
					//  v
					//----- TWO is used in junctions of this type.
					//
					JUNCTION_ThreeWayTWO();
				}
			
				// Turn right in a corner.
				else if(RIGHTPATHBOTH() && !LEFTPATHONE() && WALL_AHEAD() 
						&& (PATHCOUNT_Left() == 0) && (PATHCOUNT_Right() > 0))
					{
						TURN_Right();
					}
			
				// Turn left in a corner.
				else if(!RIGHTPATHONE() && LEFTPATHBOTH() && WALL_AHEAD() 
						&& (PATHCOUNT_Left() > 0) && (PATHCOUNT_Right() == 0))
					{
						TURN_Left();
					}
		}
		
	else if((LEFTPATHBOTH() || RIGHTPATHBOTH()) && PATH_AHEAD()
			&& (PATHCOUNT_Left() > 0) && (PATHCOUNT_Right() > 0))
			{
				// Make a decision based on discovery mode upon entering a 4-way junction.
				if( (LEFTPATHBOTH() && RIGHTPATHONE()) || (RIGHTPATHBOTH() && LEFTPATHONE()) )
					//Denna if-sats ser till att vi förstår att det är en fyrvägs även om robot är sned
					{
						JUNCTION_FourWay();
					}
					
					// Three 3-way junction modes, decision depends on entrance to junction
					// and discovery mode.
		
							//  |
							//->--- ONE is used in junctions of this type.
							//
					else if(!RIGHTPATHBOTH() && LEFTPATHBOTH() && (PATHCOUNT_Left() > 0) && (PATHCOUNT_Right() == 0))
						{
							JUNCTION_ThreeWayONE();
						}
					
							//  |
						    //---<- THREE is used in junctions of this type.
							//
					else if(RIGHTPATHBOTH() && !LEFTPATHBOTH() && (PATHCOUNT_Left() == 0) && (PATHCOUNT_Right() > 0))
						{			
							JUNCTION_ThreeWayTHREE();
						}
			}
			
	
	// Puts the automatic control in careful mode, keep the robot on track.
	if(abs(offset_-20) <= 3)
	{
		control_mode = 'c';
		LCD_SetPosition(8);
		LCD_SendString("Mode: c");
	}
	// Puts the automatic control in rapid mode, push the robot to the middle lane.
	else
	{
		control_mode = 'r';
		LCD_SetPosition(8);
		LCD_SendString("Mode: r");
	}
	int new_speed_ = PD_Control();
	LCD_SendString("   ");
	// Makes sure that the motors don't burn out (i.e go on max velocity)
	if(new_speed_ > (254-standard_speed_))
	{
		new_speed_ = 254 - standard_speed_;
	}
	else
	{
		PWM_SetSpeedRight(standard_speed_ + new_speed_);
		PWM_SetSpeedLeft(standard_speed_ - new_speed_);		
	}
} 
*/
//________________________________AUTOMATIC CONTROL END_____________________________________
void AutomaticControl()
{

	if (REFLEX_GetMarker())
	{
		LCD_SetPosition(10);
		resque_mode = 'q';
		LCD_SendString("GOLD");
		JUNCTION_delay(2);
		DEAD_END();
		DEAD_END();
		JUNCTION_delay(2);
	}
	
	if( (PATHCOUNT_Left() > 0) || (PATHCOUNT_Right() > 0) ) //Path to left or right
	{
		
		while (!LEFTPATHBOTH() && !RIGHTPATHBOTH() && !WALL_CLOSE_AHEAD()) //Keep going until center of intersect
		{
			_delay_us(250);
		} 
		// Now in intersect. Determine what type:
		LCD_Clear();
		JUNCTION_delay(2);
			
		if ((PATHCOUNT_Left() > 0) && (PATHCOUNT_Right() > 0)) // 4-way or 3-way-2
		{
			if (PATH_AHEAD()) // 4-way
			{
				JUNCTION_FourWay();
			}
			else // 3-way-2
			{
				JUNCTION_ThreeWayTWO();
			}
		}
		else if (PATHCOUNT_Right() > 0) // 3-way-3 or RightTurn
		{
			if (PATH_AHEAD()) // 3-w-3
			{
				JUNCTION_ThreeWayTHREE();
			}
			else
			{
				TURN_Right(0); //Right turn
			}
		}
		else if (PATHCOUNT_Left() > 0) // 3-way-1 or Left turn
		{
			if (PATH_AHEAD()) // 3-way-1
			{
				JUNCTION_ThreeWayONE();
		    }
			else // Left turn
			{
				TURN_Left(0);
			}
		}
		else if (WALL_CLOSE_AHEAD())
		{
			DEAD_END();
		}
	}
	else if (!( LEFTPATHONE() || RIGHTPATHONE()))
	{
		Get_sensor_values();
		
		if( ((arrSensor[1] + arrSensor[3]) / 2) < 20)
		{
			angle_ =  arrSensor[2];
			offset_ =  arrSensor[3];
		}
		else
		{
			angle_ =  arrSensor[0];
			offset_ =  arrSensor[1];
		}
		
		
		
		if(abs(offset_-20) <= 2)
		{
			control_mode = 'c';
			LCD_Clear();
			LCD_SetPosition(8);
			LCD_SendString("Mode: c");
		}
		// Puts the automatic control in rapid mode, push the robot to the middle lane.
		else
		{
			control_mode = 'r';
			LCD_Clear();
			LCD_SetPosition(8);
			LCD_SendString("Mode: r");
		}
		
		int new_speed_ = PD_Control();
		LCD_SendString("   ");
		// Makes sure that the motors don't burn out (i.e go on max velocity)
		if(new_speed_ > (254-standard_speed_))
		{
			new_speed_ = 254 - standard_speed_;
		}
		else
		{
			PWM_SetSpeedRight(standard_speed_ + new_speed_);
			PWM_SetSpeedLeft(standard_speed_ - new_speed_);		
		}
	}
	
	if(WALL_CLOSE_AHEAD())
	{
		DEAD_END();
		LCD_Clear();
		LCD_SetPosition(0);
		LCD_SendString("Mode: FUCKTARD");
	}
}
// This method is very important to call at the start of the program.
void INIT_ALL()
{
	sleep_enable();	// Enable sleep instruction
	Styr_InitPortDirections();	// Initiate Port directions for the styrmodul.
	Styr_InitPortValues();	// Initiate Port Values for the styrmodul.
	SPI_MasterInit();	// Initiate the styrmodul as the SPI master.
	LCD_Init();	// Initiate the LCD.
	PWM_Init();	// Initiate PWM for motor
	TIMER_init();	// Initiate Timer settings
	LCD_WelcomeScreen();	// Welcomes the user with a nice message ;^)
	Gyro_Init();	// Initiate gyro settings
	Speed_Interrupt_Init(); //KOMMENTERA IN, MEN FUNGERAR EJ ATT MANUELLSTYRA DÅ 
	PWM_SetDirLeft(1);
	PWM_SetDirRight(1);
	sei();	// Enable global interrupts
}

int main(void)
{
	INIT_ALL();
	
   	while (1)
	   {
    		_delay_ms(10);
    		LCD_Clear();
    		LCD_SetPosition(2);
    		LCD_SendString("AUTOMATIC_MODE");
  			MOTOR_Stop();
  			LCD_Clear();
    		while(AUTONOM_MODE)
    		{
    			AutomaticControl();
			
    			LCD_SetPosition(0);
				LCD_SendString("RS:");
				LCD_display_uint16(OCR2B);
				LCD_SendString("  ");
				LCD_SetPosition(16);
				LCD_SendString("LS:");
				LCD_display_uint16(OCR2A);
				LCD_SendString("  ");
				_delay_ms(50);
    		}
    		
    		_delay_ms(10);
    		LCD_Clear();
    		LCD_SetPosition(2);
    		LCD_SendString("MANUAL_MODE");
			LCD_Clear();

    		while(MANUAL_MODE)
    		{
				//LCD_Clear();
    			MANUAL_DRIVE();
				LCD_SetPosition(0);
				LCD_SendString("REflex: ");
    			LCD_display_uint8(REFLEX_GetMarker());
				LCD_SendString("   resq:");
				LCD_SendCharacter(resque_mode);
				PWM_SetSpeedLeft(0);
    			PWM_SetSpeedRight(0);
				LCD_SetPosition(16);
				LCD_SendString("PCL:");
    			LCD_display_uint8(PATHCOUNT_Left());
				LCD_SendString(" PCR:");
				LCD_display_uint8(PATHCOUNT_Right());
    		}
    		PWM_SetSpeedLeft(0);
    		PWM_SetSpeedRight(0);
  	}
}