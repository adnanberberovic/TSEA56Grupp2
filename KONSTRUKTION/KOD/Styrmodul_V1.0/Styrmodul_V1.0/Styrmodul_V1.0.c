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
#include "Map.c"
//#include "Path.c"
//#include <avr/pgmspace.h>

#define AUTONOM_MODE (((PIND) & (1<<PIND3)) >> 3)
#define MANUAL_MODE (!AUTONOM_MODE)


// _________________________GLOBAL VARIABLES_________________________
uint8_t arrSpeed[] = {0,0,1,1,0}; //speedLeft, SpeedRight, DirLeft, DirRight, grip
int8_t arrSensor[] = {-1,0,0,0,0,0,0}; //angle, offset, front_sensor, can_see_information, wall_reflex_information
uint8_t arrSpeedout[] = {0,0,0}; //Speedleft, speedRight, dirleft/right
int16_t TIMER_gyro;
int16_t TIMER_PD = 0;
// This variable is used to store robots angle in degrees
float rotation_angle = 0.00;
// Checks if flags are correctly set in manual/auto loop.
int Manual_Flag = 0;
int speed_var = 200;
uint8_t distance_flag = 0;

uint8_t holds_item = 0;
uint8_t arrMap[3] = {0, 0, 0}; //[0] = {xpos(4) + dir(2)}, [1] = {y-pos(5) + m?l(1)}, [2] = {left(2) + ahead(2) + right(2)}


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

volatile uint16_t wheel_counter;
volatile uint16_t distance_counter = 0; // distance_counter = 1 <=> 12,37 mm
// ett hjulvarv = 63*pi = 197 mm => distance_counter = 16


uint8_t LeftPathBoth;
uint8_t LeftPathOne;
uint8_t RightPathBoth;
uint8_t RightPathOne;
uint8_t PathCountLeft;
uint8_t PathCountRight;
uint8_t PathAhead;
uint8_t WallAhead;
uint8_t WallCloseAhead;
uint8_t FrontSensorValue;

//__________________________REGISTERS__________________________

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

//__________________________PWM__________________________

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

//__________________________SENSOR VALUES__________________________

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
void Send_map_values(){
	uint8_t posX_ = MAP_currentPos[1];
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t first_byte = posX_ * 8; //set X-pos to 5 MSB bits
	first_byte = first_byte + MAP_currentDir; //set dir to 2 lsb bits
	
	uint8_t second_byte = posY_ * 8; // set Y-pos to 5 MSB bits
	if ((MAP_goalPosition[0] == MAP_currentPos[0]) && (MAP_goalPosition[1] == MAP_currentPos[1])) //Sends goal-bit if goal
	{
		second_byte = second_byte + 1;
	}
	else
	{
		second_byte = second_byte + 0;
	}
	
	uint8_t third_byte = 0;
	if (MAP_currentDir == 0) // right in map
	{
		third_byte = third_byte + (MAP_array[posY_ - 1][posX_].description - 2) * 64; //Up in map, left for bot
		third_byte = third_byte + (MAP_array[posY_][posX_ + 1].description - 2) * 16; //right in map, forward for bot
		third_byte = third_byte + (MAP_array[posY_ + 1][posX_].description - 2) * 4; // down in map, right for bot
	}
	else if (MAP_currentDir == 1) // up in map
	{
		third_byte = third_byte + (MAP_array[posY_][posX_ - 1].description - 2) * 64; //Left in map and bot
		third_byte = third_byte + (MAP_array[posY_ - 1][posX_].description - 2) * 16; // up in map and forward for bot
		third_byte = third_byte + (MAP_array[posY_][posX_ + 1].description - 2) * 4; // right in map and bot
	}
	else if (MAP_currentDir == 2) // left in map
	{
		third_byte = third_byte + (MAP_array[posY_ + 1][posX_].description - 2) * 64; // down in map, left for bot
		third_byte = third_byte + (MAP_array[posY_][posX_ - 1].description - 2) * 16; // left in map, forward for bot
		third_byte = third_byte + (MAP_array[posY_ - 1][posX_].description - 2) * 4; // up in map, right for bot

	}
	else if (MAP_currentDir == 3)  // down in map
	{
		third_byte = third_byte + (MAP_array[posY_][posX_ + 1].description - 2) * 64; // right in map, left for bot
		third_byte = third_byte + (MAP_array[posY_ + 1][posX_].description - 2) * 16; // down in map, forward for bot
		third_byte = third_byte + (MAP_array[posY_][posX_ - 1].description - 2)  * 4; //left in map, right for bot
	}
	
	SPI_MasterTransmit(69,'k');
	SPI_MasterTransmit(first_byte,'k');
	SPI_MasterTransmit(second_byte,'k');
	SPI_MasterTransmit(third_byte,'k');
}

//__________________________TIMERS__________________________
ISR(TIMER0_OVF_vect)
{
	TIMER_gyro++;
	TIMER_PD++;
}

//__________________________GYRO__________________________
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

// Delay untill 90 degrees reached;
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
			sum += result;
		}
		
		medel = sum / 20;
		interval = (float)(TIMER_gyro)/1220.7;
		rotation_angle += medel*interval;
		if ((rotation_angle > ((target_angle - 60)*(255/speed_var))) && (speed_var_local > 70))
		{
			speed_var_local = speed_var_local * 0.95;
			PWM_SetSpeedLeft(speed_var_local);
			PWM_SetSpeedRight(speed_var_local);
		}
	}while (rotation_angle < (target_angle - 2));
	rotation_angle = 0.00; //Reset
}

// Delay untill 90 degrees reached;
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
			//medel = sum / i; // *****  L?ggas utanf?r f?r att undvika float-fel? *****
		}
		
		medel = sum / 20;
		interval = (float)(TIMER_gyro)/1220.7; //(float)(TIMER_gyro - start_time)/1200;
		rotation_angle += medel*interval;
		if ((rotation_angle < (-(target_angle - 60)*(255/speed_var))) && (speed_var_local > 70))
		{
			speed_var_local = speed_var_local * 0.95; //** eller liknande, f?r att minska hastigheten gradvis n?r den n?rmar sig f?rdig
			PWM_SetSpeedLeft(speed_var_local);
			PWM_SetSpeedRight(speed_var_local);
		}
	} while (rotation_angle > -(target_angle - 6)); // -6 due to delay from stop of loop until wheels stop and slideing
	
	rotation_angle = 0; //reset
}

//__________________________DISTANCE COUNTER_______________________________

void Speed_Interrupt_Init()
{
	EICRA = 1<< ISC00 | 0 <<ISC01; //INT0 genererar avbrott p? b?da flanker
	EIMSK = 1<< INT0;
}

ISR(INT0_vect)
{
	wheel_counter++;
	distance_counter++;
	EIFR = (1<<INTF0); //Clear queued interrupts
	
	if (distance_counter >= 32)
	{
		distance_flag = 1;
		distance_counter = 0;
	}
}

//__________________________MOTOR_______________________________

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
void autonom_get_send()
{
	Get_sensor_values();
	Send_sensor_values();
	Send_speed_value();
}

//_________________________________SENSOR VALUES_______________________________________

uint8_t REFLEX_GetMarker()
{
	Get_sensor_values();
	uint8_t marker_ = ( (arrSensor[6]/64) && (0b00000001) );
	return marker_;
}

uint8_t FRONT_SENSOR_VALUE()
{
	Get_sensor_values();
	FrontSensorValue = arrSensor[4];
	return FrontSensorValue;
}

uint8_t PATH_AHEAD()
{
	if (FRONT_SENSOR_VALUE() < 40)
	{
		PathAhead = 1;
		return PathAhead;
	}
	else
	{
		PathAhead = 0;
		return PathAhead;	
	}
}

uint8_t WALL_AHEAD()
{
	if (FRONT_SENSOR_VALUE() > 80)
	{
		WallAhead = 1;
		return WallAhead;
	}
	else
	{
		WallAhead = 0;
		return WallAhead;
	}
}

uint8_t WALL_CLOSE_AHEAD()
{
	if (FRONT_SENSOR_VALUE() > 95)
	{
		WallCloseAhead = 1;
		return 1;
	}
	else
	{
		WallCloseAhead = 0;
		return 0;
	}
}

uint8_t LEFTPATHONE()
{
	Get_sensor_values();
	LeftPathOne = ((arrSensor[5])/8 & (0b00000001));
	return LeftPathOne;
}

uint8_t RIGHTPATHONE()
{
	Get_sensor_values();
	RightPathOne = ((arrSensor[5])/4 & (0b00000001));
	return RightPathOne;
}

uint8_t LEFTPATHBOTH()
{
	Get_sensor_values();
	LeftPathBoth = ((arrSensor[5]/2) & (0b00000001));
	return LeftPathBoth;
}

uint8_t RIGHTPATHBOTH()
{
	Get_sensor_values();
	RightPathBoth = ((arrSensor[5]) & (0b00000001));
	return RightPathBoth;
}

uint8_t PATHCOUNT_Left()
{
	Get_sensor_values();
	PathCountLeft = ((arrSensor[6]/4) & (0b00000011));
	return PathCountLeft;
}

uint8_t PATHCOUNT_Right()
{
	Get_sensor_values();
	PathCountRight = (arrSensor[6] & (0b00000011));
	return PathCountRight;
}

void Update_All_FUCKING_values()
{
	Get_sensor_values();

	FrontSensorValue = arrSensor[4];
		
	if (FrontSensorValue < 40)
	{
		PathAhead = 1;
	}
	else
	{
		PathAhead = 0;	
	}
	if (FrontSensorValue > 80)
	{
		WallAhead = 1;
	}
	else
	{
		WallAhead = 0;
	}
	if (FrontSensorValue > 100)
	{
		WallCloseAhead = 1;
	}
	else
	{
		WallCloseAhead = 0;
	}
	

	LeftPathOne = ((arrSensor[5])/8 & (0b00000001));
	RightPathOne = ((arrSensor[5])/4 & (0b00000001));
	LeftPathBoth = ((arrSensor[5]/2) & (0b00000001));
	RightPathBoth = ((arrSensor[5]) & (0b00000001));
	PathCountLeft = ((arrSensor[6]/4) & (0b00000011));
	PathCountRight = (arrSensor[6] & (0b00000011));
	
}

//
// _______________________________ Update Map _______________

void set_map_right(int desc)
{
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t posX_ = MAP_currentPos[1];
	MAP_array[posY_][posX_ + 1].description = desc;
// 	if (MAP_array[posY_][posX_ + 1].description == 0)
// 	{
// 		MAP_array[posY_][posX_ + 1].description = desc;
// 	}
// 	else if (MAP_array[posY_][posX_ + 1].description == 4 && desc == 3)
// 	{
// 		MAP_array[posY_][posX_ + 1].description = desc;
// 	}
}

void set_map_up(int desc)
{
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t posX_ = MAP_currentPos[1];
	MAP_array[posY_ - 1][posX_].description = desc;
// 	if (MAP_array[posY_ - 1][posX_].description == 0)
// 	{
// 		MAP_array[posY_ - 1][posX_].description = desc;
// 	}
// 	else if (MAP_array[posY_ - 1][posX_].description == 4 && desc == 3)
// 	{
// 		MAP_array[posY_ - 1][posX_].description = desc;
// 	}
}

void set_map_left(int desc)
{
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t posX_ = MAP_currentPos[1];
	MAP_array[posY_][posX_ - 1].description = desc;
// 	if (MAP_array[posY_][posX_ - 1].description == 0)
// 	{
// 		MAP_array[posY_][posX_ - 1].description = desc;
// 	}
// 	else if (MAP_array[posY_][posX_ - 1].description == 4 && desc == 3)
// 	{
// 		MAP_array[posY_][posX_ - 1].description = desc;
// 	}
}

void set_map_down(int desc)
{
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t posX_ = MAP_currentPos[1];
	MAP_array[posY_ + 1][posX_].description = desc;
// 	if (MAP_array[posY_ + 1][posX_].description == 0)
// 	{
// 		MAP_array[posY_ + 1][posX_].description = desc;
// 	}
// 	else if (MAP_array[posY_ + 1][posX_].description == 4 && desc == 3)
// 	{
// 		MAP_array[posY_ + 1][posX_].description = desc;
// 	}
}

void set_map_FourWay()
{	
    if (MAP_currentDir == 0)
    {
        set_map_up(3);
        set_map_right(3);
        set_map_down(3);
    }
    else if (MAP_currentDir == 1)
    {
        set_map_left(3);
        set_map_right(3);
        set_map_up(3);
    }
    else if (MAP_currentDir == 2)
    {
        set_map_up(3);
        set_map_left(3);
		set_map_down(3);
    }
    else if (MAP_currentDir == 3)
    {
        set_map_left(3);
		set_map_right(3);
        set_map_down(3);
    }
}

void set_map_ThreeWay2()
{
    if (MAP_currentDir == 0)
    {
		set_map_up(3);
		set_map_right(4);
		set_map_down(3);
    }
    else if (MAP_currentDir == 1)
    {
		set_map_left(3);
		set_map_right(3);
		set_map_up(4);
    }
    else if (MAP_currentDir == 2)
    {
		set_map_up(3);
		set_map_left(4);
		set_map_down(3);
    }
    else if (MAP_currentDir == 3)
    {
		set_map_left(3);
		set_map_right(3);
		set_map_down(4);
    }
}

void set_map_ThreeWay3()
{
    if (MAP_currentDir == 0)
    {
		set_map_up(4);
		set_map_right(3);
		set_map_down(3);
    }
    else if (MAP_currentDir == 1)
    {
		set_map_left(4);
		set_map_right(3);
		set_map_up(3);
    }
    else if (MAP_currentDir == 2)
    {
		set_map_up(3);
		set_map_left(3);
		set_map_down(4);
    }
    else if (MAP_currentDir == 3)
    {
		set_map_left(3);
		set_map_right(4);
		set_map_down(3);
    }
}

void set_map_RightTurn()
{
    if (MAP_currentDir == 0)
    {
		set_map_up(4);
		set_map_right(4);
		set_map_down(3);
    }
    else if (MAP_currentDir == 1)
    {
		set_map_left(4);
		set_map_right(3);
		set_map_up(4);
    }
    else if (MAP_currentDir == 2)
    {
		set_map_up(3);
		set_map_left(4);
		set_map_down(4);
    }
    else if (MAP_currentDir == 3)
    {
		set_map_left(3);
		set_map_right(4);
		set_map_down(4);
    }
}

void set_map_ThreeWay1()
{
    if (MAP_currentDir == 0)
    {
		set_map_up(3);
		set_map_right(3);
		set_map_down(4);
    }
    else if (MAP_currentDir == 1)
    {
		set_map_left(3);
		set_map_right(4);
		set_map_up(3);
    }
    else if (MAP_currentDir == 2)
    {
		set_map_up(4);
		set_map_left(3);
		set_map_down(3);
    }
    else if (MAP_currentDir == 3)
    {
		set_map_left(4);
		set_map_right(3);
		set_map_down(3);
    }
}

void set_map_LeftTurn()
{
    if (MAP_currentDir == 0)
    {
		set_map_up(3);
		set_map_right(4);
		set_map_down(4);
    }
    else if (MAP_currentDir == 1)
    {
		set_map_left(3);
		set_map_right(4);
		set_map_up(4);
    }
    else if (MAP_currentDir == 2)
    {
		set_map_up(4);
		set_map_left(4);
		set_map_down(3);
    }
    else if (MAP_currentDir == 3)
    {
		set_map_left(4);
		set_map_right(3);
		set_map_down(4);
    }
}

void set_map_DeadEnd()
{
    if (MAP_currentDir == 0)
    {
		set_map_up(4);
		set_map_right(4);
		set_map_down(4);
    }
    else if (MAP_currentDir == 1)
    {
		set_map_left(4);
		set_map_right(4);
		set_map_up(4);
    }
    else if (MAP_currentDir == 2)
    {
	    set_map_up(4);
	    set_map_left(4);
	    set_map_down(4);
    }
    else if (MAP_currentDir == 3)
    {
	    set_map_left(4);
	    set_map_right(4);
	    set_map_down(4);
    }
}

void set_map_Corridor()
{
    if (MAP_currentDir == 0)
    {
	    set_map_up(4);
	    set_map_right(3);
	    set_map_down(4);
    }
    else if (MAP_currentDir == 1)
    {
	    set_map_left(4);
	    set_map_right(4);
	    set_map_up(3);
    }
    else if (MAP_currentDir == 2)
    {
	    set_map_up(4);
	    set_map_left(3);
	    set_map_down(4);
    }
    else if (MAP_currentDir == 3)
    {
	    set_map_left(4);
	    set_map_right(4);
	    set_map_down(3);
    }
}


//________________________________AUTOMATIC CONTROL_____________________________________
// Y = PD*G/(1+PD*G) * R

// Reference value = 0
// Error value e = r-y.
// We want to regulate the robot to drive in the middle of the corridor,
// So the reference value r should correspond to 20 cm.

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
							for discovery_mode == b, the robot will rotate 180 deg.
							if discovery_mode == ?, then the robot will take a random
							path through a junction.
							Priority order: r -> f -> l -> b.
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

// Decreases speed as robot approaches wall
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

// Decreases speed as robot enters junction
int Side_Control()
{
	if (LEFTPATHONE() || RIGHTPATHONE())
	{
		return 100;
	}
	return standard_speed_;
}

// Total control
int PD_Control()
{
	
	int newSignal;
	
	if(control_mode == 'r')
	{
		standard_speed_ = 80;
		
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
		
		K_p = 6;
		K_d = 4;
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

// Performs a 180 degree turn in the event of a dead end
void DEAD_END()
{	
	MOTOR_Stop();
	_delay_ms(100);
	
	if (offset_ - 20 > 0)
	{
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
	
	distance_counter = 0;
	distance_flag = 0;
}

void JUNCTION_delay(int delay)
{
	wheel_counter = 0;
	while (wheel_counter < (2 * delay))
	{
		_delay_us(50);
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
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	else if ( mode == 1)
	{
		Get_sensor_values();
		int angle_left =  arrSensor[2];
		
		MOTOR_RotateRight(90 - angle_left);
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	else if ( mode == 2)
	{
		if(angle_ > 5)
		{
			angle_ = 5;
		}
		else if (angle_ < -5)
		{
			angle_ = -5;
		}
		
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
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	while(PATHCOUNT_Right() > 0)
		{
			_delay_us(250);
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
		
		distance_counter = 0;
		distance_flag = 0;
	} 
	
	else if ( mode == 1)
	{
		Get_sensor_values();
		int angle_right =  arrSensor[0];
	
		MOTOR_RotateLeft(90 - angle_right);	
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	else if ( mode == 2)
	{
		if(angle_ > 5)
		{
			angle_ = 5;
		}
		else if (angle_ < -5)
		{
			angle_ = -5;
		}
		
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
		
		distance_counter = 0;
		distance_flag = 0;
	}

	while(PATHCOUNT_Left() > 0)
	{
		_delay_us(250);
	}
	
	JUNCTION_delay(3);
}

void TURN_Back(int mode)
{
	if (mode == 0) // 4-way
	{
		
		if(angle_ > 5)
		{
			angle_ = 5;
		}
		else if (angle_ < -5)
		{
			angle_ = -5;
		}
		
		if( offset_ > 20)
		{
			MOTOR_RotateLeft(180 - angle_);
		}
		else
		{
			MOTOR_RotateLeft(180 + angle_);
		}

		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	else if(mode == 1) // 3-way-1
	{
		Get_sensor_values();
		int angle_right =  arrSensor[0];
				
		MOTOR_RotateLeft(180 - angle_right);
		_delay_ms(100);
		
		Get_sensor_values();
		int angle_left = arrSensor[2];
		
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
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	else if(mode == 2) // 3-way-2
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
		
		distance_counter = 0;
		distance_flag = 0;
	}

	else if(mode == 3) // 3-way-3
	{
		MOTOR_Stop();
		Get_sensor_values();
		int angle_left =  arrSensor[2];
		
		MOTOR_RotateLeft(180 - angle_left);
		_delay_ms(100);
		
		Get_sensor_values();
		int angle_right = arrSensor[0];
		
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
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	else if(mode == 4) // corridor
	{
		if( offset_ > 20)
		{
			MOTOR_RotateLeft(180 - angle_);
		}
		else
		{
			MOTOR_RotateLeft(180 + angle_);
		}
		
		_delay_ms(100);
		MOTOR_Forward(standard_speed_);
		
		distance_counter = 0;
		distance_flag = 0;
	}
	
	
	
	while((PATHCOUNT_Left() > 0) || (PATHCOUNT_Right() > 0))
	{
		_delay_us(250);
		LCD_SetPosition(1);
		LCD_SendString("Turn back");
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

// Sets a mode based on current direction and next direction.
void DISCOVERY_SetMode()
{
	uint8_t mode = (4 + MAP_currentDir - MAP_nextDir) % 4;
	
	if(mode == 0)
	{
		discovery_mode = 'f';
	}
	else if(mode == 1)
	{
		discovery_mode = 'r';
	}
	else if(mode == 2)
	{
		discovery_mode = 'b';
	}
	else if(mode == 3)
	{
		discovery_mode = 'l';
	}
	
	LCD_SendString("M:");
	LCD_SendCharacter(discovery_mode);
	_delay_ms(250);
	_delay_ms(250);
	_delay_ms(250);
	_delay_ms(250);
	_delay_ms(250);
	_delay_ms(250);
	MOTOR_Forward(standard_speed_);
}

// One of the following three methods are called in the event of a 
// three way junction described by the ASCII art above each method.
/*  |
  ->--- ONE is used in junctions of this type.
*/
void JUNCTION_ThreeWayONE()
{
	MOTOR_Stop();
	LCD_SendString("   3V3   ");
	_delay_ms(25);
	MOTOR_Forward(standard_speed_);
	
	if (discovery_mode == 'l')
	{
		TURN_Left(1);
	}
	
	else if (discovery_mode == 'b')
	{
		MOTOR_Stop();
		_delay_ms(250);
		_delay_ms(250);
		_delay_ms(250);
		_delay_ms(250);
		MOTOR_Forward(standard_speed_);
		TURN_Back(1);

	}
	
	else if (discovery_mode == '?')
	{
		DISCOVERY_SetRandom();
		JUNCTION_ThreeWayONE();
		discovery_mode = '?';
	}

	if ((discovery_mode == 'r') || (discovery_mode == 'f'))
	{
		// Keep going forward
		distance_counter = 0;
		distance_flag = 0;
		LCD_Clear();
		
		MOTOR_Stop();
		Get_sensor_values();
		int angle_right =  arrSensor[0];
		
		if (angle_right < 0)
		{
			MOTOR_RotateLeft(-angle_right);
		}
		else
		{
			MOTOR_RotateRight(angle_right);
		}
		
		MOTOR_Forward(standard_speed_);
		
		while( PATHCOUNT_Left() > 0 )
		{
			_delay_us(250);
		}
		
		JUNCTION_delay(3);
		return;
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
		while(!WALL_CLOSE_AHEAD())
		{
			_delay_us(250);
		}
		TURN_Left(0);
	}
	else if (discovery_mode == 'r')
	{
		// Turn right
		while(!WALL_CLOSE_AHEAD())
		{
			_delay_us(250);
		}
		TURN_Right(0);
	}
	else if (discovery_mode == 'b')
	{
		TURN_Back(2);
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

LCD_Clear();
MOTOR_Stop();
LCD_SendCharacter(discovery_mode);
LCD_SendCharacter(discovery_mode);
LCD_SendCharacter(discovery_mode);
LCD_SendCharacter(discovery_mode);
LCD_SendCharacter(discovery_mode);
_delay_ms(250);
_delay_ms(250);
MOTOR_Forward(standard_speed_);

	if (discovery_mode == 'r')
	{
		TURN_Right(1);
	}
	
	else if (discovery_mode == 'b')
	{
		MOTOR_Stop();
		_delay_ms(250);
		_delay_ms(250);
		_delay_ms(250);
		_delay_ms(250);
		LCD_Clear();
		LCD_SendString(" TUUURN BACK 3v3");
		MOTOR_Forward(standard_speed_);
		TURN_Back(3);
	}

	else if ( (discovery_mode == 'f') || (discovery_mode == 'l') )
	{
		// Keep going forward
		distance_counter = 0;
		distance_flag = 0;
		LCD_Clear();
		
		MOTOR_Stop();
		Get_sensor_values();
		int angle_left =  arrSensor[2];
		
		if (angle_left < 0)
		{
			MOTOR_RotateRight(-angle_left);
		}
		else
		{
			MOTOR_RotateLeft(angle_left);
		}
		
		MOTOR_Forward(standard_speed_);
		
		while( PATHCOUNT_Right() > 0 )
		{
			_delay_us(250);
		}
		
		JUNCTION_delay(3);
		//return;
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
		TURN_Right(2);
	}
	else if(discovery_mode == 'l')
	{
		TURN_Left(2);
	}
	else if(discovery_mode == 'f')
	{
		//keep going forward
		distance_counter = 0;
		distance_flag = 0;
		
		while ((LEFTPATHONE() || RIGHTPATHONE())
				 || ((PATHCOUNT_Left() > 0) && (PATHCOUNT_Right() > 0)))
		{
			_delay_us(250);
			LCD_SetPosition(1);
			LCD_SendString("Fourway");
		}
	}	
	else if(discovery_mode == 'b')
	{
		TURN_Back(0);
	}
	else if(discovery_mode == '?')
	{
		DISCOVERY_SetRandom();
		JUNCTION_FourWay();
		discovery_mode = '?';
	}
}

void MAP_main()
{
	Send_map_values();
	// Save these under more convenient names
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t posX_ = MAP_currentPos[1];
	
	// Normal searching phase
	Phase0:
	if (!MAP_operatingMode_ && !MAP_rotating_ && !MAP_movingForward_)
	{
		MAP_countSquares();

		// If it's a visited junction
		if (MAP_array[posY_][posX_].visited && (MAP_array[posY_][posX_].description == 5))
		{
			// Add the directions between this and the last junction
			if (MAP_addJunctionDist(MAP_currentJunction, MAP_array[posY_][posX_].junctionNumber) ||
			MAP_addJunctionDist(MAP_array[posY_][posX_].junctionNumber, MAP_currentJunction))
			{
				MAP_addJunctionDir(MAP_array[posY_][posX_].junctionNumber, MAP_currentJunction, (MAP_currentDir + 2) % 4);
				if (MAP_array[posY_][posX_].junctionNumber != MAP_currentJunction)
				{
					MAP_addJunctionDir(MAP_currentJunction, MAP_array[posY_][posX_].junctionNumber, MAP_lastJunctionDir);
				}
			}
			
			// And if it has unexplored roads
			// Else go to far junction phase
			int fatIf_;
			fatIf_ = MAP_unexploredSquares -
			(MAP_array[posY_ - 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ - 1][posX_].junctionNumber].hasUnex) -
			(MAP_array[posY_ + 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ + 1][posX_].junctionNumber].hasUnex) -
			(MAP_array[posY_][posX_ - 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ - 1].junctionNumber].hasUnex) -
			(MAP_array[posY_][posX_ + 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ + 1].junctionNumber].hasUnex);
			if (fatIf_ >= 1)
			{
				MAP_currentJunction = MAP_array[posY_][posX_].junctionNumber;
				//MAP_junctionOrderArray[MAP_array[posY_][posX_].junctionNumber].hasUnex = 0;
			}
			else if(fatIf_ == 0)
			{
				MAP_currentJunction = MAP_array[posY_][posX_].junctionNumber;
				MAP_junctionOrderArray[MAP_currentJunction].hasUnex = 0;
				MAP_decideDestination();
				MAP_operatingMode_ = 2;
				goto RotatingPhase; // Exit this phase
			}

		}
		// Or if it's a new square
		else{
			// If it's a junction, add it
			// Else if it's a dead end, trace the way back to the previous junction
			if (MAP_exploredSquares > 2)
			{
				MAP_addJunction();
			}
			else if (MAP_unexploredSquares == 0)
			{
				MAP_nextDir = (MAP_currentDir + 2) % 4;
				MAP_rotating_ = 1;
				MAP_movingForward_ = 1;
				MAP_operatingMode_ = 3;
				MAP_nextJunctionShort = MAP_currentJunction;
				goto RotatingPhase; // Exit this phase
			}
		}

		// Decide the next direction
		MAP_decideDirection('l'); // Uses a = random, r = right-first, l = left-first
		if (MAP_currentDir != MAP_nextDir)
		{
			MAP_rotating_ = 1;
		}
			MAP_movingForward_ = 1;
		}
	
	// Rotating phase
	// SIM
	RotatingPhase:
	
	// Go until nextJunctionShort phase
	// This phase is meant to navigate through an explored corridor between two junctions
	//Phase3:
	if ((MAP_operatingMode_ == 3) && !MAP_rotating_ && !MAP_movingForward_)
	{
		// Checks if we can move right or left, then rotate accordingly, otherwise move forward
		if (MAP_checkDir((MAP_currentDir + 3) % 4))
		{
			MAP_nextDir = (MAP_currentDir + 3) % 4;
			MAP_rotating_ = 1;
		}
		else if (MAP_checkDir((MAP_currentDir + 5) % 4))
		{
			MAP_nextDir = (MAP_currentDir + 5) % 4;
			MAP_rotating_ = 1;
		}
		MAP_movingForward_ = 1;

		// If we have arrived to the desired junction
		if ((MAP_currentPos[0] == MAP_junctionOrderArray[MAP_nextJunctionShort].posY) &&
		(MAP_currentPos[1] == MAP_junctionOrderArray[MAP_nextJunctionShort].posX))
		{
			MAP_travelledDist = 0;
			MAP_countSquares();
			MAP_currentJunction = MAP_nextJunctionShort;

			// If the current junction has unexplored squares we go to normal phase
			// Else we go to far junction phase
			int fatIf_;
			fatIf_ = MAP_unexploredSquares -
			(MAP_array[posY_ - 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ - 1][posX_].junctionNumber].hasUnex) -
			(MAP_array[posY_ + 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ + 1][posX_].junctionNumber].hasUnex) -
			(MAP_array[posY_][posX_ - 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ - 1].junctionNumber].hasUnex) -
			(MAP_array[posY_][posX_ + 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ + 1].junctionNumber].hasUnex);
			if (fatIf_ >= 1)
			{
				MAP_rotating_ = 0;
				MAP_movingForward_ = 0;
				MAP_operatingMode_ = 0;
				goto Phase0;
			}
			else
			{
				MAP_decideDestination();
				MAP_operatingMode_ = 2;
				MAP_movingForward_ = 0;
				goto Phase2;
			}
		}
	}

	// Go to a junction farther away phase
	// Decides which junction we should be heading to
	Phase2:
	if ((MAP_operatingMode_ == 2) && !MAP_rotating_ && !MAP_movingForward_)
	{
		// If we have arrived at the desired junction then we start the normal phase
		// Else if all the map has been explored we quit the main loop
		// Else we start moving towards the nextJunctionShort, going to that phase
		if ((MAP_currentPos[0] == MAP_junctionOrderArray[MAP_nextJunctionLong].posY) &&
		(MAP_currentPos[1] == MAP_junctionOrderArray[MAP_nextJunctionLong].posX))
		{
			MAP_travelledDist = 0;
			MAP_operatingMode_ = 0;
			goto Phase0;
		}
		else if (MAP_nextJunctionLong == 255)
		{
			MAP_LOOPer = 0;
		}
		else
		{
			MAP_nextDir = MAP_getDirection(MAP_currentJunction, MAP_nextJunctionShort);
			if (!(MAP_currentDir == MAP_nextDir))
			{
				MAP_rotating_ = 1;
			}
			MAP_movingForward_ = 1;
			MAP_operatingMode_ = 3;
			goto RotatingPhase;
		}
	}

	// This phase is activated when we have discovered the goal
	if (MAP_operatingMode_ == 4 && !MAP_rotating_ && MAP_movingForward_)
	{
		// Go ResQ.PL, go!
	}

	// Checks if all the map has been explored
	//MAP_checkIfDone();
	MOTOR_Stop();
	LCD_Clear();
	LCD_SetPosition(0);
	LCD_SendString("Y:");
	LCD_display_uint16(MAP_currentPos[0]);
	LCD_SendString(" ");
	LCD_SendString("X:");
	LCD_display_uint16(MAP_currentPos[1]);
	LCD_SendString(" ");
	LCD_SetPosition(16);
	LCD_SendString("cD");
	LCD_display_uint16(MAP_currentDir);
	LCD_SendString(" ");
	LCD_SendString("nD");
	LCD_display_uint16(MAP_nextDir);
	LCD_SendString(" ");
	LCD_SendString("J:");
	LCD_display_uint16(MAP_array[posY_][posX_].description);
	LCD_SendString(" ");
}


void Floor_Marker()
{
    if (REFLEX_GetMarker())
	{
		MOTOR_Stop();
		_delay_ms(250);
		_delay_ms(250);
		SERVO_LevelLow();
		_delay_ms(250);
		_delay_ms(250);
		SERVO_ReleaseGrip();
		_delay_ms(250);
		_delay_ms(250);
	}
        //if(MAP_currentPos[0] != 16 && MAP_currentPos[1] != 15)
        //{
	        //if(resque_mode == 'd')
	        //{
		        //MAP_setGoal();
		        //resque_mode = 'q';
	        //}
	        //else if(resque_mode == 'q' && holds_item)
	        //{
		        //SERVO_ReleaseGrip();
	        //}
        //}
        //else
        //{
	        //MOTOR_Stop();
	        //for(uint8_t i = 0; i<5; i++){
		        //_delay_ms(250);
	        //}
	        //SERVO_SetGrip();
	        //holds_item = 1;
	        //for(uint8_t i = 0; i<5; i++){
		        //_delay_ms(250);
	        //}
	        //MOTOR_Forward(standard_speed_);
        //}
    //}
}

void Junction()
{
    MOTOR_Forward(standard_speed_);
    
    while (!(LeftPathBoth) && !(RightPathBoth) && !(WallCloseAhead)) //Keep going until center of intersect
    {
        Update_All_FUCKING_values();
        _delay_us(250);
		
		//Jump out of function if incorrect sensor value was given earlier.
		if (!((PathCountLeft > 0) || (PathCountRight > 0)))
		{
			return;
		}
    }
    
	
    MAP_moveForward();
    JUNCTION_delay(3);
    
	
    // Now in intersect. Determine what type:
    if ((PathCountLeft > 0) && (PathCountRight > 0)) // 4-way or 3-way-2
    {
        if (PathAhead) // 4-way
        {            
			set_map_FourWay();
            
            MAP_main();
            DISCOVERY_SetMode();
            
            JUNCTION_FourWay();
            MAP_rotate();
        }
        else // 3-way-2
        {
			set_map_ThreeWay2();
            
            MAP_main();
            DISCOVERY_SetMode();
            
            JUNCTION_ThreeWayTWO();
			
			LCD_Clear();
			LCD_SetPosition(0);
			LCD_SendString("junction 3w2");
            MAP_rotate();
            
        }
    }
    else if (PathCountRight > 0) // 3-way-3 or RightTurn
    {
        if (PathAhead) // 3-w-3
        {
			set_map_ThreeWay3();
            
            MAP_main();
            DISCOVERY_SetMode();
            
            JUNCTION_ThreeWayTHREE();
			
            MAP_rotate();
			if( (discovery_mode == 'f') || (discovery_mode == 'l'))
			{
				return;
			}
        }
        else //if (PathCountLeft == 0) //Right turn
        {
			set_map_RightTurn();
            
            MAP_main();
            DISCOVERY_SetMode();
            
			if (discovery_mode == 'r')
			{
				TURN_Right(0);
				MAP_rotate();
			}
	
			else if (discovery_mode == 'l')
			{
				MOTOR_Stop();
				LCD_Clear();
				LCD_SendString("                        lr            ");
				TURN_Left(0);
				MAP_rotate();
			}
	
			else if (discovery_mode == 'b')
			{
				MOTOR_Stop();
				LCD_Clear();
				LCD_SendString("                        br            ");
				TURN_Back(1);
				MAP_rotate();
			}
			
			else if (discovery_mode == 'f')
			{
				MOTOR_Stop();
				LCD_Clear();
				LCD_SendString("                        fr            ");
				
				// Keep going forward
				distance_counter = 0;
				distance_flag = 0;
				LCD_Clear();
					
				MOTOR_Stop();
				Get_sensor_values();
				int angle_left =  arrSensor[2];
					
				if (angle_left < 0)
				{
					MOTOR_RotateRight(-angle_left);
				}
				else
				{
					MOTOR_RotateLeft(angle_left);
				}
					
				MOTOR_Forward(standard_speed_);
					
				while( PATHCOUNT_Right() > 0 )
				{
					_delay_us(250);
				}
					
				JUNCTION_delay(3);
			}
        }
    }
    else if (PathCountLeft > 0) // 3-way-1 or Left turn
    {
        if (PathAhead) // 3-way-1
        {
			set_map_ThreeWay1();
            
            MAP_main();
            DISCOVERY_SetMode();
            JUNCTION_ThreeWayONE();
            MAP_rotate();
			if( (discovery_mode == 'f') || (discovery_mode == 'r'))
			{
				return;
			}
		}
        else //if (PathCountRight == 0)// Left turn
        {
			set_map_LeftTurn();
            
            MAP_main();
            DISCOVERY_SetMode();
			
			if (discovery_mode == 'r')
			{
				MOTOR_Stop();
				LCD_Clear();
				LCD_SendString("      rl      ");
				TURN_Right(0);
				MAP_rotate();
			}
	
			else if (discovery_mode == 'l')
			{
				TURN_Left(0);
				MAP_rotate();
			}
			
			else if (discovery_mode == 'b')
			{
				MOTOR_Stop();
				LCD_Clear();
				LCD_SendString("      bl    ");
				TURN_Back(1);
				MAP_rotate();
			}
			
			else if (discovery_mode == 'f')
			{
				
				
				MOTOR_Stop();
				LCD_Clear();
				LCD_SendString("      fl      ");
				
				// Keep going forward
				distance_counter = 0;
				distance_flag = 0;
				LCD_Clear();
		
				MOTOR_Stop();
				Get_sensor_values();
				int angle_right =  arrSensor[0];
		
				if (angle_right < 0)
				{
					MOTOR_RotateLeft(-angle_right);
				}
				else
				{
					MOTOR_RotateRight(angle_right);
				}
		
				MOTOR_Forward(standard_speed_);
		
				while( PATHCOUNT_Left() > 0 )
				{
					_delay_us(250);
				}
		
				JUNCTION_delay(3);
			}
        }
    }
    else if (WallCloseAhead)
    {
        MOTOR_Stop();
        LCD_Clear();
        LCD_SendString("Dead End");
        LCD_SetPosition(16);
        LCD_SendString("I sv?ng IF.. FEL");
        while(1);
		set_map_DeadEnd();
        
        MAP_main();
        DISCOVERY_SetMode();
        DEAD_END();
        MAP_rotate();
        
    }
    
    // Ending to make sure robot out of junction
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
    
    while (abs(angle_) > 15)
    {
        JUNCTION_delay(1);
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
    }
    

}

void Pre_PD_controll()
{
    Get_sensor_values();
    if( ((arrSensor[1] + arrSensor[3]) / 2) < 20)
    {
        if((arrSensor[2] - angle_) > 10)
        {
            _delay_us(1);
        }
        else
        {
            angle_ =  arrSensor[2];
            offset_ =  arrSensor[3];
        }
    }
    else
    {
        if((arrSensor[0] - angle_) > 10 )
        {
            _delay_us(1);
        }
        else
        {
            angle_ =  arrSensor[0];
            offset_ =  arrSensor[1];
        }
    }
    
    if(abs(offset_-20) <= 2)
    {
        control_mode = 'c';
    }
    // Puts the automatic control in rapid mode, push the robot to the middle lane.
    else
    {
        control_mode = 'r';
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

void AutomaticControl()
{
	
	Update_All_FUCKING_values();

	Floor_Marker();

// 	if (REFLEX_GetMarker())
// 	{
// 		MOTOR_Stop();
// 		_delay_ms(250);
// 		_delay_ms(250);
// 		SERVO_LevelLow();
// 		_delay_ms(250);
// 		_delay_ms(250);
// 		SERVO_ReleaseGrip();
// 		_delay_ms(250);
// 		_delay_ms(250);
// 	}
	
	if( (PathCountLeft > 0) || (PathCountRight > 0) ){ //Path to left or right
		
        Junction();
		
	}
	else if ( !( LeftPathOne || RightPathOne) || 
			(LeftPathOne && (arrSensor[1] > 26) /*ROoffs*/) ||  //To close to right wall
			(RightPathOne && (arrSensor[3] > 14)/*LOffs*/) ) //To close to left wall
	{
        Pre_PD_controll();
	}
	
	if (WALL_CLOSE_AHEAD()) // dead end square
	{
		MAP_moveForward();
		set_map_DeadEnd();
		
		MAP_main();
		DISCOVERY_SetMode();
		DEAD_END();
		MAP_rotate();
		angle_ = 0;
	}
	if((distance_counter >= 10) && (distance_flag == 1))
	{
		MAP_moveForward();
		
		set_map_Corridor();
		
		MAP_main();
		//DISCOVERY_SetMode();
		
		if (discovery_mode == 'b')
		{
			//TURN_Back(4);
		}
		//MAP_rotate();
		distance_flag = 0;
	}
}

//________________________________AUTOMATIC CONTROL END_____________________________________

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
	Speed_Interrupt_Init(); //KOMMENTERA IN, MEN FUNGERAR EJ ATT MANUELLSTYRA D?
	PWM_SetDirLeft(1);
	PWM_SetDirRight(1);
	
	distance_counter = 0;
	distance_flag = 0;
	//MAP_addJunction();
	//MAP_junctionOrderArray[MAP_currentJunction].hasUnex = 0;
	MAP_array[16][15].description = 3;
	MAP_currentDir = 1;
	MAP_nextDir = 1;
	MAP_setVisited();	// Map initialization
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
    		while(AUTONOM_MODE && MAP_LOOPer)
    		{
	    		AutomaticControl();
				Send_sensor_values();
				LCD_SetPosition(0);
// 				LCD_SendString("Y:");
// 				LCD_display_uint16(MAP_currentPos[0]);
// 				LCD_SendString(" ");
// 				LCD_SendString("X:");
// 				LCD_display_uint16(MAP_currentPos[1]);
// 				LCD_SendString(" ");
				LCD_SendString("Ar:");
				LCD_display_uint16(MAP_junctionOrderArray[0].hasUnex);
				LCD_SendString(" ");
				//LCD_SetPosition(16);
				LCD_SendString("cDir:");
				LCD_display_uint16(MAP_currentDir);
				LCD_SendString(" ");
				LCD_SendString("nDir:");
				LCD_display_uint16(MAP_nextDir);
				LCD_SendString(" ");
				LCD_SetPosition(16);
				LCD_SendString("jC:");
				LCD_display_uint16(MAP_junctionCount);
				LCD_SendString(" ");
    		}
			while(!MAP_LOOPer)
			{
				MOTOR_Stop();
				LCD_Clear();
				LCD_SetPosition(0);
				LCD_SendString("Map Looper 0 ");
				_delay_ms(75);
// 				LCD_display_uint8(MAP_array[14][16].description);
// 				LCD_SendCharacter(' ');
// 				LCD_display_uint8(MAP_array[14][16].visited);
// 				
				if(MANUAL_MODE)
				{
					MAP_LOOPer = 1;
				}
			}
			    		
    		_delay_ms(10);
			MOTOR_Stop();
    		LCD_Clear();
    		LCD_SetPosition(2);
    		LCD_SendString("MANUAL_MODE");			
			MOTOR_Stop();
			LCD_Clear();
    		while(MANUAL_MODE)
    		{
				Get_sensor_values();
    			MANUAL_DRIVE();
				LCD_SetPosition(0);
				LCD_SendString("PL");
				LCD_display_uint8(PATHCOUNT_Left());
				LCD_SendString(" ");
				LCD_SendString("BL");
				LCD_display_int8(LEFTPATHBOTH());
				LCD_SendString(" ");
				LCD_SendString("OL");
				LCD_display_uint8(LEFTPATHONE());
				LCD_SendString(" ");
				LCD_SetPosition(16);
				LCD_SendString("PR");
				LCD_display_uint8(PATHCOUNT_Right());
				LCD_SendString(" ");
				LCD_SendString("BR");
				LCD_display_int8(RIGHTPATHBOTH());
				LCD_SendString(" ");
				LCD_SendString("Re");
				LCD_display_uint8(REFLEX_GetMarker());
				LCD_SendString(" ");
				_delay_ms(10);
    		}
			MOTOR_Stop();
  	}
	  
}