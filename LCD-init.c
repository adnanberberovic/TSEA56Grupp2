/*
 * LCD JM162A, 2 rader × 16 tecken
 * displayen har en rad (internt) som sedan presenteras på två rader
 * En två-raders display måste alltså konfigureras för två rader för att fungera som en sådan.
 * 8-bitars mode
 * 5x7 dots format for characters + cursor
 * 
 */ 


#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
// #include <avr/pgmspace.h>
// #include <util/delay.h>


/// en del funktioner kan deklareras i en enskild *.h fil ///
// void LCD_busy();
// void LCD_init();
// void send_command_to_LCD(char cmd);
// 

int LCD_busy()
{
		DDRD = 0; //sätter port D till ingång
		PORTB = 1 << 1; //read busy flag
		PORTB &= ~(1 << 0); // clear bit 0 in PORTB to 0 (sätter register select instruction "RS=0")
		PORTB = (1 << 2); // aktiverar LCD:en (enable pinne på LCD)
//		_delay_us(2);
		char instr = PIND;
		PORTB &= ~(1 << 2); // clear bit 2 in PORTB to 0
		PORTB &= ~(1 << 1); // clear bit 1 in PORTB to 0
		DDRD = 0xff; //sätter port D till utgång
		return instr >> 7; // MSB is the busy flag
}

void send_command_to_LCD(char cmd) {
	while (LCD_busy())
//	_delay_ms(1);

	PORTB &= ~(1 << 0); // clear bit 0 in PORTB to 0 (sätter Register Select till instruction "RS=0")
	PORTD = cmd;
	PORTB = 1 << 2;
//	if (cmd == 0b01 || cmd == 0b10) // if clear or return home instruction
//	_delay_ms(1.5);				    // then execution time is longer
//	else
//	_delay_us(50);


	PORTB &= ~(1 << 2);
}

void send_char_to_LCD(char tecken)
{
	// siffror
	if(tecken == '0') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//00110000
		else if (tecken == '1') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//00110001 
		else if (tecken == '2') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//00110010 
		else if (tecken == '3') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//00110011 
		else if (tecken == '4') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//00110100 
		else if (tecken == '5') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//00110101 
		else if (tecken == '6') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//00110110 
		else if (tecken == '7') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//00110111 
		else if (tecken == '8') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//00111000 
		else if (tecken == '9') PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//00111001 
		// stora bokstäver
		else if (tecken == 'A') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01000001 
		else if (tecken == 'B') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01000010 
		else if (tecken == 'C') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01000011 
		else if (tecken == 'D') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01000100 
		else if (tecken == 'E') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01000101 
		else if (tecken == 'F') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01000110 
		else if (tecken == 'G') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01000111 
		else if (tecken == 'H') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01001000 
		else if (tecken == 'I') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01001001 
		else if (tecken == 'J') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01001010 
		else if (tecken == 'K') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01001011 
		else if (tecken == 'L') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01001100 
		else if (tecken == 'M') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01001101 
		else if (tecken == 'N') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01001110 
		else if (tecken == 'O') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01001111 
		// andra stora bokstäver		
		else if (tecken == 'P') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01010000 
		else if (tecken == 'Q') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01010001 
		else if (tecken == 'R') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01010010 
		else if (tecken == 'S') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01010011 
		else if (tecken == 'T') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01010100 
		else if (tecken == 'U') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01010101 
		else if (tecken == 'V') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01010110 
		else if (tecken == 'W') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01010111 
		else if (tecken == 'X') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01011000 
		else if (tecken == 'Y') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01011001 
		else if (tecken == 'Z') PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01011010 
		// små bokstäver
		else if (tecken == 'a') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01100001 
		else if (tecken == 'b') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01100010 
		else if (tecken == 'c') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01100011 
		else if (tecken == 'd') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01100100 
		else if (tecken == 'e') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01100101 
		else if (tecken == 'f') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01100110 
		else if (tecken == 'g') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01100111 
		else if (tecken == 'h') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01101000 
		else if (tecken == 'i') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01101001 
		else if (tecken == 'j') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01101010 
		else if (tecken == 'k') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01101011 
		else if (tecken == 'l') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01101100 
		else if (tecken == 'm') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01101101 
		else if (tecken == 'n') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);//01101110 
		else if (tecken == 'o') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);//01101111 
		// andra små bokstäver
		else if (tecken == 'p') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);//01110000 
		else if (tecken == 'q') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);//01110001 
		else if (tecken == 'r') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);//01110010 
		else if (tecken == 's') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);//01110011 
		else if (tecken == 't') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);//01110100 
		else if (tecken == 'u') PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);//01110101 
		else if (tecken == 'v') /*01110110*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);
		else if (tecken == 'w') /*01110111*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);
		else if (tecken == 'x') /*01111000*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);
		else if (tecken == 'y') /*01111001*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);
		else if (tecken == 'z') /*01111010*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);
		// övriga tecken
	 //else if (tecken == '/n') /*00100000*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(0 << 0);
		else if (tecken == '!') /*00100001*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(0 << 1)|(1 << 0);
		else if (tecken == '"') /*00100010*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(0 << 0);
		else if (tecken == '#') /*00100011*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(0 << 2)|(1 << 1)|(1 << 0);
		else if (tecken == '$') /*00100100*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(0 << 0);
		else if (tecken == '%') /*00100101*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(0 << 1)|(1 << 0);
		else if (tecken == '&') /*00100110*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(0 << 0);
	  //else if (tecken == ''') /*00100111*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(0 << 3)|(1 << 2)|(1 << 1)|(1 << 0);
		else if (tecken == '(') /*00101000*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(0 << 0);
		else if (tecken == ')') /*00101001*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(0 << 1)|(1 << 0);
		else if (tecken == '*') /*00101010*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);
		else if (tecken == '+') /*00101011*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);
		else if (tecken == ',') /*00101100*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);
		else if (tecken == '-') /*00101101*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);
		else if (tecken == '.') /*00101110*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);
		else if (tecken == '/') /*00101111*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(0 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);
		//
		else if (tecken == ':') /*00111010*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(0 << 0);
		else if (tecken == ';') /*00111011*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);
		else if (tecken == '<') /*00111100*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);
		else if (tecken == '=') /*00111101*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);
		else if (tecken == '>') /*00111110*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);
		else if (tecken == '?') /*00111111*/ PORTD = (0 << 7)|(0 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);
		//
		else if (tecken == '{') /*01111011*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(0 << 2)|(1 << 1)|(1 << 0);
		else if (tecken == '|') /*01111100*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(0 << 0);
		else if (tecken == '}') /*01111101*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(0 << 1)|(1 << 0);
	 //else if (tecken == '->') /*01111110*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);
     //else if (tecken == '<-') /*01111111*/ PORTD = (0 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);
		//
		else if (tecken == '^') /*01011110*/ PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(0 << 0);
		else if (tecken == '_') /*01011111*/ PORTD = (0 << 7)|(1 << 6)|(0 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);
		else PORTD = (1 << 7)|(1 << 6)|(1 << 5)|(1 << 4)|(1 << 3)|(1 << 2)|(1 << 1)|(1 << 0); //svart ruta
}

//void send_char_to_LCD(char tecken)
//{
	//PORTD = convert_(tecken);
	//
//}

// initieringen av LCD kontrollern, enligt Initializing Flowchart(Condition fosc=270KHz) i databladet
void LCD_init()
{		
//		_delay_ms(30);

		// konfigurera LCD:en för 8 bitar, 2 linjer, 5x8 pixlar (dots) dvs LCD:en får instruktion 00 0011 1000
		send_command_to_LCD(0b00111000);
//		_delay_us(39);

		// display,cursor and blinking off, instruktion 00 0000 1000
		send_command_to_LCD(0b00001000);
//		_delay_us(39);

		// Clear display, instruktion 00 0000 0001
		send_command_to_LCD(0b00000001);
//		_delay_ms(1.53);

		// cursor moving direction: left-to-right, skifta ej displayen (dvs shift disabled), instruktion 00 0000 0110
		send_command_to_LCD(0b00000110);

		// display on, cursor and blinking still off, instruktion 00 0000 1100
		send_command_to_LCD(0b00001100);
		
		
		PORTB = (1 << 0)|(0 << 1); // för att börja skicka data
				
}

int main()
{
	LCD_init();
    while(1)
    {
        //...
		// LCD application code:
		// Set display on
		// Set display line
		// Write data to CGRAM
		send_char_to_LCD('S');
		// Write data to line 1
		// Write data to line 2
    }
}
