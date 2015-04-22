// PC_v0.2.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <windows.h>
#include <stdint.h>

using namespace std;

//Variables for initiating com-port
HANDLE hSerial;
DCB dcbSerialParams = { 0 };
COMMTIMEOUTS timeouts = { 0 };

//For reading/writning from com
int8_t ReadBuff_[10]; // Buffer to store indata from com, [2]-bytes
DWORD BytesRead_ = 0; // how many bytes that were actually read
int8_t WriteBuff_[10]; // Buffer to store outdata from com, [2]-bytes
DWORD BytesWritten_ = 0; // how many bytes that were actually written

struct node { // definition of the linked list node
	int8_t val;
	struct node *next;
};

typedef struct node buffer_; // buffer_ definieras

buffer_ *head_BTout = NULL;
buffer_ *head_BTin = NULL;

void add_node(buffer_* lst_head, int8_t val)
{

	buffer_ * curr = lst_head;
	while (curr->next != NULL) // step to end of list
	{
		curr = curr->next;
	}
	curr->next = new buffer_;
	curr->next->val = val;
	curr->next->next = NULL; // Add node last.
}

int pop_node(buffer_ ** lst_head)
{
	buffer_* next_node = NULL;
	int8_t retval = 0;

	if (*lst_head != NULL)
	{
		next_node = (*lst_head)->next;
		retval = (*lst_head)->val;
		free(*lst_head);
		*lst_head = next_node;
	}

	return retval;
}

void BT_send(uint8_t val)
{
	add_node(head_BTout, val);
}

void flush_list(buffer_ ** lst_head)
{
	while (*lst_head != NULL)
	{
		pop_node(lst_head);
	}
}


void TimeoutSetup()
{
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if (!SetCommTimeouts(hSerial, &timeouts)){
		cerr << "**** TimeoutSetup error. ****\n";
	}
}

void Comport_init(wstring comport)
{
	hSerial = CreateFile(
		comport.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

	//Parametersettings ----------------------------------
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams))
	{
		//Error getting state
		cerr << "Error getting state\n";
	}

	//Setting baudrate and similar settings
	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;

	if (!SetCommState(hSerial, &dcbSerialParams))
	{
		cerr << "Error setting state\n";
	}
	//------------------------------------------
}

void Read_ComData() //Reads data from hSerial (COM4) and stores in ReadBuff.
{
	fill(begin(ReadBuff_), end(ReadBuff_), '\0');
	if (!ReadFile(hSerial, ReadBuff_, 10, &BytesRead_, NULL)) // BytesRead contains amount of read bytes
	{
		cerr << "Reading from file failed!\n";
	}
	cout << "\nReadbuff: ";
	for (int i = 0; i < 10; i++)
	{
		char c = static_cast<char>(ReadBuff_[i]);
		cout << c;
	}
	cout <<	"\n Bytes read: "<< BytesRead_ << endl << "> ";

}

void Write_ComData() //Writes data to hSerial (COM4) and stores in WriteBuff.
{
	fill(begin(WriteBuff_), end(WriteBuff_), '\0');
	char W_in{};
	int i = 0;

	while (cin.peek() != '\n')
	{
		cin >> W_in;
		WriteBuff_[i] = W_in;
		i++;
	}

	cout << "\nWriteBuff_: " << WriteBuff_ << endl;
	if (!WriteFile(hSerial, WriteBuff_, 10, &BytesWritten_, NULL)) // "100" says how many bytes to write
	{
		cerr << "Writing to file failed!\n";
	}
	else
	{
		cin >> W_in;
		cout << "Data written\n> ";
	}

}

void Init_Welcome()
{
	cout << "\n*******************************************\n\n";
	cout << "          ResQ.Pl - Master Race \n";
	cout << "\n*******************************************\n\n";
	cout << " \n - 'init' to establish connection\n - 'close connection' to close initialized connection \n - 'read' / 'write' to do that\n Enter command:" << endl;
	cout << "> ";
}

int _tmain(int argc, _TCHAR* argv[])
{
	//Intro to print out
	char in; //Input from keyboard
	bool ComInit_ = false; //Flag 


	string instring_;
	wstring com3_(L"COM3");

	Init_Welcome();

	//Read in from keyboard and initialize connection
	cin >> noskipws;
	while (cin.peek() != EOF)
	{
		cin >> in;

		if (in == '\n') //if command has been entered
		{

			// init com-procedure if stringmatch
			if (ComInit_ == false && instring_ == "init")
			{
				Comport_init(com3_); //Initializes COM4 
				ComInit_ = true;
				cout << "COM3" << " initialized!\n> ";
			}

			// initialization already done
			else if (ComInit_ == true && instring_ == "init")
			{
				cout << "COM3 already initialized!\n> ";
			}

			//Write to com
			else if (instring_ == "write")
			{
				cout << "Write what to send:\n> ";
				Write_ComData();
			}

			//Read from com
			else if (instring_ == "read")
			{
				cout << "Reading...\n";
				Read_ComData();
			}

			// Close connection
			else if ((instring_ == "close connection") | (instring_ == "close"))
			{
				CloseHandle(hSerial);
				ComInit_ = false;
				cout << "Connection to COM3 closed!\n> ";
			}
			else
			{
				cout << "Unknown input\n> ";
			}

			instring_ = "";
		}
		else
		{
			instring_ = instring_ + in;
		}
	}
	return 0;
}

