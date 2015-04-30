#ifndef GUI__Bluetooth__
#define GUI__Bluetooth__

#include <iostream>
#include <SDL.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <windows.h>
#include <stdint.h>


using namespace std;

//#define BUFFSIZE 10
//
//HANDLE hComm; //Handle for comport
//DCB dcbSerialParams = { 0 }; //dcb for comport
//COMMTIMEOUTS timeouts = { 0 }; //value for timeoutconfig
//
//DWORD BytesWritten_ = 0; // how many bytes that were actually written
//DWORD BytesRead_ = 0; // how many bytes that were actually read
//char ReadBuff_[BUFFSIZE];
//char WriteBuff_[BUFFSIZE];
//int flag__ = 0;
//
//bool Command_flag = false;


//Initialize commport
HANDLE Init_CommPort(string comport){

	hComm = CreateFile(
		comport.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

	//	Parametersettings ----------------------------------
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hComm, &dcbSerialParams)){
		//		Error getting state
		printf("Error getting state %d\n");
	}

	// Setting baudrate and similar settings
	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;

	SetCommState(hComm, &dcbSerialParams);

	if (!SetCommState(hComm, &dcbSerialParams)){
		printf("Error setting state. %d \n");

	}

	timeouts.ReadIntervalTimeout = 1;
	timeouts.ReadTotalTimeoutMultiplier = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 50;
	timeouts.WriteTotalTimeoutConstant = 10;

	if (!SetCommTimeouts(hComm, &timeouts))
	{
		printf("Error setting time-outs. %d");
		return 0;
	}
	return hComm;
}

//Initialize welcome
//void Init_Welcome()
//{
//	cout << "\n*******************************************\n\n";
//	cout << "          ResQ.Pl - Master Race \n";
//	cout << "\n*******************************************\n\n";
//	cout << " \n - 'init' to establish connection\n - 'close connection' to close initialized connection \n - 'read' / 'write' to do that\n Enter command:" << endl;
//	cout << "> ";
//}

void Read_ComData(){
	fill(begin(ReadBuff_), end(ReadBuff_), '\0');

	if (Command_flag == true) //instruction that was sent = command/request 
	{
		if (!ReadFile(hComm, ReadBuff_, 1, &BytesRead_, NULL)) // rid of ninja 0
		{
			cerr << "Reading from file failed!\n";
		}
	}
	cout << Command_flag;
	Command_flag = false; //Reset commandflag

	if (!ReadFile(hComm, ReadBuff_, BUFFSIZE, &BytesRead_, NULL)) // BytesRead contains amount of read bytes
	{
		cerr << "Reading from file failed!\n";
	}
	cout << "\nReadbuff: ";

	for (uint8_t i = 0; i < BytesRead_; i++)
	{
		int c = ReadBuff_[i];
		cout << c;
	}
	cout << "\n Bytes read: " << BytesRead_ << endl << "> ";

}

void Check_command(char in_){ //check if input
	switch (in_){
	case '2': Command_flag = true;
		break;
	case '3': Command_flag = true;
		break;
	case '1': Command_flag = true;
		break;
	}
	return;
}

void Write_ComData(){
	fill(begin(WriteBuff_), end(WriteBuff_), '\0');
	char W_in{};
	int i = 0;
	while ((cin.peek() != '\n') && (i != BUFFSIZE)){
		cin >> W_in;
		Check_command(W_in);
		WriteBuff_[i] = W_in;
		i++;
	}
	cout << "\nWriteBuff_: " << WriteBuff_ << endl;
	if (!WriteFile(hComm, WriteBuff_, BUFFSIZE, &BytesWritten_, NULL)) // "100" says how many bytes to write
	{
		cerr << "Writing to file failed!\n";
	}
	else
	{
		cin >> W_in;
		cout << "Data written\n> ";
	}
}

void Get_Sensor_values(int8_t arrTarget[], int size, uint8_t startbit_)
{
	uint8_t startSensor[1] = { startbit_ };
	if (!WriteFile(hComm, startSensor, 1, &BytesWritten_, NULL)) // Send "get-sensor"
	{
		cerr << "Writing to file failed!\n";
	}
	if (!ReadFile(hComm, arrTarget, size, &BytesRead_, NULL)){
		cerr << "Reading from file failed!\n";
	}
	if (BytesRead_ != size){
		cerr << "Bytes read did not match arr size! :  " << BytesRead_ << "   " << (sizeof(arrTarget) / sizeof(arrTarget[0])) << endl;
	}
	//cout << "Bytes re1ad: " << BytesRead_ << "____" << static_cast<int>(arrTarget[0]) << static_cast<int>(arrTarget[1]) << static_cast<int>(arrTarget[2]) << static_cast<int>(arrTarget[3]) << endl;

	if (!ReadFile(hComm, ReadBuff_, 1, &BytesRead_, NULL)) // rid of ninja 0
	{
		cerr << "Reading from file failed!\n";
	}
}

//Handles input from keyboard
void Input_Loop(){
	string instring_;
	char in;
	bool ComInit_ = false;
	cin >> noskipws;
	while (cin.peek() != EOF)
	{
		cin >> in;

		if (in == '\n'){ //if command has been entered
			// init com-procedure if stringmatch
			if (ComInit_ == false && instring_ == "init"){
				Init_CommPort("COM3"); //Initializes COM3
				ComInit_ = true;
				cout << "COM3" << " initialized!\n> ";
			}

			// initialization already done
			else if (ComInit_ == true && instring_ == "init"){
				cout << "COM3 already initialized!\n> ";
			}

			//Write to com
			else if (instring_ == "write"){
				cout << "Write what to send:\n> ";
				Write_ComData();
			}

			//Read from com
			else if (instring_ == "read")	{
				cout << "Reading...\n";
				Read_ComData();
			}

			// Close connection
			else if ((instring_ == "close connection") | (instring_ == "close"))
			{
				CloseHandle(hComm);
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
}

#endif