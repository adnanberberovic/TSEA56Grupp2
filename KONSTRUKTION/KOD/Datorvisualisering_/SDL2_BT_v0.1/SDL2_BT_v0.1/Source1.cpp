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

#DEFINE BUFFSIZE 10

HANDLE hComm; //Handle for comport
DCB dcbSerialParams = { 0 }; //dcb for comport
COMMTIMEOUTS timeouts = { 0 }; //value for timeoutconfig
DWORD BytesWritten_ = 0; // how many bytes that were actually written

int ReadBuff_[BUFFSIZE];
int ReadBuff_[BUFFSIZE];


//Initialize commport
void Init_CommPort(string comport){

	hComm = CreateFile(
		comport.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

	//Parametersettings ----------------------------------
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hComm, &dcbSerialParams)){
		//Error getting state
		printf("Error getting state %d\n");
	}

	//Setting baudrate and similar settings
	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;

	SetCommState(hComm, &dcbSerialParams);

	if (!SetCommState(hComm, &dcbSerialParams)){
		printf("Error setting state. %d \n");

	}

	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 50;
	timeouts.WriteTotalTimeoutConstant = 10;

	if (!SetCommTimeouts(hComm, &timeouts))
	{
		printf("Error setting time-outs. %d");
		return;
	}

}
//Initialize welcome
void Init_Welcome()
{
	cout << "\n*******************************************\n\n";
	cout << "          ResQ.Pl - Master Race \n";
	cout << "\n*******************************************\n\n";
	cout << " \n - 'init' to establish connection\n - 'close connection' to close initialized connection \n - 'read' / 'write' to do that\n Enter command:" << endl;
	cout << "> ";
}