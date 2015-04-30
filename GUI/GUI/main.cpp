//
//  main.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#include <windows.h>
#include "SetupSDL.h"
#include "State.h"
#include "Manual.h"
#include "Autonomt.h"
#include <iostream>
#include <string>
#include <stdexcept>
#include <cmath>

using namespace std;

HANDLE hComm; //Handle for comport
DCB dcbSerialParams = { 0 }; //dcb for comport
COMMTIMEOUTS timeouts = { 0 }; //value for timeoutconfig

DWORD BytesWritten_ = 0; // how many bytes that were actually written
DWORD BytesRead_ = 0; // how many bytes that were actually read

void Init_CommPort(string comport){

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
		return;
	}

}

int main(int argc, char* argv[] )
{
	Init_CommPort("COM3");

    SetupSDL* sdl_lib;
    sdl_lib = new SetupSDL;
    
    Autonom* Autonom_;
    Autonom_ = new class Autonom(sdl_lib, hComm);
    
    Manual* Manual_;
    Manual_ = new Manual(sdl_lib, hComm);
    
    string statestring{"Manual"};
    
    bool globalrunning = true;
    
    try
    {
        while (globalrunning)
        {
           if (statestring == "Manual")
           {
                Manual_ -> run(statestring);
           }
            
            if (statestring  == "Autonom")
            {
                Autonom_ -> run(statestring);
            }
            
            if (statestring  == "quit")
            {
                
                delete Autonom_;
                delete Manual_;
                delete sdl_lib;
				CloseHandle(hComm);
                
                SDL_Quit();
                
                globalrunning = false;
                return 0;
                
            }
        }
    }
    catch (...)
    {
        cout << "Ett okant fel har intraffat.\n";
    }
    
    return 0;
}
