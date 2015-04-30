//
//  State.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//


#include "SetupSDL.h"
#include "State.h"
#include <Windows.h>
#include <iostream>


using namespace std;

void State::Get_Sensor_values(int8_t arrTarget[], int size, uint8_t startbit_, void* hComm_, LPDWORD BytesWritten_)
{
	char ReadBuff_[10];
	uint8_t startSensor[1] = { startbit_ };
	if (!WriteFile(hComm_, startSensor, 1, BytesWritten_, NULL)) // Send "get-sensor"
	{
		cerr << "Writing to file failed!\n";
	}
	if (!ReadFile(hComm_, arrTarget, size, BytesWritten_, NULL)){
		cerr << "Reading from file failed!\n";
	}
	if (*BytesWritten_ != size){
		cerr << "Bytes read did not match arr size! :  " << BytesWritten_ << "   " << (sizeof(arrTarget) / sizeof(arrTarget[0])) << endl;
	}
	
	if (!ReadFile(hComm_, ReadBuff_, 1, BytesWritten_, NULL)) // rid of ninja 0
	{
		cerr << "Reading from file failed!\n";
	}
}