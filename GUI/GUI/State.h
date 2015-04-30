//
//  State.h
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#ifndef __GUI__State__
#define __GUI__State__

#include <string>
#include <Windows.h>

class State
{    
public:
	virtual void      event(std::string& statestring, bool& running) = 0;
	virtual void      update(std::string& statestring, bool& running) = 0;
	virtual void      render() = 0;
	virtual void      run(std::string& statestring) = 0;
	void Get_Sensor_values(int8_t*, int, uint8_t, void*, LPDWORD);
};


//
#endif /* defined(__GUI__State__) */

