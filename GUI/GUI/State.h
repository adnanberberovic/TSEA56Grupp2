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

class State
{
    
public:
    virtual void      event(std::string& statestring, bool& running) = 0;
    virtual void      update(std::string& statestring, bool& running) = 0;
    virtual void      render() = 0;
    virtual void      run(SetupSDL* sdl_lib __attribute__((unused))
                          , std::string& statestring) = 0;
    
};



#endif /* defined(__GUI__State__) */

