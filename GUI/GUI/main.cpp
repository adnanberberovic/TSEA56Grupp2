//
//  main.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#include "SetupSDL.h"
#include "State.h"
#include "Autonomt.h"
#include "Manual.h"
#include <iostream>
#include <string>
#include <stdexcept>
#include <cmath>

using namespace std;

int main(int argc, char* argv[] )
{
	

    SetupSDL* sdl_lib;
    sdl_lib = new SetupSDL;
    
    Autonom* Autonom_;
    Autonom_ = new class Autonom(sdl_lib);
    
    Manual* Manual_;
    Manual_ = new Manual(sdl_lib);
    
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
