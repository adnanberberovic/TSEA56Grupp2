//
//  Manual.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-21.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#include "SetupSDL.h"
#include "../../KONSTRUKTION/KOD/Datorvisualisering_/SDL2_BT_v0.1/SDL2_BT_v0.1/Bluetooth.h"
#include "Manual.h"
#include <iostream>
#include <sstream>

using namespace std;

Manual::Manual(SetupSDL* sdl_lib)
{
 
    //SDL-init
    renderer_ = sdl_lib -> get_renderer();
    mainevent_ = sdl_lib -> get_event();
    
    init_gfx_win();
    //init_gfx_mac();

}

Manual::~Manual()
{
	delete Pil1;
	delete Pil2;
    delete Bakgrund;

	CloseHandle(hComm);
	renderer_ = nullptr;
}

void Manual::event(string& statestring, bool& running)
{
   
    while (SDL_PollEvent(mainevent_))
    {
        
        if (mainevent_ -> type == SDL_QUIT)
        {
            statestring = "quit";
            running = false;
            return;
        }
        
        else if (mainevent_ -> type == SDL_KEYDOWN)
        {
            
            if (mainevent_ -> key.keysym.sym == SDLK_ESCAPE)
            {
                statestring = "quit";
                running = false;
                return;
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_SPACE)
            {
                statestring = "Autonom";
                running = false;
                return;
            }
  			else if (mainevent_->key.keysym.sym == SDLK_1)
            {
                Speed = 255*2/10;
                Pil1->skift_to(0);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_2)
            {
				Speed = 255*3 / 10;
                Pil1->skift_to(1);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_3)
            {
				Speed = 255*4 / 10;
                Pil1->skift_to(2);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_4)
            {
				Speed = 255 *5/ 10;
                Pil1->skift_to(3);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_5)
            {
				Speed = 255 *6/ 10;
                Pil1->skift_to(4);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_6)
            {
				Speed = 255*7/ 10;
                Pil1->skift_to(5);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_7)
            {
				Speed = 255*8 / 10;
                Pil1->skift_to(6);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_8)
            {
				Speed = 255*9/10 ;
                Pil1->skift_to(7);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_9)
            {
                Speed = 255;
                Pil1->skift_to(8);
            }
            
			else if (mainevent_->key.keysym.sym == SDLK_q)
            {
                agg = 1;
                Pil2->skift_to(0);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_w)
            {
                agg = 2;
                Pil2->skift_to(1);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_e)
            {
                agg = 3;
                Pil2->skift_to(2);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_r)
            {
                agg = 4;
                Pil2->skift_to(3);
            }
			else if (mainevent_->key.keysym.sym == SDLK_g)
			{
				Klo = 1;
			}
			else if (mainevent_->key.keysym.sym == SDLK_d)
			{
				Klo = 0;
			}

				
        }
        
    }


	if (currentKeyStates[SDL_SCANCODE_RIGHT])
	{
		Speed_Horizont = 1;
	}
	else if (currentKeyStates[SDL_SCANCODE_LEFT])
	{
		Speed_Horizont = -1;
	}
	if (currentKeyStates[SDL_SCANCODE_UP])
	{
		Speed_Vertical = 1;
	}
	else if (currentKeyStates[SDL_SCANCODE_DOWN])
	{
		Speed_Vertical = -1;
	}

	if (!(currentKeyStates[SDL_SCANCODE_RIGHT]) && !(currentKeyStates[SDL_SCANCODE_LEFT]) && !(currentKeyStates[SDL_SCANCODE_UP]) &&
		!(currentKeyStates[SDL_SCANCODE_DOWN])) {
		Speed_Vertical = 0;
		Speed_Horizont = 0;
	}

    
    return;
}

void Manual::update(string& statestring, bool& running)
{
    Set_Speed();

	uint8_t arrSpeed[] = { 1, Speed_right, Speed_left, Dir_left, Dir_right, Klo }; // Make array to send with start-flag -1
	int8_t arrSensor[] = { Sensor_val1, Sensor_val2, Sensor_val3, Sensor_val4 };

	if (!WriteFile(hComm, arrSpeed, (sizeof(arrSpeed)/sizeof(arrSpeed[0])), &BytesWritten_, NULL)) // Send array with speed values
	{
		cerr << "Writing to file failed!\n";
	}
	SDL_Delay(100);
	Get_Sensor_values(arrSensor);
	cout << static_cast<int>(Sensor_val1) << static_cast<int>(Sensor_val2) << static_cast<int>(Sensor_val3) << static_cast<int>(Sensor_val4) << endl;
}


void Manual::render()
{
    SDL_SetRenderDrawColor(renderer_ , 255, 255, 255, 255);
    SDL_RenderClear(renderer_ );

    Bakgrund->render(renderer_);
    Pil1->render_Moveable(renderer_);
    Pil2->render_Moveable(renderer_);
  

    SDL_RenderPresent(renderer_);
    SDL_Delay(10);
}


void Manual::run(string& statestring) {
    
    bool running{true};
    
    while(running)
    {
        //Event
        this -> event(statestring,running);
        
        //Update
        this -> update(statestring,running);
        
        //Render
        this -> render();
        
    }
    
    return;
}


void Manual::Set_Speed()
{
    
	Dir_left = 1;
	Dir_right = 1;

	if (abs(Speed_Horizont) > 0 && abs(Speed_Vertical) > 0)
	{
		if (Speed_Horizont > 0)
		{
			if (Speed_Vertical > 0){
				Speed_right = Speed*agg / 5;
				Speed_left = Speed;
			}
			else
			{
				Speed_right = Speed*agg / 5;
				Speed_left = Speed;
				Dir_left = 0;
				Dir_right = 0;
			}
		}
		else
		{
			if (Speed_Vertical > 0){
				Speed_left = Speed*agg / 5;
				Speed_right = Speed;
			}
			else
			{
				Speed_left = Speed*agg / 5;
				Speed_right = Speed;
				Dir_left = 0;
				Dir_right = 0;
			}
		}
	}
	else if (Speed_Horizont > 0)
	{
		Speed_left = Speed;
		Speed_right = Speed;
		Dir_left = 1;
		Dir_right = 0;

	}
	else if (Speed_Horizont < 0)
	{
		Speed_left = Speed;
		Speed_right = Speed;
		Dir_left = 0;
		Dir_right = 1;
	}
	else if (Speed_Horizont == 0 && Speed_Vertical == 0)
	{
		Speed_left = 0;
		Speed_right = 0;
	}
	else
	{
		Speed_left = Speed;
		Speed_right = Speed;
	}

	if (Speed_Vertical < 0 && Speed_Horizont == 0 )
	{
		Dir_left = 0;
		Dir_right = 0;
	}


}

void Manual::init_gfx_win()
{
    
    Init_CommPort("COM3"); //Initialize comport
    //Konstuera Bakgrund
    Bakgrund = new Unmoveable_Object("C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Manual.png", renderer_, 0, 0);
    Pil1 = new Moveable_Object("C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Pil.png", renderer_, 300, 204);
    Pil2 = new Moveable_Object("C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Pil.png", renderer_, 300, 345);
    
}

void Manual::init_gfx_mac();
{

    Bakgrund = new Unmoveable_Object(BG_plats.c_str(), renderer_, 0, 0);
    Pil1 = new Moveable_Object(Pil_plats.c_str(), renderer_, 300, 204);
    Pil2 = new Moveable_Object(Pil_plats.c_str(), renderer_, 300, 345);
    
}


	





