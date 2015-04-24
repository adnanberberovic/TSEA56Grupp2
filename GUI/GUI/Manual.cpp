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
	Init_CommPort("COM3"); //Initialize comport
    //Konstuera Bakgrund
    Bakgrund = new Unmoveable_Object("C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Manual.png", renderer_, 0, 0);
    Pil1 = new Moveable_Object("C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Pil.png", renderer_, 300, 204);
    Pil2 = new Moveable_Object("C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Pil.png", renderer_, 300, 345);
}

Manual::~Manual()
{
    renderer_ = nullptr;
    delete Bakgrund;
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
            
			else if (mainevent_->key.keysym.sym == SDLK_RIGHT)
            {
                Speed_Horizont = 1;
				Speed_Vertical = 0;
                
            }
			else if (mainevent_->key.keysym.sym == SDLK_LEFT)
            {
                Speed_Horizont = -1;
				Speed_Vertical = 0;
                
            }
			else if (mainevent_->key.keysym.sym == SDLK_DOWN)
            {
                Speed_Vertical = 1;
				Speed_Horizont = 0;
                
                
            }
			else if (mainevent_->key.keysym.sym == SDLK_UP)
            {
                Speed_Vertical = 1;
				Speed_Horizont = 0;
                
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

				
        }
        
    }
    
    //Pacman1->change_Speed(Event_xSpeed, Event_ySpeed);  //ändra hastighet Detta ska in i specialklassen för robot
    
    return;
}

void Manual::update(string& statestring, bool& running)
{
    Set_Speed();
	cerr << static_cast<int>(Speed_left) << "<<<left  Right>>>>>" << static_cast<int>(Speed_right) << '\n';
	Speed_left = 1;
	Speed_right = 254;
	uint8_t arrSpeed[] = { 1, Speed_left, Speed_right }; // Make array to send with start-flag -1

	if (!WriteFile(hComm, arrSpeed, 3, &BytesWritten_, NULL)) // Send array with speed values
	{
		cerr << "Writing to file failed!\n";
	}
	SDL_Delay(50);

}


void Manual::render()
{
    SDL_SetRenderDrawColor(renderer_ , 255, 255, 255, 255);
    SDL_RenderClear(renderer_ );

    Bakgrund->render(renderer_);
    Pil1->render_Moveable(renderer_);
    Pil2->render_Moveable(renderer_);
  

    SDL_RenderPresent(renderer_);
    SDL_Delay(50);
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
    
    //if (front_ < 15) {
    //    Speed_left = 0;
    //    Speed_right = 0;
    //    return;
    //}

    if(abs(Speed_Horizont) > 0 &&  abs(Speed_Vertical) > 0)
    {
        if (Speed_Horizont > 0)
        {
            Speed_right = Speed*agg/5;
            Speed_left = Speed;
        }
        else
        {
            Speed_left = Speed*agg/5;
            Speed_right = Speed;
        }
    }
    else if (Speed_Horizont > 0)
    {
        Speed_right = 0;
		Speed_left = Speed;
    }
    else if (Speed_Horizont < 0)
    {
		cerr << "Vi ar i vanster\n";
        Speed_left = 0;
		Speed_right = Speed;
    }
    else
    {
        Speed_left = Speed;
        Speed_right = Speed;
    }

	
}





