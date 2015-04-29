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
	init_text();
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

	Speed_Vertical = 0;
	Speed_Horizont = 0;

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


    
    return;
}

void Manual::update(string& statestring, bool& running)
{

	Set_Speed();

	uint8_t arrSpeed[] = { 1, Speed_right, Speed_left, Dir_left, Dir_right, Klo }; // Make array to send with start-flag -1
	Get_Sensor_values(arrSensor, (sizeof(arrSensor) / sizeof(arrSensor[0])));
	if (!WriteFile(hComm, arrSpeed, (sizeof(arrSpeed) / sizeof(arrSpeed[0])), &BytesWritten_, NULL)) // Send array with speed values
	{
		cerr << "Writing to file failed!\n";
	}
	SDL_Delay(10);
	
	update_text(arrSensor[0], arrSensor[1], arrSensor[2], ((0x40 & arrSensor[3]) >> 6), (0x07 & arrSensor[3]) /*CR*/, ((0x38 & arrSensor[3]) >> 3)/*CL*/);
}

void Manual::render()
{
    SDL_SetRenderDrawColor(renderer_ , 255, 255, 255, 255);
    SDL_RenderClear(renderer_ );

    Bakgrund->render(renderer_);
    Pil1->render_Moveable(renderer_);
    Pil2->render_Moveable(renderer_);
	render_text();

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
        
		//Mode
		Check_Mode(statestring, running);

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

void Manual::init_gfx_mac()
{

    Bakgrund = new Unmoveable_Object(BG_plats.c_str(), renderer_, 0, 0);
    Pil1 = new Moveable_Object(Pil_plats.c_str(), renderer_, 300, 204);
    Pil2 = new Moveable_Object(Pil_plats.c_str(), renderer_, 300, 345);
    
}

void Manual::init_text()
{
	Text_Angle_ = new Text(Font_.c_str(), "02", renderer_, 293, 420, 50, 50);
	Text_Offset_ = new Text(Font_.c_str(), "02", renderer_, 293, 479, 50, 50);
	Text_Reflex_ = new Text(Font_.c_str(), "02", renderer_, 293, 538, 50, 50);
	Text_Front = new Text(Font_.c_str(), "02", renderer_, 293, 596, 50, 50);
	Text_Clear_right_ = new Text(Font_.c_str(), "02", renderer_, 293, 655, 50, 50);
	Text_Clear_left_ = new Text(Font_.c_str(), "02", renderer_, 293, 713, 50, 50);;

}

void Manual::render_text()
{
	Text_Angle_->render(renderer_);
	Text_Offset_->render(renderer_);
	Text_Reflex_->render(renderer_);
	Text_Front->render(renderer_);
	Text_Clear_right_->render(renderer_);
	Text_Clear_left_->render(renderer_);
}

void Manual::update_text(int A, int O, int F, int R, int CR, int CL)
{

	Text_Angle_->Update_Texture(to_string(A), renderer_);
	Text_Offset_->Update_Texture(to_string(O), renderer_);
	Text_Reflex_->Update_Texture(to_string(R), renderer_);
	Text_Front->Update_Texture(to_string(F), renderer_);
	Text_Clear_right_->Update_Texture(to_string(CR), renderer_);
	Text_Clear_left_->Update_Texture(to_string(CL), renderer_);

}
	
void Manual::Check_Mode(string &statestring, bool &running)
{
	if (arrSensor[3] >= 0)
	{
		statestring = "Autonom";
		running = false;
		return;
	}

}





