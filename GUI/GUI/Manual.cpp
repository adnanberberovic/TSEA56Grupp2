//
//  Manual.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-21.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#include "SetupSDL.h"
#include "Manual.h"
#include <iostream>
#include <sstream>

using namespace std;

Manual::Manual(SetupSDL* sdl_lib)
{
 
    //SDL-init
    renderer_ = sdl_lib -> get_renderer();
    mainevent_ = sdl_lib -> get_event();
    
    //Konstuera Bakgrund
    Bakgrund = new Unmoveable_Object("/Users/Andreas/Skola/KP2/GUI/Bilder/Manual.png", renderer_, 0, 0);
    Pil1 = new Moveable_Object("/Users/Andreas/Skola/KP2/GUI/Bilder/Pil.png", renderer_, 300, 204);
    Pil2 = new Moveable_Object("/Users/Andreas/Skola/KP2/GUI/Bilder/Pil.png", renderer_, 300, 345);
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
            
            if (mainevent_ -> key.keysym.sym == SDLK_RIGHT)
            {
                Speed_Horizont = 1;
                
            }
            if (mainevent_ -> key.keysym.sym == SDLK_LEFT)
            {
                Speed_Horizont = -1;
                
            }
            if (mainevent_ -> key.keysym.sym == SDLK_DOWN)
            {
                Speed_Vertical = -1;
                
                
            }
            if (mainevent_ -> key.keysym.sym == SDLK_UP)
            {
                Speed_Vertical = 1;
                
            }
            if ( (mainevent_->key.keysym.sym != SDLK_RIGHT) && (mainevent_ -> key.keysym.sym != SDLK_LEFT) && (mainevent_ -> key.keysym.sym != SDLK_DOWN) && (mainevent_ -> key.keysym.sym != SDLK_UP) )
            {
                Speed = 0;
            }
            
            if (mainevent_ -> key.keysym.sym == SDLK_1)
            {
                Speed = 10/255;
                Pil1->skift_to(0);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_2)
            {
                Speed = 100/255*20;
                Pil1->skift_to(1);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_3)
            {
                Speed = 100/255*30;
                Pil1->skift_to(2);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_4)
            {
                Speed = 100/255*40;
                Pil1->skift_to(3);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_5)
            {
                Speed = 100/255*50;
                Pil1->skift_to(4);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_6)
            {
                Speed = 100/255*60;
                Pil1->skift_to(5);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_7)
            {
                Speed = 100/255*70;
                Pil1->skift_to(6);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_8)
            {
                Speed = 100/255*80;
                Pil1->skift_to(7);
            }
            else if (mainevent_ -> key.keysym.sym == SDLK_9)
            {
                Speed = 255;
                Pil1->skift_to(8);
            }
            
            if (mainevent_ -> key.keysym.sym == SDLK_q)
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


void Manual::run(SetupSDL* sdl_lib __attribute__((unused))
                  ,string& statestring) {
    
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
    
    if (front_ < 15) {
        Speed_left = 0;
        Speed_right = 0;
        return;
    }

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
        Speed_right = Speed;
        Speed_left = 0;
    }
    else if (Speed_Horizont < 0)
    {
        Speed_left = Speed;
        Speed_right = 0;
    }
    else
    {
        Speed_left = Speed;
        Speed_right = Speed;
    }


}





