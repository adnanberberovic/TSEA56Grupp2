//
//  Manual.h
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-21.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#ifndef __GUI__Manual__
#define __GUI__Manual__

#include <stdio.h>
#ifdef __APPLE__
#include <SDL2/SDL.h>
#include <SDL2_image/SDL_image.h>
#include <SDL2_ttf/SDL_ttf.h>
#include <SDL2_mixer/SDL_mixer.h>
#endif

#ifdef __linux__
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_mixer.h>
#endif


#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <SDL_mixer.h>



#include <string>
#include <vector>
#include <stdio.h>
#include "State.h"
#include "Objects.h"


using namespace std;



class Manual: public State
{

public:
    
    Manual(SetupSDL* sdl_lib);
    ~Manual();
    
    virtual void      event(string& statestring,bool& running) override;
    virtual void      update(string& statestring,bool& running)  override; //i cc - object_vector -> update()
    virtual void      render()  override; //i cc - object_vector -> render()
    virtual void      run(string& statestring) override;
    void Set_Speed();
    
private:

    SDL_Renderer* renderer_;
    SDL_Event* mainevent_;

    Unmoveable_Object* Bakgrund;
    Moveable_Object* Pil1;
    Moveable_Object* Pil2;
    
    //Inerna IN --> UT
    int8_t Speed_Horizont;
    int8_t Speed_Vertical;
    uint8_t Speed;
    uint8_t Speed_right;
    uint8_t Speed_left;
    int8_t agg;
    
    //Externa IN <-- UT
    int8_t angle_;
    int8_t offset_;
    int8_t reflex_;
    int8_t space_right_;
    int8_t space_left_;
    int8_t front_ = 20;
    
    
};



#endif /* defined(__GUI__Manual__) */
