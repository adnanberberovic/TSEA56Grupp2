//
//  SetupSDL.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#include "SetupSDL.h"
#include <iostream>

using namespace std;


SetupSDL::SetupSDL()
{
    //Setup sound
    if(Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024)==-1)
    {
        cerr << "Mix_OpenAudio: \n" << Mix_GetError() << endl;
        exit(2);
    }
    
    //Setup font
    if(!TTF_WasInit() && TTF_Init()==-1)
    {
        cerr << "TTF_Init(): \n" << TTF_GetError() << endl;;
        exit(1);
    }
    
    Mix_AllocateChannels(16);
    Mix_Volume(-1, MIX_MAX_VOLUME);
    
    
    //Create window
    window = nullptr;
    
    window = SDL_CreateWindow("ResQ_PL", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH,
                              SCREEN_HEIGHT, SDL_WINDOW_RESIZABLE);
    
    if (window == nullptr )
    {
        cout << "Window can't be created" << endl;
        exit(1);
    }
    
    
    
    //Create renderer
    renderer = SDL_CreateRenderer(window, -1, 0);
    
    //Create event
    mainevent = new SDL_Event;
    
    // Make the scaled rendering look smoother
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
}


SetupSDL::~SetupSDL()
{
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    delete mainevent;
    renderer = nullptr;
    window = nullptr;
}


SDL_Renderer* SetupSDL::get_renderer()
{
    return renderer;
}

SDL_Event* SetupSDL::get_event()
{
    return mainevent;
}



