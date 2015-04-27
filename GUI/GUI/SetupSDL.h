//
//  SetupSDL.h
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#ifndef __GUI__SetupSDL__
#define __GUI__SetupSDL__
#endif /* defined(__GUI__SetupSDL__) */


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


#include <sstream>


class SetupSDL
{
private:
	SDL_Window* window;
	SDL_Renderer* renderer;
	SDL_Event* mainevent;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 800;
    
    
public:
    
    SetupSDL();
    ~SetupSDL();
    
	SDL_Renderer* get_renderer();
	SDL_Event* get_event();
    
};
