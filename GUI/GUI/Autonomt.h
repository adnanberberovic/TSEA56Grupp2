//
//  Autonomt.h
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#ifndef __GUI__Autonomt__
#define __GUI__Autonomt__


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
#include <	SDL_image.h>
#include <SDL_ttf.h>
#include <SDL_mixer.h>


#include <string>
#include <vector>
#include <stdio.h>
#include "State.h"
#include "Objects.h"

using namespace std;


class Autonom: public State
{
private:
    
    Uint32 CurrTime_; // oKLARA
    Uint32 StartTime_ ;
    
    SDL_Renderer* renderer_;
    SDL_Event* mainevent_;
    
    
    int Event_xSpeed;
    int Event_ySpeed;
    
    int Speed = 1;
    int Offset_ = 0;
    
    int Position = 0;
    
    Text* Text_Angle_;
    Text* Text_Offset_;
    
    Unmoveable_Object* Bakgrund;
    
    Speed_meter* Speed_left;
    Speed_meter* Speed_right;
    
    Robot* Robot_angle;
    Robot* Robot_offset;
    Robot* Robot_Rotaton;
    Robot* Robot_Bana;

    vector< vector<Tile*> > Tile_vector;
    
    //Platser Texturer
    //Mac
//    string Robot_plats = "/Users/Andreas/Skola/KP2/GUI/Bilder/Robot.png";
//    string Tiles_plats = "/Users/Andreas/Skola/KP2/GUI/Bilder/vag.png";
//    string Font = "/Users/Andreas/Library/Fonts/DS-DIGI.TTF";
//    string SM_plats = "/Users/Andreas/Skola/KP2/GUI/Bilder/Hastighet.png";
//    string BG_plats = "/Users/Andreas/Skola/KP2/GUI/Bilder/BG.png";
    
    
    //Windows
        string Robot_plats = "C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Robot.png";
    
        string Tiles_plats = "C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/vag.png";
    
        string Font = "C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/ds_digital/DS-DIGI.TTF";
    
        string SM_plats = "C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/Hastighet.png";
    
        string BG_plats = "C:/Users/Måns/Documents/GitHub/TSEA56Grupp2/GUI/Bilder/BG.png";
    
    
    
public:
    Autonom(SetupSDL* sdl_lib);
    ~Autonom();
    
    virtual void      event(string& statestring,bool& running) override;
    virtual void      update(string& statestring,bool& running)  override; //i cc - object_vector -> update()
    virtual void      render()  override; //i cc - object_vector -> render()
    virtual void      run(string& statestring) override;
    
    const char* Time_to_char(Uint32);
    
    void Update_Game(string& statestring,bool& running);

    void Start_Timer();
    void Update_Timer();

    void init_Tiles();
    void Render_Tiles_Helper();
    void delete_Tiles();
    void Skift_Tiles_left();
    void Skift_Tiles_right();
    void Change_left_speed();
    void Change_right_speed();
    
};



#endif /* defined(__GUI__Autonomt__) */
