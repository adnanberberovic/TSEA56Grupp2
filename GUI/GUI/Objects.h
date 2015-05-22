//
//  Objects.h
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#ifndef __GUI__Objects__
#define __GUI__Objects__


#ifdef __APPLE__
#include <SDL2/SDL.h>
#include <SDL2_image/SDL_image.h>
#include <SDL2_ttf/SDL_ttf.h>
#include <SDL2_mixer/SDL_mixer.h>
#endif

#ifdef __linux__
#include <SDL2/SDL2.h>
#include <SDL2/SDL2_image.h>
#include <SDL2/SDL2_ttf.h>
#include <SDL2/SDL2_mixer.h>
#endif


#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <SDL_mixer.h>


#include <string>
#include <vector>


class Object
{
    
public:
    
    ~Object();
    SDL_Rect get_Rect();
    SDL_Texture* get_Texture();
    void render(SDL_Renderer*);
    void init_Rect_Texture(std::string, SDL_Renderer*, int, int);
    
protected:
    
    SDL_Texture* Texture_{nullptr};
    int Texture_Width; // in till konstuktorn
    int Texture_Height; // same bara temp.
    SDL_Rect Rect_;
	SDL_RendererFlip flip = SDL_FLIP_NONE;
    
};


//
// Unmoveable Object
//

class Unmoveable_Object : public Object
{
    
public:
    
    Unmoveable_Object(std::string, SDL_Renderer*, int, int);
    ~Unmoveable_Object() = default;
    
};


//
// Movable Object
//

class Moveable_Object : public Object
{

public:
    Moveable_Object(std::string, SDL_Renderer*, int, int);
    ~Moveable_Object() = default;
    void skift_to(int);
    void render_Moveable(SDL_Renderer*);
    
private:
    SDL_Rect dsRect_;
    SDL_Rect srRect_;
   // SDL_RendererFlip flip{};
    int xPos_;
    int yPos_;
};

//
// Speed
//

class Speed_meter : public Object
{
public:
    Speed_meter(std::string, SDL_Renderer*, int, int);
    ~Speed_meter() = default;
    
    void Change_speed(int);
    void render_Speed(SDL_Renderer*);

    
private:
    int Speed;
    SDL_Rect dsRect_;
    SDL_Rect srRect_;
    //SDL_RendererFlip flip{};
    int xPos_;
    int yPos_;
    

    
    
    
};

//
// Tile
//
//

class Tile : public Object // Om vi kör virtuall, kan man byta till unmoveable
{
public:
    
    Tile(std::string, SDL_Renderer*, int, int);
    ~Tile() = default;
        
    void make_Wall();
    void make_Path();
    void make_Reset();
    void skift_left();
    void skift_right();
	void change_tile(int to_status);
    
    void render_Tile(SDL_Renderer*);
    int get_Status();
    
    
protected:
    
    int Status_;
    SDL_Rect dsRect_;
    SDL_Rect srRect_;
    //SDL_RendererFlip flip{};
    
    
};


class Text : public Object
{
public:
    Text(const char*, const char*, SDL_Renderer*, int, int, int, int);
    ~Text();
    void Update_Texture(std::string, SDL_Renderer*);
    const char* Get_Font();
    
private:
    const char* FontFile_;
    //const char* PrevText_{};
    TTF_Font* font;
    SDL_Color fontcolor;
    int xPos_;
};


class Robot : public Object
{
    
public:
    
    void set_angle(int);
    void update_sprite(int);
    void render1(SDL_Renderer*);
    void init_Rotate(int,int,int);
    
    Robot(std::string, SDL_Renderer*, int, int); // Kanske ska bort
    ~Robot() = default;
    
    //Startar upp hastighet och tile
    void init_Direction_Tile(int, int); //Ny få upp i konstruktorerna
    
    // Get-funktioner
    int get_xDirection();
    int get_yDirection();
    std::string get_Offset();
    std::string get_Angle();

    
    //Change-funktioner
    void change_Direction(int, int);
    void change_Rect(int, int);
    void change_offset(int);

    
protected:
    
    SDL_RendererFlip flip{};
    double angle_;
    Uint32 sprite_numb = SDL_GetTicks() % 3;
    SDL_Rect dsRect_;
    SDL_Rect srRect_;
    int xPos_;
    int yPos_;
    
    
    int xDirection_;
    int yDirection_;
    int Direction_;
    int xRect_;
    int yRect_; // Grund-hastighet
    
    int offset_;
    
};


#endif /* defined(__GUI__Objects__) */
