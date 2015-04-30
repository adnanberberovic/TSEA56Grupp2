
//  Objects.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#include "Objects.h"
#include <iostream>
#include <string>
#include <math.h>
using namespace std;
//
//
// Object
//

Object::~Object()
{
    SDL_DestroyTexture(Texture_);
    Texture_= nullptr;
}

SDL_Rect Object::get_Rect()
{
    return Rect_;
}

SDL_Texture* Object::get_Texture()
{
    return Texture_;
}

void Object::render(SDL_Renderer* renderer)
{
    SDL_RenderCopy(renderer, Texture_, nullptr, &Rect_ );
}

void Object::init_Rect_Texture(string file_texture, SDL_Renderer* renderer, int xPos, int yPos)
{
    
    SDL_Surface* temp = IMG_Load(file_texture.c_str());
	if (temp == nullptr)
	{
		cerr << "Fucka you!\n";
		return;
	}
	Texture_Width = temp->w;
    Texture_Height = temp->h;
    Texture_ = SDL_CreateTextureFromSurface(renderer, temp);
    SDL_FreeSurface(temp);
    
    Rect_.x = xPos;
    Rect_.y = yPos;
    Rect_.w = Texture_Width;
    Rect_.h = Texture_Height;
}

//
//
// Unmoveable-Object
//

Unmoveable_Object::Unmoveable_Object(std::string file_texture,
                                     SDL_Renderer* renderer, int xPos, int yPos)
{
    init_Rect_Texture(file_texture, renderer, xPos, yPos);
}


Moveable_Object::Moveable_Object(std::string file_texture,
                                     SDL_Renderer* renderer, int xPos, int yPos)
{
    init_Rect_Texture(file_texture, renderer, xPos, yPos);
    dsRect_ = {xPos,yPos,37,32};
    srRect_ = {0,0,37,32};
    xPos_ = xPos;
    yPos_ = yPos;
}


void Moveable_Object::render_Moveable(SDL_Renderer* renderer)
{
    
    SDL_RenderCopyEx(renderer, Texture_, &srRect_, &dsRect_, NULL, NULL, flip);
    
}

void Moveable_Object::skift_to(int to)
{
    Rect_.x =  xPos_ + 80*to;
    dsRect_.x = xPos_ + 80*to;
}



//
// Speed_meter
//

Speed_meter::Speed_meter(std::string file_texture, SDL_Renderer* renderer, int xPos, int yPos)
{
    init_Rect_Texture(file_texture, renderer, xPos, yPos);
    dsRect_ = {xPos,yPos,60,140};
    srRect_ = {0,0,60,140};
    xPos_ = xPos;
    yPos_ = yPos;
}


void Speed_meter::Change_speed(int Speed)
{
    dsRect_ = {xPos_,yPos_,60,140};
    srRect_ = {0,0,60,140};
    
    if(Speed > 255)
        Speed = 255;
    if(Speed < 0)
        Speed = 0;
    
    srRect_.y =  140 - 140*Speed/255 ;
    dsRect_.y = dsRect_.y + 140 - 140*Speed/255 ;
    dsRect_.h = 140*Speed/255 ;
    
}

void Speed_meter::render_Speed(SDL_Renderer* renderer)
{
   
    SDL_RenderCopyEx(renderer, Texture_, &srRect_, &dsRect_, NULL, NULL, flip);

}


//
//
// Tile
//


Tile::Tile(std::string file_texture, SDL_Renderer* renderer, int xPos, int yPos)
{
    init_Rect_Texture( file_texture, renderer, xPos, yPos);
    Status_ = 0;
    dsRect_ = {xPos,yPos,40,40};
    srRect_ = {0,0,40,40};

}


void Tile::render_Tile(SDL_Renderer* renderer)
{
    if(Status_ == 0){
        return;
    }
    if (Status_ > 0) {
        SDL_RenderCopyEx(renderer, Texture_, &srRect_,
                         &dsRect_, NULL, NULL, flip);
    }
    else
    return;
}

int Tile::get_Status()
{
    return Status_;
}

void Tile::make_Reset()
{
    Status_ = 0;
    srRect_ = {0,0,40,40};
}

void Tile::make_Path()
{
    Status_ = 1;
    srRect_ = {40,0,40,40};
    
}

void Tile::make_Wall()
{
    Status_ = 2;
    srRect_ = {80,0,40,40};
}

void Tile::skift_left()
{
    Rect_.x =  Rect_.x - 40;
    dsRect_.x = dsRect_.x - 40;
}

void Tile::skift_right()
{
    Rect_.x =  Rect_.x + 40;
    dsRect_.x = dsRect_.x + 40;
}

//
// Text
//

Text::Text(const char* font_file, const char* text_print, SDL_Renderer* renderer, int xPos,int yPos,int w,int h)
    {
    
    FontFile_ = font_file;
    fontcolor = { 0 , 0, 0, 255};
    font = TTF_OpenFont(font_file, 50);
        
                SDL_Surface* text_surface;
                text_surface = TTF_RenderText_Blended(font, text_print, fontcolor);
                Texture_ = SDL_CreateTextureFromSurface(renderer, text_surface);
                SDL_FreeSurface(text_surface);
        
        
    Rect_ = {xPos, yPos, w, h};
        xPos_ = xPos;
    
}

Text::~Text()
{
       TTF_CloseFont(font);
}

void Text::Update_Texture(string text_print,SDL_Renderer* renderer)
{
    
        int size = text_print.size();
    
        SDL_DestroyTexture(Texture_);
        SDL_Surface* text_surface;
        text_surface = TTF_RenderText_Blended(font, text_print.c_str(), fontcolor);
        Texture_ = SDL_CreateTextureFromSurface(renderer, text_surface);
        SDL_FreeSurface(text_surface);
        Rect_.w = size*20;
    Rect_.x = xPos_ - (size-1)*20;
 
}

const char* Text::Get_Font()
    {
    return FontFile_;
}

//
// Robot
//
//


Robot::Robot(std::string file_texture,SDL_Renderer* renderer,int xPos, int yPos)
{
    init_Rect_Texture( file_texture, renderer, xPos, yPos);
    init_Direction_Tile(xPos, yPos);
    init_Rotate(xPos,yPos,20);
    
    angle_ = 0;
    offset_ = 0;
    xPos_ = xPos;
    yPos_ = yPos;

}



void Robot::set_angle(int new_angle)
{
	
    angle_ = new_angle*(-1);
}


void Robot::render1(SDL_Renderer* renderer)
{
    
    dsRect_.x = xPos_ + offset_;
    
    SDL_RenderCopyEx(renderer, Texture_, &srRect_,
                     &dsRect_, angle_, nullptr, flip);
}


void Robot::init_Rotate(int xPos, int yPos, int jump)
{
    dsRect_ = {xPos, yPos, 100, 120};
    srRect_ = {0, 0, 100, 120};
}


void Robot::init_Direction_Tile(int xPos,
                                int yPos)
{
    xRect_ = xPos;
    yRect_ = yPos;
    
}



int Robot::get_xDirection()
{
    return xDirection_;
}

int Robot::get_yDirection()
{
    return yDirection_;
}


void Robot::change_Direction(int xDirection, int yDirection)
{
    xDirection_ = xDirection;
    yDirection_ = yDirection;
}


void Robot::change_Rect(int x, int y)
{
    Rect_.x += x;
    dsRect_.x += x;
    
    Rect_.y += y;
    dsRect_.y += y;
    
}

void Robot::change_offset(int offset){
    
	offset = offset - 20;
    if(offset > 10)
        offset = 10;
    if(offset < -10)
        offset = -10;
    
    
    offset_ = offset*3;
    
}

string Robot::get_Offset()
{
    int OS3 = offset_/3;
    string offset_string = to_string(OS3);
    return offset_string;
}

string Robot::get_Angle()
{
    int AG = static_cast<int>(angle_);
    string angle_string = to_string(AG);
    return angle_string;
}







