//
//  Autonomt.cpp
//  GUI
//
//  Created by Andreas Brorsson on 2015-04-17.
//  Copyright (c) 2015 Andreas Brorsson. All rights reserved.
//

#include "SetupSDL.h"
#include "Autonomt.h"
#include <iostream>
#include <sstream>
#include <windows.h>
//#include <string.h>


using namespace std;


//Konstruktor
Autonom::Autonom(SetupSDL* sdl_lib, void* hComm_)
{
    
    //SDL-init
    renderer_ = sdl_lib -> get_renderer();
    mainevent_ = sdl_lib -> get_event();
	hComm = hComm_;
    //Konstuera Bakgrund
    Bakgrund = new Unmoveable_Object(BG_plats.c_str(), renderer_, 0, 0);
    
    Speed_left = new Speed_meter(SM_plats.c_str(), renderer_, 150 , 444);
    Speed_right = new Speed_meter(SM_plats.c_str(), renderer_, 274, 444);
    
    //Konstruera Robotar
    Robot_angle = new Robot(Robot_plats.c_str(), renderer_, 194, 50 );
    Robot_offset = new Robot(Robot_plats.c_str(), renderer_, 192, 250);
    Robot_Rotaton = new Robot(Robot_plats.c_str(), renderer_, 194, 660);
    
    //Text
    Text_Angle_ = new Text(Font.c_str(), "01", renderer_, 80, 20, 50, 50);
    Text_Offset_ = new Text(Font.c_str(), "02", renderer_, 80, 224, 50, 50);
    
    //väg/vägg
    init_Tiles();
    
    //(Tile_vector[x][y])->make_Wall();
    //(Tile_vector[15 ][0])->make_Wall();

	arrMap[0] = 0x79;
	arrMap[1] = 0x78;
	arrMap[2] = 0x88;
	(Tile_vector[2][0])->change_tile(1);
	(Tile_vector[2][0])->change_tile(2);
    
    Speed_left->Change_speed(140);
    Speed_right->Change_speed(300);
  
}

//Destruktor (INTE KLAR)
Autonom::~Autonom()
{
    
    delete_Tiles();
    
    delete Robot_angle;
    delete Robot_Rotaton;
    delete Robot_offset;
    
    delete Text_Angle_;
    delete Text_Offset_;
    
    delete Speed_right;
    delete Speed_left;
    delete Bakgrund;
    
    renderer_ = nullptr;
}

void Autonom::event(string& statestring, bool& running)
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
                statestring = "Manual";
                running = false;
                return;
            }
            
           /* else if (mainevent_ -> key.keysym.sym == SDLK_RIGHT)
            {
                Offset_ = Offset_ + 1;
                Robot_angle->set_angle(90);
                Skift_Tiles_right();
                Robot_offset->change_offset(Offset_);
            }
            
            else if (mainevent_ -> key.keysym.sym == SDLK_LEFT)
            {
                Offset_ = Offset_ - 1;
                Robot_angle->set_angle(-90);
                Skift_Tiles_left();
                Robot_offset->change_offset(Offset_);

            }
            
            else if (mainevent_ -> key.keysym.sym == SDLK_DOWN)
            {

                Robot_angle->set_angle(180);
                Speed = Speed - 4;
                Robot_angle->set_angle(0);
                Speed_left->Change_speed(Speed);
                Speed_right->Change_speed(Speed);

                
            }*/
            
            else if (mainevent_ -> key.keysym.sym == SDLK_UP)
            {
                Speed = Speed + 4;
                Robot_angle->set_angle(0);
                Speed_left->Change_speed(Speed);
                Speed_right->Change_speed(Speed);
				reset_Tiles();

            }
            
        }
        
    }
    
    //Pacman1->change_Speed(Event_xSpeed, Event_ySpeed);  //ändra hastighet Detta ska in i specialklassen för robot
    
    return;
}


//Update
void Autonom::update(string& statestring, bool& running)
{
	DWORD BytesWritten_;
	DWORD BytesRead_;
	Get_Sensor_values(arrSensor, (sizeof(arrSensor) / sizeof(arrSensor[0])), 87, hComm, &BytesRead_);
	SDL_Delay(20);
	//Get_Sensor_values(arrSpeed, (sizeof(arrSpeed) / sizeof(arrSpeed[0])), 89, hComm, &BytesRead_);
	//SDL_Delay(20);
	Get_Sensor_values(arrMap, (sizeof(arrMap) / sizeof(arrMap[0])), 69, hComm, &BytesRead_);
	SDL_Delay(20);
	update_texture(arrSensor[0], arrSensor[1], ((0x40 & arrSensor[3]) >> 6), arrSpeed[0], arrSpeed[1]);
	update_map(arrMap[0], arrMap[1], arrMap[2]);

}

void Autonom::render()
{
    
    SDL_SetRenderDrawColor(renderer_ , 255, 255, 255, 255);
    SDL_RenderClear(renderer_ );
    
    //Unmoveble
    Bakgrund->render(renderer_);
    Speed_left->render_Speed(renderer_);
    Speed_right->render_Speed(renderer_);
    
    //Text helper
    Text_Angle_->render(renderer_);
    Text_Offset_->render(renderer_);
    
    //Tile helper
    
    //Robot
    Robot_angle->render1(renderer_);
    Robot_offset->render1(renderer_);
    Robot_Rotaton->render1(renderer_);
    
    Render_Tiles_Helper();
    
    
    SDL_RenderPresent(renderer_);
    SDL_Delay(50);
}


//Oklar
const char* Autonom::Time_to_char(Uint32 ToPrint){
    stringstream sstr;
    sstr << ToPrint;
    return sstr.str().c_str();
}


//Timer
void Autonom::Start_Timer(){
    StartTime_ = SDL_GetTicks()/1000; //Starttid i sekunder
    CurrTime_ = SDL_GetTicks()/1000 - StartTime_; //Curr time s‰tts till 0;
}

void Autonom::Update_Timer(){
    CurrTime_ = SDL_GetTicks()/1000 - StartTime_;
}

void Autonom::init_Tiles()
{
    for (int i = 30; i >= 0; i --){
        vector<Tile*> row;

        for (int j = 17; j >= 0; j--)
        {
            row.insert(row.begin(),
                       new Tile(Tiles_plats,
                                renderer_, 244+40*i, 61+40*j));
        }
        Tile_vector.insert(Tile_vector.begin(), row);
    }
}

void Autonom::Render_Tiles_Helper()
{
    
    
    for (vector< vector<Tile*> >::iterator vec = Tile_vector.begin(); vec != Tile_vector.end(); ++vec){
        for ( vector<Tile*>::iterator i = (*vec).begin(); i != (*vec).end(); ++i){
            
        (*i)->render_Tile(renderer_);
        }
        
    }
    
}

void Autonom::delete_Tiles()
{
    
    for (vector< vector<Tile*> >::iterator vec = Tile_vector.begin(); vec != Tile_vector.end(); ++vec){
        
        for ( vector<Tile*>::iterator i = (*vec).begin(); i != (*vec).end(); ++i){
            
            delete (*i);
        }
        
    }
    
}

void Autonom::reset_Tiles()
{

	for (vector< vector<Tile*> >::iterator vec = Tile_vector.begin(); vec != Tile_vector.end(); ++vec){

		for (vector<Tile*>::iterator i = (*vec).begin(); i != (*vec).end(); ++i){

			(*i)->make_Reset();
		}

	}

}

void Autonom::Skift_Tiles_left()
{
    if (Position > -7){
        
        Position = Position - 1;
        
    for (vector< vector<Tile*> >::iterator vec = Tile_vector.begin(); vec != Tile_vector.end(); ++vec){
        
        for ( vector<Tile*>::iterator i = (*vec).begin(); i != (*vec).end(); ++i){
            
            (*i)->skift_left();
            }
        }
    }
    else{
        return;
    }
    

}

void Autonom::Skift_Tiles_right()
{
    if (Position < 7){
        
                Position = Position + 1;
        
        for (vector< vector<Tile*> >::iterator vec = Tile_vector.begin(); vec != Tile_vector.end(); ++vec){
        
        for ( vector<Tile*>::iterator i = (*vec).begin(); i != (*vec).end(); ++i){
            
            (*i)->skift_right();
            }
        }
    }
    else{
        return;
    }
}

void Autonom::Change_right_speed()
{


}
void Autonom::Change_left_speed()
{


}

void Autonom::run(string& statestring) {
    Start_Timer();
    
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

void Autonom::Check_Mode(string &statestring, bool &running)
{
	if (arrSensor[6] < 0)
	{
		statestring = "Manual";
		running = false;
		reset_Tiles(); 
		return;
	}

}

void Autonom::update_texture(int A, int O, int Re, int L, int R)
{
	Text_Offset_->Update_Texture(to_string(O - 20), renderer_);
	Text_Angle_->Update_Texture(to_string(A), renderer_);
	Robot_angle->set_angle(A);
	Robot_offset->change_offset(O);
	Speed_left->Change_speed(L);
	Speed_right->Change_speed(R);
}

void Autonom::update_map(int8_t xPosD, int8_t yPosM, int8_t LFR)
{
    (Tile_vector[xPos][yPos])->Place_Robot_Here(0);
    
	xPos = ((xPosD & 0xf8) / 8) + 0;
	yPos = ((yPosM & 0xf8) / 8) + 0;
	dir =   (xPosD & 0x03) + 0;
    goal = (yPosM & 0x03) + 0;
    Left = ((LFR & 0xc0) / 64) + 0;
	Front = ((LFR & 0x30) / 16) + 0;
	Right = ((LFR & 0x0c) / 4) + 0;
  

	if (xPos >= (22 + shift_right))
	{
		Skift_Tiles_left();
		shift_right++;
	}
	if (xPos <= (8 - shift_left))
	{
		Skift_Tiles_right();
		shift_left++;
	}

	(Tile_vector[xPos][yPos])->Place_Robot_Here(1);
    
    if(goal)
    {
        (Tile_vector[xPos][yPos])->make_Goal();
    }

	if ((xPos >= 0) && (xPos <= 30) && (yPos >= 0) && (yPos <= 16))
	{
		if (dir == 0)
		{
			(Tile_vector[xPos][yPos - 1])->change_tile(Left);
			(Tile_vector[xPos + 1][yPos])->change_tile(Front);
			(Tile_vector[xPos][yPos + 1])->change_tile(Right);
		}
		else if (dir == 1)
		{
			(Tile_vector[xPos - 1][yPos])->change_tile(Left);
			(Tile_vector[xPos][yPos - 1])->change_tile(Front);
			(Tile_vector[xPos + 1][yPos])->change_tile(Right);
		}
		else if (dir == 2)
		{
			(Tile_vector[xPos][yPos + 1])->change_tile(Left);
			(Tile_vector[xPos - 1][yPos])->change_tile(Front);
			(Tile_vector[xPos][yPos - 1])->change_tile(Right);
		}
		else if (dir == 3)
		{
			(Tile_vector[xPos + 1][yPos])->change_tile(Left);
			(Tile_vector[xPos][yPos + 1])->change_tile(Front);
			(Tile_vector[xPos - 1][yPos])->change_tile(Right);
		}
	}

    if (dir == 0)
    {
        Robot_Rotaton->set_angle(270);
    }
    else if (dir == 1)
    {
        Robot_Rotaton->set_angle(0);
    }
    else if (dir == 2)
    {
        Robot_Rotaton->set_angle(90);
    }
    else if (dir == 3)
    {
        Robot_Rotaton->set_angle(180);
    }
    
    
}








//By Bsoson



