/*
 * Map.c
 *
 * Created: 4/23/2015 1:13:46 PM
 *  Author: robop806
 */ 


#include <avr/io.h>
#include <stdlib.h>
#include "Map.h"

// Sets the position of the goal in the map
void MAP_setGoal()
{
	int posY = MAP_currentPos[0];
	int posX = MAP_currentPos[1];
	MAP_array[posY][posX].goal = 1;
	MAP_goalPosition[0] = posY;
	MAP_goalPosition[1] = posX;
}

// Description:
// 0 = Start
// 1 = Unknown
// 2 = Uncertain
// 3 = Road
// 4 = Wall
// 5 = Intersection
void MAP_setSquareDescription(int description, int posY, int posX)
{
	if(posY <= 16 && posX <= 29) // Is square within map?
	{
		MAP_array[posY][posX].description = description;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it
int MAP_checkDirDown()
{
	if (MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].description == 3 && !MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].visited)
	{
		MAP_nextDir = 3;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it
int MAP_checkDirUp()
{
	if (MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].description == 3 && !MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].visited)
	{
		MAP_nextDir = 1;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it
int MAP_checkDirLeft()
{
	if (MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].description == 3 && !MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].visited)
	{
		MAP_nextDir = 2;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it
int MAP_checkDirRight()
{
	if (MAP_array[MAP_currentPos[0]][MAP_currentPos[1] + 1].description == 3 && !MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1] + 1].visited)
	{
		MAP_nextDir = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Decides which will be the next direction for the robot
// The different modes for choosing direction are: 'r' for right-first, 'l' for left-first or 'a' for random 
void MAP_decideDirection(char mode)
{
	char mode_ = mode;
	int robotDir = MAP_currentDir;
	
	// Randomizes the direction and mode, thus making the decisions (pseudo) random
	if (mode == 'a')
	{
		robotDir = rand() % 4;
		if (rand() % 1)
		{
			mode_ = 'l';
		} 
		else
		{
			mode_ = 'r';
		}
	}
	
	
	if (mode_ == 'l')
	{
		// Left-first
		if ( robotDir == 2 && (MAP_checkDirDown() || MAP_checkDirLeft() || MAP_checkDirUp() || MAP_checkDirRight()) )
		{} 
		else if ( robotDir == 1 && (MAP_checkDirLeft() || MAP_checkDirUp() || MAP_checkDirRight() || MAP_checkDirDown()) )
		{}
		else if (  robotDir == 0 && (MAP_checkDirUp() || MAP_checkDirRight() || MAP_checkDirDown() || MAP_checkDirLeft()) )
		{}
		else if (  robotDir == 3 && (MAP_checkDirRight() || MAP_checkDirDown() || MAP_checkDirLeft() || MAP_checkDirUp()) )
		{}
	}
	else if (mode_ == 'r')	
	{
		// Right-first
		if (  robotDir == 2 && (MAP_checkDirUp() || MAP_checkDirLeft() || MAP_checkDirDown() || MAP_checkDirRight()) )
		{}
		else if (  robotDir == 1 && (MAP_checkDirRight() ||  MAP_checkDirUp() || MAP_checkDirLeft() || MAP_checkDirDown()) )
		{}
		else if (  robotDir == 0 && (MAP_checkDirDown() || MAP_checkDirRight() ||  MAP_checkDirUp() || MAP_checkDirLeft()) )
		{}
		else if ( robotDir == 3 && (MAP_checkDirLeft() || MAP_checkDirDown() || MAP_checkDirRight() ||  MAP_checkDirUp()) )
		{}
	}
}

// Sets the current position as visited
void MAP_setVisited()
{
	MAP_array[MAP_currentPos[0]][MAP_currentPos[1]].visited = 1;
}

// Count the number of unexplored squares in a 1 square radius (up, down, left, right)
void MAP_countUnexplored()
{
	int dir_ = MAP_currentDir;
	MAP_unexploredSquares = MAP_checkDirDown() + MAP_checkDirLeft() + MAP_checkDirRight() + MAP_checkDirUp();
	MAP_currentDir = dir_;
}

// Uses Dijkstras algorithm to decide which location to set as a destination
void MAP_decideDestination()
{
	
}

int main(void)
{
	MAP_countUnexplored();
    if (MAP_unexploredSquares >= 1)
    {
		if (MAP_unexploredSquares > 1)
		{
			MAP_setSquareDescription(5, MAP_currentPos[0], MAP_currentPos[1]);
		}
    } 
    else
    {
		// Åk till föregående korsning
    }
	MAP_decideDirection('a'); // Uses random
	// Åk fram en ruta
	if (MAP_array[MAP_currentPos[0]][MAP_currentPos[1]].goal == 1)
	{
		/*Påbörja undsättningsfas*/
		
	}
	
	return 0;
}