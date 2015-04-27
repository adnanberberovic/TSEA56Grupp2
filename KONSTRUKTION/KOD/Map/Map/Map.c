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
// 5 = Junction
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
void MAP_countUnexploredSquares()
{
	int dir_ = MAP_currentDir;
	MAP_unexploredSquares = MAP_checkDirDown() + MAP_checkDirLeft() + MAP_checkDirRight() + MAP_checkDirUp();
	MAP_currentDir = dir_;
}

// Uses Dijkstras algorithm to decide which location to set as a destination
void MAP_decideDestination()
{
	
}

// Adds the travelled distance from the last junction to specified junction
void MAP_addJunctionDist(int junction)
{
	if (junction > MAP_currentJunction)
	{
		MAP_junctionDistArray[MAP_currentJunction][junction] = MAP_travelledDist;
	}
	else
	{
		MAP_junctionDistArray[junction][MAP_currentJunction] = MAP_travelledDist;
	}
}

// Adds the current square as a junction
void MAP_addJunction()
{
	posY_ = MAP_currentPos[0];
	posX_ = MAP_currentPos[1];

	// Adds it in the map array
	MAP_setSquareDescription(5, posY_, posX_);

	// Increments counters
	MAP_currentJunction++;
	MAP_junctionCount++;
	
	// Adds it in the junction array
	MAP_junctionOrderArray[MAP_currentJunction].hasUnex = 1;
	MAP_junctionOrderArray[MAP_currentJunction].posY = posY_;
	MAP_junctionOrderArray[MAP_currentJunction].posX = posX_;

	// Adds the distance in the distance array
	if (MAP_currentJunction > 0)
	{
		MAP_addJunctionDist(MAP_currentJunction - 1);
	}
		
}

// Returns the last junction with unexplored roads
// Call with junctionCount
int MAP_lastUnexJunction(int x)
{
	if (x == 0)
	{
		/* Kartan är helt utforskad!!! */
		return -1;
	}
	else if (MAP_junctionOrderArray[x - 1].hasUnex == 1)
	{
		return x - 1;
	}
	else
	{
		MAP_lastUnexJunction(x - 1);
	}
}

// Kodskelett för kartläggning
/*
TODO
-fallet där korsningen man kommer till har besökts tidigare
*/
int main(void)
{
	MAP_countUnexploredSquares();
    if (MAP_unexploredSquares >= 1)
    {
		if (MAP_unexploredSquares > 1)
		{
			MAP_addJunction();
		}

    } 
    else
    {
		// Åk till föregående korsning - Egen fas
    }
	MAP_decideDirection('a'); // Uses random
	if (MAP_currentDir != MAP_nextDir)
	{
		/* Rotera lämpligt - Egen fas
			-ändra currentDir*/
	}
	/* TODO Åk fram en ruta - Egen fas
		-räkna upp travelledDist
		-ändra currentPos*/
	if (MAP_array[MAP_currentPos[0]][MAP_currentPos[1]].goal == 1)
	{
		// Påbörja undsättning - Egen fas
		
	}
	
	return 0;
}
