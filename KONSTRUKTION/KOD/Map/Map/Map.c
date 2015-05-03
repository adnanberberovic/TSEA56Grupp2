/*
 * Map.c
 *
 * Created: 4/23/2015 1:13:46 PM
 *  Author: robop806
 */ 


#include <avr/io.h>
#include <stdlib.h>
#include "Map.h"

// FLAGz man!
unsigned int operatingMode_ = 0; // 0 = normal, 3 = return to prev sq, 4 = resQ
unsigned int rotating_ = 0;
unsigned int movingForward_ = 0;
// unsigned int movingToPrevSq_;
// unsigned int resQActive_;
 int LOOPer = 1;

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

// Check if the square is an unexplored road and sets the next direction to face it, send with 0 if exploring
int MAP_checkDirDown(int exploring)
{
	if (MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].description == 3 && (!MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].visited || exploring))
	{
		MAP_nextDir = 3;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it, send with 0 if exploring
int MAP_checkDirUp(int exploring)
{
	if (MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].description == 3 && (!MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].visited || exploring))
	{
		MAP_nextDir = 1;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it, send with 0 if exploring
int MAP_checkDirLeft(int exploring)
{
	if (MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].description == 3 && (!MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].visited || exploring))
	{
		MAP_nextDir = 2;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it, send with 0 if exploring
int MAP_checkDirRight(int exploring)
{
	if (MAP_array[MAP_currentPos[0]][MAP_currentPos[1] + 1].description == 3 && (!MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1] + 1].visited || exploring))
	{
		MAP_nextDir = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Chechks given direction
int MAP_checkDir(int dir)
{
	if (dir == 0)
	{
		return MAP_checkDirRight(1);
	}
	else if (dir == 1)
	{
		return MAP_checkDirUp(1);
	}
	else if (dir == 2)
	{
		return MAP_checkDirLeft(1);
	}
	else if (dir == 3)
	{
		return MAP_checkDirDown(1);
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
		if ( robotDir == 2 && (MAP_checkDirDown(0) || MAP_checkDirLeft(0) || MAP_checkDirUp(0) || MAP_checkDirRight(0)) )
		{} 
		else if ( robotDir == 1 && (MAP_checkDirLeft(0) || MAP_checkDirUp(0) || MAP_checkDirRight(0) || MAP_checkDirDown(0)) )
		{}
		else if (  robotDir == 0 && (MAP_checkDirUp(0) || MAP_checkDirRight(0) || MAP_checkDirDown(0) || MAP_checkDirLeft(0)) )
		{}
		else if (  robotDir == 3 && (MAP_checkDirRight(0) || MAP_checkDirDown(0) || MAP_checkDirLeft(0) || MAP_checkDirUp(0)) )
		{}
	}
	else if (mode_ == 'r')	
	{
		// Right-first
		if (  robotDir == 2 && (MAP_checkDirUp(0) || MAP_checkDirLeft(0) || MAP_checkDirDown(0) || MAP_checkDirRight(0)) )
		{}
		else if (  robotDir == 1 && (MAP_checkDirRight(0) ||  MAP_checkDirUp(0) || MAP_checkDirLeft(0) || MAP_checkDirDown(0)) )
		{}
		else if (  robotDir == 0 && (MAP_checkDirDown(0) || MAP_checkDirRight(0) ||  MAP_checkDirUp(0) || MAP_checkDirLeft(0)) )
		{}
		else if ( robotDir == 3 && (MAP_checkDirLeft(0) || MAP_checkDirDown(0) || MAP_checkDirRight(0) ||  MAP_checkDirUp(0)) )
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
	int dir_ = MAP_currentDir; //WHY?
	MAP_unexploredSquares = MAP_checkDirDown(0) + MAP_checkDirLeft(0) + MAP_checkDirRight(0) + MAP_checkDirUp(0);
	MAP_currentDir = dir_; // WHY?
}

// Uses Dijkstras algorithm to decide which location to set as a destination
void MAP_decideDestination()
{
	// Simpleton variant
	MAP_nextJunctionLong = MAP_lastUnexJunction();
	if (MAP_nextJunctionLong == 420) {}
	else if (MAP_nextJunctionLong < MAP_currentJunction)
	{
		MAP_nextJunctionShort = MAP_currentJunction - 1;
	}
	else if (MAP_nextJunctionLong > MAP_currentJunction)
	{
		MAP_nextJunctionShort = MAP_currentJunction + 1;
	}
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

// Adds the direction between two junctions, assuming j1 is the last visited and j2 the current one
void MAP_addJunctionDir(int j1, int j2, int direction)
{
	if (direction == 0)
	{
		MAP_junctionOrderArray[j1].right = j2;
	}
	else if (direction == 1)
	{
		MAP_junctionOrderArray[j1].up = j2;
	}
	else if (direction == 2)
	{
		MAP_junctionOrderArray[j1].left = j2;
	}
	else if (direction == 3)
	{
		MAP_junctionOrderArray[j1].down = j2;
	}
}

// Adds the current square as a junction
void MAP_addJunction()
{
	int posY_ = MAP_currentPos[0];
	int posX_ = MAP_currentPos[1];

	// Adds directions between last junction and this one
	MAP_addJunctionDir(MAP_currentJunction, MAP_junctionCount, MAP_lastJunctionDir);
	MAP_addJunctionDir(MAP_junctionCount, MAP_currentJunction, (MAP_currentDir - 2) % 4);
	
	// Increments counters
	MAP_currentJunction = MAP_junctionCount++;

	// Adds it in the map array
	MAP_setSquareDescription(5, posY_, posX_);
	MAP_array[posY_][posX_].junctionNumber = MAP_currentJunction;

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
		return 420;
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

void MAP_rotate()
{
	// Simulation code starts here
	MAP_currentDir = MAP_nextDir;
	int rotating_ = 0;
	// Simulation code ends here
}

void MAP_moveForward()
{
	// Simulation code starts here
	if (MAP_currentDir == 0)
	{
		MAP_currentPos[1]++;
	}
	else if (MAP_currentDir == 1)
	{
		MAP_currentPos[0]--;
	}
	else if (MAP_currentDir == 2)
	{
		MAP_currentPos[1]--;
	}
	else if (MAP_currentDir == 3)
	{
		MAP_currentPos[0]++;
	}
	movingForward_ = 0;
	// Simulation code ends here
}

int MAP_getDirection(int j1, int j2)
{
	if (MAP_junctionOrderArray[j1].right == j2)
	{
		return 0;
	}
	else if (MAP_junctionOrderArray[j1].up == j2)
	{
		return 1;
	}
	else if (MAP_junctionOrderArray[j1].left == j2)
	{
		return 2;
	}
	else if (MAP_junctionOrderArray[j1].down == j2)
	{
		return 3;
	}
}

void MAP_main()
{
	int posY_ = MAP_currentPos[0];
	int posX_ = MAP_currentPos[1];
	
	// Normal searching phase
	if (!operatingMode_ && !rotating_ && !movingForward_)
	{
		MAP_countUnexploredSquares();
		// If it's visited junction
		if (MAP_array[posY_][posX_].visited && MAP_array[posY_][posX_].description == 5)
		{
			// Add the distance between this and the last junction
			MAP_addJunctionDir(MAP_currentJunction, MAP_array[posY_][posX_].junctionNumber, MAP_lastJunctionDir);
			MAP_addJunctionDir(MAP_array[posY_][posX_].junctionNumber, MAP_currentJunction, (MAP_currentDir - 2) % 4);
			

			// Has unexplored roads
			if (MAP_unexploredSquares)
			{
				MAP_currentJunction = MAP_array[posY_][posX_].junctionNumber;
				MAP_junctionOrderArray[MAP_array[posY_][posX_].junctionNumber].hasUnex = 0;
			} 
			else
			{
				MAP_junctionOrderArray[MAP_array[posY_][posX_].junctionNumber].hasUnex = 0;
				MAP_nextDir = (MAP_currentDir - 2) % 4;
				rotating_ = 1;
				movingForward_ = 1;
				operatingMode_ = 3;
				goto afterNormal; // Go to previous junction
			}
		}
		// If it's a new square
		else
		{
			// Check if it's a junction
			if (MAP_unexploredSquares > 1)
			{
				MAP_addJunction();
			}
			// Or if it's a dead end
			else if (MAP_unexploredSquares == 0)
			{
				MAP_nextDir = (MAP_currentDir - 2) % 4;
				rotating_ = 1;
				movingForward_ = 1;
				operatingMode_ = 3;
				goto afterNormal; // Go to previous junction
			}
		}
		MAP_decideDirection('a'); // Uses random
		if (MAP_currentDir != MAP_nextDir)
		{
			rotating_ = 1;
		}
		movingForward_ = 1;
	}
	
	// Rotating phase
	afterNormal:
	if (rotating_)
	{
		/* Rotera lämpligt - Egen fas
				-ändra currentDir
				-hur veta vilken mode ska följa denna???*/

		// Simulation code starts here
		MAP_rotate();
		// Simulation code ends here
	}
	
	// Moving forward phase
	if (movingForward_ && !rotating_)
	{
		// åk framåt

		// Simulation code starts here
		MAP_moveForward();
		// Simulation code ends here
	}
	
	// Go until next junction phase
	if (operatingMode_ == 3 && !rotating_ && !movingForward_)
	{
		// åk tillbaka till föregående korsning
		// ändrar direction om det behövs
		if (MAP_checkDir((MAP_currentDir - 1) % 4))
		{
			MAP_nextDir = (MAP_currentDir - 1) % 4;
			rotating_ = 1;
		}
		else if (MAP_checkDir((MAP_currentDir + 1) % 4))
		{
			MAP_nextDir = (MAP_currentDir + 1) % 4;
			rotating_ = 1;
		}
		movingForward_ = 1;
		if (MAP_currentJunction == MAP_nextJunctionShort)
		{
			operatingMode_ = 2;
			movingForward_ = 0;
		}
	}

	// Go to a junction far, far away
	if (operatingMode_ == 2 && !rotating_ && !movingForward_)
	{
		MAP_decideDestination();
		if (MAP_currentJunction == MAP_nextJunctionLong)
		{
			operatingMode_ = 0;
		}
		else if (MAP_nextJunctionLong == 420)
		{
			LOOPer = 0;
		}
		else
		{
			MAP_nextDir = MAP_getDirection(MAP_currentJunction, MAP_nextJunctionShort);
			if (!(MAP_currentDir == MAP_nextDir))
			{
				rotating_ = 1;
			}
			movingForward_ = 1;
		}
	}
	
	// ResQ phase
	if (operatingMode_ == 4 && !rotating_ && movingForward_)
	{
		// Go ResQ.PL, go!
	}
}

// Kodskelett för kartläggning
/*
TODO
-fallet där korsningen man kommer till har besökts tidigare
*/
int main(void)
{
	/*do {
		
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
			int delta_dir = MAP_nextDir - MAP_currentDir;
			// Rotera lämpligt - Egen fas:
			if (abs(delta_dir) == 2)
			{
				// rotera 180 grader ?t vilket h?ll som helst
			}
			else if ( (delta_dir == 1) | (delta_dir == -3) )
			{
				// rotera 90 grader ?t v?nster
			}
			else 
			{
				// rotera 90 grader ?t h?ger
			}
			
			// ?ndra currentDir:
			MAP_currentDir = MAP_nextDir;
		}
		// TODO ?k fram en ruta - Egen fas
		MAP_travelledDist++;
		// ?ndra currentPos:
		if (!MAP_currentDir) {MAP_currentPos[1]++;} // drove right
		else if (MAP_currentDir == 1) {MAP_currentPos[0]--;} // drove up
		else if (MAP_currentDir == 2) {MAP_currentPos[1]--;} // drove left
		else {MAP_currentPos[0]++;} // drove down
		
		MAP_setVisited();
		
	} while (MAP_array[MAP_currentPos[0]][MAP_currentPos[1]].goal == 0)
	
	
	// Påbörja undsättning - Egen fas*/

	while (LOOPer)
	{
		MAP_main();
	}
	return 0;
}
