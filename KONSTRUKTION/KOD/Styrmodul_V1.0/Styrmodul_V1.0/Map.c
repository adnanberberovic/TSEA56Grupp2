/*
 * Map.c
 *
 * Created: 4/23/2015 1:13:46 PM
 *  Author: robop806
 */ 


#include <avr/io.h>
#include "Map.h"
#include "path.c"


// Sets the position of the goal in the map
void MAP_setGoal()
{
 	uint8_t posY = MAP_currentPos[0];
 	uint8_t posX = MAP_currentPos[1];
 	MAP_addJunction();
 	MAP_goalJunction = MAP_currentJunction;
	MAP_junctionOrderArray[MAP_goalJunction].hasUnex = 0;
 	MAP_junctionOrderArray[MAP_goalJunction].goal = 1;
 	MAP_goalPosition[0] = posY;
 	MAP_goalPosition[1] = posX;
    //MAP_operatingMode_ = 4;
 	//MAP_resQmode = 1;	
}

// Description:
// 0 = Start
// 1 = Unknown
// 2 = Uncertain
// 3 = Road
// 4 = Wall
// 5 = Junction
void MAP_setSquareDescription(uint8_t description, uint8_t posY, uint8_t posX)
{
	if(posY <= 16 && posX <= 29) // Is square within map?
	{
		MAP_array[posY][posX].description = description;
	}
}

// Check if the square is an unexplored road and sets the next direction to face it, send with 0 if exploring
uint8_t MAP_checkDirDown(uint8_t exploring)
{
	if (((MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].description == 3) &&
		(!MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].visited || exploring)) ||
		(MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].description == 5))
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
uint8_t MAP_checkDirUp(uint8_t exploring)
{
	if (((MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].description == 3) &&
		(!MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].visited || exploring)) ||
		(MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].description == 5))
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
uint8_t MAP_checkDirLeft(uint8_t exploring)
{
	if (((MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].description == 3) &&
		(!MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].visited || exploring)) ||
		(MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].description == 5))
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
uint8_t MAP_checkDirRight(uint8_t exploring)
{
	if (((MAP_array[MAP_currentPos[0]][MAP_currentPos[1] + 1].description == 3) &&
		(!MAP_array[MAP_currentPos[0]][MAP_currentPos[1] + 1].visited || exploring)) ||
		(MAP_array[MAP_currentPos[0]][MAP_currentPos[1] + 1].description == 5))
	{
		MAP_nextDir = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

// Checks given direction
uint8_t MAP_checkDir(uint8_t dir)
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
	else
	{
		return 255;
	}
}

// Decides which will be the next direction for the robot
// The different modes for choosing direction are: 'r' for right-first, 'l' for left-first or 'a' for random 
void MAP_decideDirection(char mode)
{

	char mode_ = mode;
	uint8_t robotDir = MAP_currentDir;
	
	// Randomizes the direction and mode, thus making the decisions (pseudo) random
	if (mode == 'a')
	{
		
	}
	else if (mode_ == 'l')
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

// Counts the number of unexplored squares in a 1 square radius (up, down, left, right)
void MAP_countSquares()
{
	uint8_t dir_ = MAP_currentDir; // The following lines change currentDir which we don't want
	MAP_unexploredSquares = MAP_checkDirDown(0) + MAP_checkDirLeft(0) + MAP_checkDirRight(0) + MAP_checkDirUp(0);
	MAP_exploredSquares = MAP_checkDirDown(1) + MAP_checkDirLeft(1) + MAP_checkDirRight(1) + MAP_checkDirUp(1);
	MAP_currentDir = dir_;
}

// Uses Dijkstras algorithm to decide which location to set as a destination
// This function sets the nextJunctionShort
// SIM
void MAP_decideDestination()
{
	MAP_lastUnexJunction(MAP_junctionCount);
	MAP_findPath(MAP_currentJunction, MAP_nextJunctionLong);
	MAP_nextJunctionShort = getNextJunction();
	
	
	// Simpleton variant
	//MAP_nextJunctionLong = MAP_lastUnexJunction(MAP_junctionCount);
// 	MAP_lastUnexJunction(MAP_junctionCount);
// 	if (MAP_nextJunctionLong == 255) {MAP_LOOPer = 0;}
// 	else if (MAP_nextJunctionLong < MAP_currentJunction)
// 	{
// 		MAP_nextJunctionShort = MAP_currentJunction - 1;
// 	}
// 	else if (MAP_nextJunctionLong > MAP_currentJunction)
// 	{
// 		MAP_nextJunctionShort = MAP_currentJunction + 1;
// 	}
}

// Adds the travelled distance from the last junction to specified junction in the junctionDistArray
uint8_t MAP_addJunctionDist(uint8_t j1, uint8_t j2)
{
	uint8_t returner_ = 0;

	if (((MAP_junctionDistArray[j1][j2] == 0) || (MAP_junctionDistArray[j1][j2] > MAP_travelledDist)) && (j1 != j2) && (MAP_travelledDist != 0))
	{
		MAP_junctionDistArray[j1][j2] = MAP_travelledDist;
		MAP_junctionDistArray[j2][j1] = MAP_travelledDist;
		returner_ = 1;
	}

	MAP_travelledDist = 0;
	return returner_;
}

// Adds the direction between two junctions, adds it in j1
void MAP_addJunctionDir(uint8_t j1, uint8_t j2, uint8_t direction)
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
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t posX_ = MAP_currentPos[1];
	
	// Initiate direction to 63 so direction to junction 0 works
	MAP_junctionOrderArray[MAP_junctionCount].right = 63;
	MAP_junctionOrderArray[MAP_junctionCount].up = 63;
	MAP_junctionOrderArray[MAP_junctionCount].left = 63;
	MAP_junctionOrderArray[MAP_junctionCount].down = 63;

	// Adds directions between last junction and this one
	if (MAP_addJunctionDist(MAP_currentJunction, MAP_junctionCount) || MAP_addJunctionDist(MAP_junctionCount, MAP_currentJunction))
	{
		// Only adds the direction if the distance is shorter than before, thus only the shortest direction
		// between two junctions is marked
		MAP_addJunctionDir(MAP_currentJunction, MAP_junctionCount, MAP_lastJunctionDir);
		MAP_addJunctionDir(MAP_junctionCount, MAP_currentJunction, (MAP_currentDir + 2) % 4);
	}

	// Increments counters
	MAP_currentJunction = MAP_junctionCount++;

	// Adds it in the map array
	MAP_setSquareDescription(5, posY_, posX_);
	MAP_array[posY_][posX_].junctionNumber = MAP_currentJunction;

	// Adds it in the junction array
	MAP_junctionOrderArray[MAP_currentJunction].hasUnex = 1;
	MAP_junctionOrderArray[MAP_currentJunction].posY = posY_;
	MAP_junctionOrderArray[MAP_currentJunction].posX = posX_;
}

// Returns the last junction with unexplored roads
// Call with junctionCount
// SIM
void MAP_lastUnexJunction(uint8_t x)
{
	if (x <= 0)
	{
		MAP_mapped = 1;
		MAP_resQmode = 1;	
	}
	else if (MAP_junctionOrderArray[x - 1].hasUnex == 1)
	{
		MAP_nextJunctionLong = x - 1;
	}
	else
	{
		MAP_lastUnexJunction(x - 1);
	}
}

// Rotates the robot to face nextDir
// SIM
void MAP_rotate()
{
	MAP_currentDir = MAP_nextDir;
	MAP_rotating_ = 0;
	
	if (MAP_array[MAP_currentPos[0]][MAP_currentPos[1]].description == 5)
	{
		MAP_lastJunctionDir = MAP_currentDir;
	}	
}

// Moves the robot one square forward
// SIM
void MAP_moveForward()
{
	// Simulation code starts here
	// Simulation code ends here
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
		
	MAP_movingForward_ = 0;
	MAP_travelledDist++;
	MAP_setVisited();
}

// Returns the direction from j1 to j2
uint8_t MAP_getDirection(uint8_t j1, uint8_t j2)
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
	else 
	{
		return 0;
	}
}

// Checks if all squares have been explored, and if so, quits the main loop
uint16_t MAP_checkIfDone()
{
	//uint8_t iterator = 0;
	uint16_t done_ = 0;

  	for (int i = 0; i < 16; i++)
  	{
  		for (int j = 0; j < 29; j++)
  		{
  			done_ += (MAP_array[i][j].visited == 0) && (MAP_array[i][j].description == 3);
  		}
   }
   
   return done_;
	
//  	while (iterator < MAP_junctionCount)
//  	{
//  		done_ += MAP_junctionOrderArray[iterator].hasUnex;
//  		iterator++;
//  	}
//  
//  	if (!done_ && MAP_junctionCount)
//  	{
//  		MAP_LOOPer = 0;
//  	}
}

void MAP_findPath(unsigned int j1, unsigned int j2) {
	initNodeArray();
	initPaths();

	node * root;
	root = nodeArray[j1];
	root->nr = j1;
	insert(&root, j2); // eventually the path is saved in a list

	// the tree nodes point to nodeArray (left, middle, right, parent)
	// should be enough to flush this array to free memory
	flushNodeArray();
	root->left = root->middle = root->right = root->parent = NULL;
	root = NULL;
	free(root);

}