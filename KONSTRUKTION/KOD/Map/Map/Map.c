/*
 * Map.c
 *
 * Created: 4/23/2015 1:13:46 PM
 *  Author: robop806
 */ 


#include <avr/io.h>
#include "Map.h"

// Flags used to navigate between the different phases in the main function
uint8_t operatingMode_ = 0; // 0 = normal, 2 = go to next junction long, 3 = go to next junction short, 4 = goal discovered
uint8_t rotating_ = 0;
uint8_t movingForward_ = 0;
uint8_t LOOPer = 1;

// Sets the position of the goal in the map
void MAP_setGoal()
{
	uint8_t posY = MAP_currentPos[0];
	uint8_t posX = MAP_currentPos[1];
	MAP_junctionOrderArray[MAP_currentJunction].goal = 1;
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
	// Simpleton variant
	//MAP_nextJunctionLong = MAP_lastUnexJunction(MAP_junctionCount);
	MAP_lastUnexJunction(MAP_junctionCount);
	if (MAP_nextJunctionLong == 255) {LOOPer = 0;}
	else if (MAP_nextJunctionLong < MAP_currentJunction)
	{
		MAP_nextJunctionShort = MAP_currentJunction - 1;
	}
	else if (MAP_nextJunctionLong > MAP_currentJunction)
	{
		MAP_nextJunctionShort = MAP_currentJunction + 1;
	}
}

// Adds the travelled distance from the last junction to specified junction in the junctionDistArray
uint8_t MAP_addJunctionDist(uint8_t j1, uint8_t j2)
{
	uint8_t returner_ = 0;

	if ((MAP_junctionDistArray[j1][j2] == 0 || MAP_junctionDistArray[j1][j2] > MAP_travelledDist) && j1 != j2 && MAP_travelledDist != 0)
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

	// Adds directions between last junction and this one
	if (MAP_addJunctionDist(MAP_currentJunction, MAP_junctionCount) || MAP_addJunctionDist(MAP_junctionCount, MAP_currentJunction))
	{
		// Only adds the direction if the distance is shorter than before, thus onnly the shortest direction
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
uint8_t MAP_lastUnexJunction(uint8_t x)
{
	if (x <= 0)
	{
		// The map is fully explored
		return 255;
	}
	else if (MAP_junctionOrderArray[x - 1].hasUnex == 1)
	{
		MAP_nextJunctionLong = x - 1;
		return x - 1;
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
	// Simulation code starts here
	MAP_currentDir = MAP_nextDir;
	rotating_ = 0;
	// Simulation code ends here
}

// Moves the robot one square forward
// SIM
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
	MAP_travelledDist++;
	MAP_setVisited();
	// Simulation code ends here
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
}

// Checks if all squares have been explored, and if so, quits the main loop
void MAP_checkIfDone()
{
	uint8_t i = 0;
	uint8_t done_ = 0;

	while (i < MAP_junctionCount)
	{
		done_ += MAP_junctionOrderArray[i].hasUnex;
		i++;
	}

	if (!done_ && MAP_junctionCount)
	{
		LOOPer = 0;
	}
}

// The main searching function. Makes all the decisions
void MAP_main()
{
	// Save these under more convenient names
	uint8_t posY_ = MAP_currentPos[0];
	uint8_t posX_ = MAP_currentPos[1];
	
	// Normal searching phase
	if (!operatingMode_ && !rotating_ && !movingForward_)
	{
		MAP_countSquares();

		// If it's a visited junction
		if (MAP_array[posY_][posX_].visited && (MAP_array[posY_][posX_].description == 5))
		{
			// Add the directions between this and the last junction
			if (MAP_addJunctionDist(MAP_currentJunction, MAP_array[posY_][posX_].junctionNumber) ||
				MAP_addJunctionDist(MAP_array[posY_][posX_].junctionNumber, MAP_currentJunction))
			{
				MAP_addJunctionDir(MAP_array[posY_][posX_].junctionNumber, MAP_currentJunction, (MAP_currentDir + 2) % 4);
				if (MAP_array[posY_][posX_].junctionNumber != MAP_currentJunction)
				{
					MAP_addJunctionDir(MAP_currentJunction, MAP_array[posY_][posX_].junctionNumber, MAP_lastJunctionDir);
				}
			}
			
			// And if it has unexplored roads
			// Else go to far junction phase
			int fatIf_;
			fatIf_ = MAP_unexploredSquares -
					(MAP_array[posY_ - 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ - 1][posX_].junctionNumber].hasUnex) -
					(MAP_array[posY_ + 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ + 1][posX_].junctionNumber].hasUnex) -
					(MAP_array[posY_][posX_ - 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ - 1].junctionNumber].hasUnex) -
					(MAP_array[posY_][posX_ + 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ + 1].junctionNumber].hasUnex);
			if (fatIf_ >= 1)
			{
				MAP_currentJunction = MAP_array[posY_][posX_].junctionNumber;
				MAP_junctionOrderArray[MAP_array[posY_][posX_].junctionNumber].hasUnex = 0;
			} 
			else
			{
				MAP_currentJunction = MAP_array[posY_][posX_].junctionNumber;
				MAP_junctionOrderArray[MAP_currentJunction].hasUnex = 0;
				MAP_decideDestination();
				operatingMode_ = 2;
				goto afterNormal; // Exit this phase
			}
		}
		// Or if it's a new square
		else
		{
			// If it's a junction, add it
			// Else if it's a dead end, trace the way back to the previous junction
			if (MAP_exploredSquares > 2)
			{
				MAP_addJunction();
			}
			else if (MAP_unexploredSquares == 0)
			{
				MAP_nextDir = (MAP_currentDir + 2) % 4;
				rotating_ = 1;
				movingForward_ = 1;
				operatingMode_ = 3;
				MAP_nextJunctionShort = MAP_currentJunction;
				goto afterNormal; // Exit this phase
			}
		}

		// Decide the next direction
		MAP_decideDirection('l'); // Uses a = random, r = right-first, l = left-first
		if (MAP_currentDir != MAP_nextDir)
		{
			rotating_ = 1;
		}
		movingForward_ = 1;
	}
	
	// Rotating phase
	// SIM
	afterNormal:
	if (rotating_)
	{
		// Simulation code starts here
				MAP_rotate();
		// Simulation code ends here
	}

	// Moving forward phase
	// SIM
	if (movingForward_ && !rotating_)
	{
		// Simulation code starts here
		if (MAP_array[posY_][posX_].description == 5)
		{
			MAP_lastJunctionDir = MAP_currentDir;
		}

		MAP_moveForward();

		// Update the position variables
		posY_ = MAP_currentPos[0];
		posX_ = MAP_currentPos[1];
		// Simulation code ends here
	}

	// Go until nextJunctionShort phase
	// This phase is meant to navigate through an explored corridor between two junctions
	if ((operatingMode_ == 3) && !rotating_ && !movingForward_)
	{
		// Checks if we can move right or left, then rotate accordingly, otherwise move forward
		if (MAP_checkDir((MAP_currentDir + 3) % 4))
		{
			MAP_nextDir = (MAP_currentDir + 3) % 4;
			rotating_ = 1;
		}
		else if (MAP_checkDir((MAP_currentDir + 5) % 4))
		{
			MAP_nextDir = (MAP_currentDir + 5) % 4;
			rotating_ = 1;
		}
		movingForward_ = 1;

		// If we have arriwed to the desired junction
		if ((MAP_currentPos[0] == MAP_junctionOrderArray[MAP_nextJunctionShort].posY) &&
			(MAP_currentPos[1] == MAP_junctionOrderArray[MAP_nextJunctionShort].posX))
		{
			MAP_travelledDist = 0;
			MAP_countSquares();
			MAP_currentJunction = MAP_nextJunctionShort;

			// If the current junction has unexplored squares we go to normal phase
			// Else we go to far junction phase
			int fatIf_;
			fatIf_ = MAP_unexploredSquares -
					(MAP_array[posY_ - 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ - 1][posX_].junctionNumber].hasUnex) -
					(MAP_array[posY_ + 1][posX_].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_ + 1][posX_].junctionNumber].hasUnex) -
					(MAP_array[posY_][posX_ - 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ - 1].junctionNumber].hasUnex) -
					(MAP_array[posY_][posX_ + 1].description == 5 && !MAP_junctionOrderArray[MAP_array[posY_][posX_ + 1].junctionNumber].hasUnex);
			if (fatIf_ >= 1)
			{
				rotating_ = 0;
				movingForward_ = 0;
				operatingMode_ = 0;
			}
			else
			{
				MAP_decideDestination();
				operatingMode_ = 2;
				movingForward_ = 0;
			}
		}
	}

	// Go to a junction farther away phase
	// Decides which junction we should be heading to
	if ((operatingMode_ == 2) && !rotating_ && !movingForward_)
	{
		// If we have arrived at the desired junction then we start the normal phase
		// Else if all the map has been explored we quit the main loop
		// Else we start moving towards the nextJunctionShort, going to that phase
		if ((MAP_currentPos[0] == MAP_junctionOrderArray[MAP_nextJunctionLong].posY) &&
			(MAP_currentPos[1] == MAP_junctionOrderArray[MAP_nextJunctionLong].posX))
		{
			MAP_travelledDist = 0;
			operatingMode_ = 0;
		}
		else if (MAP_nextJunctionLong == 255)
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
			operatingMode_ = 3;
		}
	}

	// This phase is activated when we have discovered the goal
	if (operatingMode_ == 4 && !rotating_ && movingForward_)
	{
		// Go ResQ.PL, go!
	}

	// Checks if all the map has been explored
	MAP_checkIfDone();
}


// detta beh�vs i en main f�r kartans del
int main(void)
{
	// initiera robotens position
	MAP_array[15][15].description = 3;
	MAP_setVisited();


	while (LOOPer)
	{
		//updateMap(); en stycke som uppdaterad kartan med nyf�rv�rvad information
		MAP_main();
	}

	return 0;
}
