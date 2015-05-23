#ifndef MAP_H
#define MAP_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

// Variables
uint8_t MAP_unexploredSquares = 0;
uint8_t MAP_exploredSquares = 0;
uint8_t MAP_currentPos[2] = {16, 15}; // NOTE: (y,x) base!!!!!! NOT (x,y)!
uint8_t MAP_currentDir = 1; // 0 = Right, 1 = Up, 2 = Left, 3 = Down 
uint8_t MAP_goalPosition[2]; // The (y,x) pos of the goal
uint8_t MAP_nextDir = 1; // Tells which direction we should face before moving forward again
uint8_t MAP_currentJunction = 0; // Tells us which is the current junction (or last visited), 0-63
uint8_t MAP_junctionCount = 0; // The number of junctions discovered so far
uint8_t MAP_travelledDist = 0; // Number of squares travelled from the previous junction
uint8_t MAP_lastJunctionDir = 0; // Direction out of the last junction
uint8_t MAP_nextJunctionShort = 0; // The immediate next desired junction
uint8_t MAP_nextJunctionLong = 0; // The long-term desired junction

// Flags used to navigate between the different phases in the main function
uint8_t MAP_operatingMode_ = 0; // 0 = normal, 2 = go to next junction long, 3 = go to next junction short, 4 = goal discovered
uint8_t MAP_rotating_ = 0;
uint8_t MAP_movingForward_ = 0;
uint8_t MAP_LOOPer = 1;

// Structs
struct MAP_square
{
	uint8_t visited :1;
	uint8_t description :3;
	uint8_t junctionNumber :6;
};
struct MAP_junction
{
	// The junction number in the given direction
	uint8_t right :6;
	uint8_t up :6;
	uint8_t left :6;
	uint8_t down :6;
	uint8_t hasUnex :1; // Flag if unexplored roads are present
	uint8_t posY :4;
	uint8_t posX :5;
	uint8_t goal :1; // Flag if this junction is the goal
};

// Arrays
struct MAP_square MAP_array[16][29]; // The map square structs
uint8_t MAP_junctionDistArray[64][64]; // Distance between junctions
struct MAP_junction MAP_junctionOrderArray[64]; // The junction square structs. Junctions are numbered 0-63, accessed
												// in this array by their number

// Methods
void MAP_decideDirection(char);
uint8_t MAP_checkDirUp(uint8_t);
uint8_t MAP_checkDirDown(uint8_t);
uint8_t MAP_checkDirLeft(uint8_t);
uint8_t MAP_checkDirRight(uint8_t);
uint8_t MAP_checkDir(uint8_t);
void MAP_setGoal();
void MAP_setSquareDescription(uint8_t, uint8_t, uint8_t);
void MAP_setVisited();
void MAP_countSquares();
void MAP_decideDestination();
void MAP_addJunction();
uint8_t MAP_addJunctionDist(uint8_t, uint8_t);
void MAP_addJunctionDir(uint8_t, uint8_t, uint8_t);
void MAP_lastUnexJunction(uint8_t);
void MAP_rotate();
void MAP_moveForward();
uint8_t MAP_getDirection(uint8_t, uint8_t);
void MAP_main();
void MAP_checkIfDone();

#endif
