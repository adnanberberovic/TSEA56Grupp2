#ifndef MAP_H
#define MAP_H

// Variables
int MAP_unexploredSquares = 0;
int MAP_currentPos[2] = {16, 15}; // NOTE: (y,x) base!!!!!! NOT (x,y)!
int MAP_currentDir = 1; // 0 = Right, 1 = Up, 2 = Left, 3 = Down 
int MAP_goalPosition[2];
int MAP_nextDir = 1;
struct MAP_square
{
	unsigned int goal :1;
	unsigned int visited :1;
	unsigned int description :3;
};
struct MAP_junction // Contains the distance to every connected junction
{
	unsigned int right :5;
	unsigned int up :5;
	unsigned int left :5;
	unsigned int down :5;
};
struct MAP_square MAP_array[16][29];

// For optimization: path cost to other junctions.
struct MAP_junction MAP_junctionArray[32][32]; // Mother fucker too big	

// Methods
void MAP_decideDirection(char);
int MAP_checkDirUp();
int MAP_checkDirDown();
int MAP_checkDirLeft();
int MAP_checkDirRight();
void MAP_setGoal();
void MAP_setSquareDescription(int, int, int);
void MAP_setVisited();
void MAP_countUnexplored();
void MAP_decideDestination();

#endif