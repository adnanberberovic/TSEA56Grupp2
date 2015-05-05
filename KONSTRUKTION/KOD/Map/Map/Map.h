#ifndef MAP_H
#define MAP_H

// Variables
unsigned int MAP_unexploredSquares = 0;
unsigned int MAP_currentPos[2] = {15, 15}; // NOTE: (y,x) base!!!!!! NOT (x,y)!
unsigned int MAP_currentDir = 1; // 0 = Right, 1 = Up, 2 = Left, 3 = Down 
unsigned int MAP_goalPosition[2]; // The (y,x) pos of the goal
unsigned int MAP_nextDir = 1; // Tells which direction we should face before moving forward again
unsigned int MAP_currentJunction = 0; // Tells us which is the current junction (or last visited), 0-63
unsigned int MAP_junctionCount = 0; // The number of junctions discovered so far
unsigned int MAP_travelledDist = 0; // Number of squares travelled from the previous junction
unsigned int MAP_lastJunctionDir = 0; // Direction out of the last junction
unsigned int MAP_nextJunctionShort = 0; // The immediate next desired junction
unsigned int MAP_nextJunctionLong = 0; // The long-term desired junction

FILE *fp;

// Structs
struct MAP_square
{
	unsigned int goal :1;
	unsigned int visited :1;
	unsigned int description :3;
	unsigned int junctionNumber :6;
};
struct MAP_junction // Contains the distance to every connected junction
{
	// The junction number in the given direction
	unsigned int right :6;
	unsigned int up :6;
	unsigned int left :6;
	unsigned int down :6;
	
	// eller kanske detta
	unsigned int hasUnex :1; // Flag if unexplored roads are present
	unsigned int posY :4;
	unsigned int posX :5;
	
	// när man kommit till korsning med bara utforskade vägar, kolla rekursivt om korsningen innan har 
	// outforskade vägar. När numret på den korsning man ska åka till har hittats - Dijkstra!
	
};

// Arrays
struct MAP_square MAP_array[16][29]; // The map square structs
int MAP_junctionDistArray[64][64]; // Distance between junctions
struct MAP_junction MAP_junctionOrderArray[64]; // The junction square structs. Junctions are numbered 0-63, accessed
	// in this array by their number

// Methods
void MAP_decideDirection(char);
int MAP_checkDirUp(int);
int MAP_checkDirDown(int);
int MAP_checkDirLeft(int);
int MAP_checkDirRight(int);
int MAP_checkDir(int);
void MAP_setGoal();
void MAP_setSquareDescription(int, int, int);
void MAP_setVisited();
void MAP_countUnexploredSquares();
void MAP_decideDestination();
void MAP_addJunction();
void MAP_addJunctionDist(int);
void MAP_addJunctionDir(int, int, int);
int MAP_lastUnexJunction(int);
void MAP_rotate();
void MAP_moveForward();
int MAP_getDirection(int, int);
void MAP_main();

#endif
