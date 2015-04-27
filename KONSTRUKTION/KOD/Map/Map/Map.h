#ifndef MAP_H
#define MAP_H

// Variables
int MAP_unexploredSquares = 0;
int MAP_currentPos[2] = {16, 15}; // NOTE: (y,x) base!!!!!! NOT (x,y)!
int MAP_currentDir = 1; // 0 = Right, 1 = Up, 2 = Left, 3 = Down 
int MAP_goalPosition[2]; // The (y,x) pos of the goal
int MAP_nextDir = 1; // Tells which direction we should face before moving forward again
int MAP_currentJunction = -1; // Tells us which is the current junction (or last visited), 0-63
int MAP_junctionCount = 0; // The number of junctions discovered so far
int MAP_travelledDist = 0; // Number of squares travelled from the previous junction

// Structs
struct MAP_square
{
	unsigned int goal :1;
	unsigned int visited :1;
	unsigned int description :3;
};
struct MAP_junction // Contains the distance to every connected junction
{
	unsigned int distRight :5;
	unsigned int distUp :5;
	unsigned int distLeft :5;
	unsigned int distDown :5;
	
	// eller kanske detta
	unsigned int hasUnex :1; // Flag if unexplored roads are present
	unsigned int poxY :4;
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
int MAP_checkDirUp();
int MAP_checkDirDown();
int MAP_checkDirLeft();
int MAP_checkDirRight();
void MAP_setGoal();
void MAP_setSquareDescription(int, int, int);
void MAP_setVisited();
void MAP_countUnexploredSquares();
void MAP_decideDestination();
void MAP_addJunction();
void MAP_addJunctionDist(int);
int MAP_lastUnexJunction();

#endif
