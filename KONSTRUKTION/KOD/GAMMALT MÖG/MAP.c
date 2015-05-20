/*
 *	TSEA56 Grupp 2
 *	Mapping algorithm
 *	Author:
 *	Adnan Berberovic
 *	Robert Oprea
 */

class MAP
{
public:

	// Variables
	int MAP_unexploredSquares = ;
	void MAP_markSquare(char, int, int);
	char MAP_decideDirection(char);
	int MAP_goalPosition[2];
	struct MAP_square
	{
		unsigned int goal :1;
		unsigned int robot :1;
		unsigned int description :3;
	};
	MAP_square MAP_array[16][29];

	// Methods
	void MAP_setGoal(int, int);
	void MAP_setSquareDescription(int, int, int);
	MAP_square MAP_getSquareFromPos(int, int);

};


void MAP::MAP_setGoal(int posX, int posY)
{
	MAP_array[posX][posY].goal = 1;
	MAP_goalPosition[0] = posX;
	MAP_goalPosition[1] = posY;
}

void MAP::MAP_setSquareDescription(int description, int posX, int posY)
{
	MAP_array[posX][posY].description = description;
}

MAP_square MAP::MAP_getSquareFromPos(int posX, int posY)
{
	return MAP_array[posX][posY];
}



int main(void)
{


	return 0;
}