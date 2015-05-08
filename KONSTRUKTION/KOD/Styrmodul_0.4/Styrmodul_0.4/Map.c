/*
 * Map.c
 *
 * Created: 4/23/2015 1:13:46 PM
 *  Author: robop806
 */ 


//#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "Map.h"

// FLAGz man!
int operatingMode_ = 0; // 0 = normal, 3 = return to prev sq, 4 = resQ
int rotating_ = 0;
int movingForward_ = 0;
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
	if ((MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].description == 3) && (!MAP_array[MAP_currentPos[0] + 1][MAP_currentPos[1]].visited || exploring))
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
	if ((MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].description == 3) && (!MAP_array[MAP_currentPos[0] - 1][MAP_currentPos[1]].visited || exploring))
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
	if ((MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].description == 3) && (!MAP_array[MAP_currentPos[0]][MAP_currentPos[1] - 1].visited || exploring))
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
	if ((MAP_array[MAP_currentPos[0]][MAP_currentPos[1] + 1].description == 3) && (!MAP_array[MAP_currentPos[0]][MAP_currentPos[1] + 1].visited || exploring))
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
		fprintf(fp,"Right=%d\n",MAP_checkDirRight(1));
        return MAP_checkDirRight(1);
	}
	else if (dir == 1)
	{
		fprintf(fp,"Up=%d\n",MAP_checkDirUp(1));
        return MAP_checkDirUp(1);
	}
	else if (dir == 2)
	{
		fprintf(fp,"Left=%d\n",MAP_checkDirLeft(1));
        return MAP_checkDirLeft(1);
	}
	else if (dir == 3)
	{
		fprintf(fp,"Down=%d\n",MAP_checkDirDown(1));
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
	fprintf(fp,"Down=%d, Left=%d, Right=%d, Up=%d\n",MAP_checkDirDown(0),MAP_checkDirLeft(0),MAP_checkDirRight(0),MAP_checkDirUp(0));
	MAP_currentDir = dir_; // WHY?
}

// Uses Dijkstras algorithm to decide which location to set as a destination
void MAP_decideDestination()
{
	// Simpleton variant
	MAP_nextJunctionLong = MAP_lastUnexJunction(MAP_junctionCount);
	fprintf(fp,"nJl decided\n");
	fflush(fp);
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
	if (x <= 0)
	{
		/* Kartan �r helt utforskad!!! */
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
	rotating_ = 0;
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
	MAP_setVisited();
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
	
//	fprintf(fp,"Op mode: %d ",operatingMode_);
//	fprintf(fp,"Rot=%d ",rotating_);
//	fprintf(fp,"MovF=%d\n",movingForward_);
//	fflush(fp);
	
	// Normal searching phase
	if (!operatingMode_ && !rotating_ && !movingForward_)
	{
                        fprintf(fp,"Normal mode start\n");
                        fflush(fp);
                        
		MAP_countUnexploredSquares();
		// If it's a visited junction
		if (MAP_array[posY_][posX_].visited && (MAP_array[posY_][posX_].description == 5))
		{
                                            fprintf(fp,"Entered visited junction\n");
                                            fflush(fp);
                                            
			// Add the distance between this and the last junction
			MAP_addJunctionDir(MAP_currentJunction, MAP_array[posY_][posX_].junctionNumber, MAP_lastJunctionDir);
			MAP_addJunctionDir(MAP_array[posY_][posX_].junctionNumber, MAP_currentJunction, (MAP_currentDir - 2) % 4);
			

			// Has unexplored roads
			if (MAP_unexploredSquares)
			{
				fprintf(fp,"Has UnexSQ=%d\n",MAP_unexploredSquares);
				fflush(fp);
                MAP_currentJunction = MAP_array[posY_][posX_].junctionNumber;
				MAP_junctionOrderArray[MAP_array[posY_][posX_].junctionNumber].hasUnex = 0;
			} 
			else
			{
                
				MAP_junctionOrderArray[MAP_array[posY_][posX_].junctionNumber].hasUnex = 0;
				//MAP_nextDir = (MAP_currentDir - 2) % 4;
				//rotating_ = 1;
				//movingForward_ = 1;
				operatingMode_ = 3;
				goto afterNormal; // Go to previous junction
			}
		}
		// If it's a new square
		else
		{
            fprintf(fp,"Entered new square\n");
            fflush(fp);
            
			// Check if it's a junction
			if (MAP_unexploredSquares > 1)
			{
				fprintf(fp,"Has unex roads, UnexSQ=%d\n",MAP_unexploredSquares);
				fflush(fp);
                MAP_addJunction();
			}
			// Or if it's a dead end
			else if (MAP_unexploredSquares == 0)
			{
                 fprintf(fp,"Dead end, UnexSq=%d\n",MAP_unexploredSquares);
                 fflush(fp);
                 
				MAP_nextDir = (MAP_currentDir - 2) % 4;
				rotating_ = 1;
				movingForward_ = 1;
				operatingMode_ = 3;
				goto afterNormal; // Go to previous junction
			}
		}
		MAP_decideDirection('a'); // Uses a = random, r = right-first, l = left-first
		if (MAP_currentDir != MAP_nextDir)
		{
			rotating_ = 1;
		}
		movingForward_ = 1;
		fprintf(fp,"Normal mode end\n");
		fflush(fp);
	}
	
	
	fprintf(fp,"Op mode: %d ",operatingMode_);
	fprintf(fp,"Rot=%d ",rotating_);
	fprintf(fp,"MovF=%d\n",movingForward_);
	fflush(fp);
	
	// Rotating phase
	afterNormal:
	if (rotating_)
	{
                  fprintf(fp,"Rotating mode starts\n");
                  fflush(fp);
		/* Rotera l�mpligt - Egen fas
				-�ndra currentDir
				-hur veta vilken mode ska f�lja denna???*/

		// Simulation code starts here
		MAP_rotate();
		// Simulation code ends here
		fprintf(fp,"Rotating mode ends\n");
		fflush(fp);
	}
	
	fprintf(fp,"Op mode: %d ",operatingMode_);
	fprintf(fp,"Rot=%d ",rotating_);
	fprintf(fp,"MovF=%d\n",movingForward_);
	fflush(fp);
	
	// Moving forward phase
	if (movingForward_ && !rotating_)
	{
                       fprintf(fp,"Moving forward mode starts\n");
                       fflush(fp);
                       
		// �k fram�t

		// Simulation code starts here
		MAP_moveForward();
		// Simulation code ends here
		fprintf(fp,"Moving forward mode ends\n");
		fflush(fp);
	}
	
	fprintf(fp,"Op mode: %d ",operatingMode_);
	fprintf(fp,"Rot=%d ",rotating_);
	fprintf(fp,"MovF=%d\n",movingForward_);
	fflush(fp);
	
	// Go until next junction phase
	if ((operatingMode_ == 3) && !rotating_ && !movingForward_)
	{
                       fprintf(fp,"Go to junction mode starts\n");
                       fflush(fp);
		// �k tillbaka till f�reg�ende korsning
		// �ndrar direction om det beh�vs
		MAP_nextJunctionShort = MAP_currentJunction;
		
        if (MAP_checkDir((MAP_currentDir - 1) % 4))
		{
			fprintf(fp,"ROT=-1\n");
            MAP_nextDir = (MAP_currentDir - 1) % 4;
			rotating_ = 1;
		}
		else if (MAP_checkDir((MAP_currentDir + 1) % 4))
		{
			fprintf(fp,"ROT=1");
            MAP_nextDir = (MAP_currentDir + 1) % 4;
			rotating_ = 1;
		}
		fflush(fp);
		movingForward_ = 1;
		if ((MAP_currentPos[0] == MAP_junctionOrderArray[MAP_nextJunctionShort].posY) &&
		   (MAP_currentPos[1] == MAP_junctionOrderArray[MAP_nextJunctionShort].posX))
		{
			operatingMode_ = 2;
			movingForward_ = 0;
		}
		fprintf(fp,"Go to junction mode ends\n");
		fflush(fp);
	}

fprintf(fp,"Op mode: %d ",operatingMode_);
	fprintf(fp,"Rot=%d ",rotating_);
	fprintf(fp,"MovF=%d\n",movingForward_);
	fflush(fp);
	
	// Go to a junction far, far away
	if ((operatingMode_ == 2) && !rotating_ && !movingForward_)
	{
                       fprintf(fp,"Far junction mode starts\n");
                       fflush(fp);
		MAP_decideDestination();
		fprintf(fp,"Destination decided, nJl=%d\n",MAP_nextJunctionLong);
		fflush(fp);
		if ((MAP_currentPos[0] == MAP_junctionOrderArray[MAP_nextJunctionLong].posY) &&
		   (MAP_currentPos[1] == MAP_junctionOrderArray[MAP_nextJunctionLong].posX))
		{
                                fprintf(fp,"nJl == curPos!!\n");
                                fflush(fp);
			operatingMode_ = 0;
		}
		else if (MAP_nextJunctionLong == 420)
		{
             fprintf(fp,"420 weed master race resq.pl\n");
             fflush(fp);
			LOOPer = 0;
		}
		else
		{
            fprintf(fp,"�k mot nJs\n");
            fflush(fp);
			MAP_nextDir = MAP_getDirection(MAP_currentJunction, MAP_nextJunctionShort);
			if (!(MAP_currentDir == MAP_nextDir))
			{
				rotating_ = 1;
			}
			movingForward_ = 1;
		}
		fprintf(fp,"Far junction mode ends\n");
		fflush(fp);
	}
	
	fprintf(fp,"Op mode: %d ",operatingMode_);
	fprintf(fp,"Rot=%d ",rotating_);
	fprintf(fp,"MovF=%d\n",movingForward_);
	fflush(fp);
	
	// ResQ phase
	if (operatingMode_ == 4 && !rotating_ && movingForward_)
	{
		// Go ResQ.PL, go!
	}
}

// Kodskelett f�r kartl�ggning
/*
TODO
-fallet d�r korsningen man kommer till har bes�kts tidigare
*/

//------------------------------SIM--SIM--SIM-----------------------------------

int karta[15][15] = { {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                      {0,1,0,0,0,1,1,1,1,0,1,1,1,0,0},
                      {0,1,1,1,0,0,0,0,1,0,0,0,1,1,0},
                      {0,1,0,1,1,1,1,1,1,1,0,0,1,0,0},
                      {0,1,0,0,0,1,0,0,0,1,1,1,1,0,0},
                      {0,1,1,1,0,1,0,0,0,1,0,0,1,0,0},
                      {0,1,0,1,0,1,1,1,1,1,0,0,0,0,0},
                      {0,1,0,1,0,0,0,0,0,1,1,1,1,1,0},
                      {0,1,1,1,0,1,1,1,0,0,0,1,0,1,0},
                      {0,1,0,0,0,0,0,1,0,0,0,1,0,1,0},
                      {0,1,0,0,0,1,1,1,1,1,1,1,0,1,0},
                      {0,1,0,0,0,1,0,0,1,0,0,0,0,1,0},
                      {0,1,1,1,0,1,0,0,1,0,0,0,0,1,0},
                      {0,1,0,1,1,1,1,1,1,1,1,1,1,1,0},
                      {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0}
                      };
                      
void printMap()
{
     int i;
     for (i=0; i < 16; i++)
     {
         int j;
         for (j=0; j < 29; j++)
         {
             if (MAP_currentPos[0] == i && MAP_currentPos[1] == j)
             {
                fprintf(fp,"O");
                fflush(fp);
             }
             else if (MAP_array[i][j].description == 0)
             {
                fprintf(fp,"/");
                fflush(fp);
             }
             else if (MAP_array[i][j].description == 3)
             {
                  fprintf(fp,".");
                  fflush(fp);
             }
             else if (MAP_array[i][j].description == 4)
             {
                  fprintf(fp,"#");
                  fflush(fp);
             }
             else if (MAP_array[i][j].description == 5)
             {
                  int nombru = MAP_array[i][j].junctionNumber;
                  fprintf(fp,"X");
                  //fprintf(fp,"[%d]",nombru);
                  fflush(fp);
             }
         }
         
         fprintf(fp,"\n");
         fflush(fp);
         
     }
//     fprintf(fp,"\n");
//     fprintf(fp,"\n");
//     fprintf(fp,"\n");
//     fflush(fp);
}

void setDescr(int y, int x, int start)
{
     int y_ = y - 1;
     int x_ = x - start;
     
     if (karta[y_][x_] == 0)
     {
        MAP_array[y][x].description = 4;
        }
     else if (karta[y_][x_] == 1)
     {
          MAP_array[y][x].description = 3;
     }
}

void updateMap()
{
     unsigned int dirr_ = MAP_currentDir + 1;
     int y_ = MAP_currentPos[0];
     int x_ = MAP_currentPos[1];
     int i;
     for (i=0; i <= 2; i++)
     {
         if ((dirr_ - i) % 4 == 0)
         {
            setDescr(y_, x_ + 1, 12);
         }
         else if ((dirr_ - i) % 4 == 1)
         {
              setDescr(y_ - 1, x_, 12);
         }
         else if ((dirr_ - i) % 4 == 2)
         {
              setDescr(y_, x_ - 1, 12);
         }
         else if ((dirr_ - i) % 4 == 3)
         {
              setDescr(y_ + 1, x_, 12);
         }
     }
}

//------------------------------SIM--SIM--SIM-----------------------------------

int main(void)
{
    fp = fopen("debug.txt","w+");
    
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
			// �k till f�reg�ende korsning - Egen fas
		
	    }
		MAP_decideDirection('a'); // Uses random
		if (MAP_currentDir != MAP_nextDir)
		{
			int delta_dir = MAP_nextDir - MAP_currentDir;
			// Rotera l�mpligt - Egen fas:
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
	
	
	// P�b�rja unds�ttning - Egen fas*/
	
	unsigned int kkk = 0;
	--kkk;
    fprintf(fp,"START %u\n",kkk);
	fflush(fp);
       MAP_array[15][15].description = 3;
       MAP_setVisited();
       int i = 0;
	while (LOOPer)
	{
          fprintf(fp,"%d",i);
          i++;
          fprintf(fp,"\n");
          fprintf(fp,"CurPos=(%u,%u)\n",MAP_currentPos[0],MAP_currentPos[1]);
          fprintf(fp,"CurJ=%u, JCon=%u, lJDir=%u, nJs=%u, nJl=%u\n",MAP_currentJunction, MAP_junctionCount,
                               MAP_lastJunctionDir, MAP_nextJunctionShort, MAP_nextJunctionLong);
          //fprintf(fp,"(3,25) visited = %d\n",MAP_array[3][25].visited);
          //fprintf(fp,"(4,24) visited = %d\n",MAP_array[4][24].visited);
          fflush(fp);
          updateMap();
          printMap();
          fflush(fp);
        MAP_main();
        fprintf(fp,"CurPos=(%d,%d)\n",MAP_currentPos[0],MAP_currentPos[1]);
        fprintf(fp,"\n\n\n");
        fflush(fp);
	}
	fprintf(fp,"SLUT");
	fflush(fp);
	
	return 0;
}
