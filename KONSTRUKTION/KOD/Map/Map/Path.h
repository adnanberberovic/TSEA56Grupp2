/*
 * Path.h
 *
 * Created: 10/5/2015
 * Last modified: 14/5/2015
 * Author: nikag669
 *
 */

#ifndef PATH_H
#define PATH_H

struct tree_el {
   unsigned int cost, nr;
   struct tree_el * right, *middle, * left, * parent;
};
typedef struct tree_el node;


//--------------------------------------------------------

struct list_el {
   unsigned int cost, nr;
   struct list_el * next;
};
typedef struct list_el item;

//-------------------------------------------------------

unsigned int MAP_startJunction = 0; //startrutan
node * nodeArray[7]; // all junctions and also start/destination squares

// create two path-lists
item * head1 = NULL, * head2 = NULL;



#endif
