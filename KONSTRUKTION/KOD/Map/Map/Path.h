#ifndef PATH_H
#define PATH_H


struct tree_el {
   unsigned int cost, nr;
   struct tree_el * right, *middle, * left, * parent;
};
typedef struct tree_el node;


//--------------------------------------------------------

struct list_el {
   unsigned int cost;
   struct list_el * next;
};
typedef struct list_el item;

//-------------------------------------------------------

unsigned int MAP_startJunction = 0; //startrutan
node * nodeArray[7];

// create two path-lists
item * head1 = NULL, * head2 = NULL;



#endif
