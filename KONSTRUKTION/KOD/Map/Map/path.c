/*
 * path.c
 *
 * Created: 5/5/2015
 * Last modified: 14/5/2015
 * Author: nikag669
 *
 */

#include<stdlib.h>
#include<stdio.h>
#include "Path.h"
#include "Map.h"

/* the shortest path will be saved
either in head1 or head2 */


void printPath() {
	item * curr;
	if (head1) {
		curr = head1;
	}
	else {
		curr = head2;
	}
	while(curr) {
      printf("%d\n", curr->nr);
      curr = curr->next;
	}
}

// allocate memory to every node in nodeArray[i]
void initNodeArray() {
	unsigned int i;
	for (i=0; i<6; i++) {
		nodeArray[i] = (node *)malloc(sizeof(node));
		nodeArray[i]->left = nodeArray[i]->middle = nodeArray[i]->right = nodeArray[i]->parent = NULL;
		nodeArray[i]->nr = i;
		nodeArray[i]->cost = 0;
	}
}

// frees memory the nodes in nodeArray occupies
void flushNodeArray() {
	unsigned int i;
	for (i=0; i<6; i++) {
		nodeArray[i]->left = nodeArray[i]->middle = nodeArray[i]->right = nodeArray[i]->parent = NULL;
		nodeArray[i] = NULL;
		free(nodeArray[i]);
	}
}

// initiate two path-lists
void initPaths() {
	head1 = (item *)malloc(sizeof(item));
	head2 = (item *)malloc(sizeof(item));
	head1 = head2 = NULL;
}

// inserts an item in front of a list
void buildList(node ** tree, int isFirst) {
	node * curr_node = *tree;
	if (isFirst) {
		while (curr_node) {
			item * curr_item;
			curr_item = (item *)malloc(sizeof(item));
			curr_item->next = head1;
			curr_item->nr = curr_node->nr;
			head1 = curr_item;
			curr_node = curr_node->parent;
		}
		printPath();
	}
	else {
		while (curr_node) {
			item * curr_item;
			curr_item = (item *)malloc(sizeof(item));
			curr_item->next = head2;
			curr_item->nr = curr_node->nr;
			head2 = curr_item;
			curr_node = curr_node->parent;
		}
		printPath();
	}
}

// deletes the list (except: doesn't free the head node)
void flushList(item ** head) {
	if (!(*head)) { //empty
		return;
	}

	item * curr = *head;
	*head = NULL;

	if (curr->next != NULL)
	{
		flushList(&(curr->next));
		free(curr); //move outside of if{} to free the first node also
	}
}

// compares the two path-lists and flushes the more expensive one
void choosePath() {
	if (&(head1->cost) >= &(head2->cost)) {
		flushList(&head2);
	}
	else {
		flushList(&head1);
	}
}

// checks if a node is already in a list (or in a tree branch)
int node_exists(node ** tree, unsigned int nodeNr) {
	node * curr = *tree;
	do {
		if (curr->nr == nodeNr) {
			return 1;
		}
		curr = curr->parent;
	} while (curr != 0); //segmentation fault (curr->parent != 0)

	return 0;
}

void insert(node ** tree) {
	//stop growing branch if reached desired leaf
	if ((*tree)->nr == MAP_startJunction) { //0
		// TO DO: save this path to a list(?) with cost /done
		if (!head1) { // if list head1 is empty
			// save path to head1
			buildList(tree, 1);
		}
		else {
			// save path to head2
			buildList(tree, 0);
		}
		// flushes expensive path-list
		choosePath();
		return;
	}
	unsigned int i, dist, child=1; // osäker om child kan överskrivas eller ej, i olika nivåer
	for (i=0; i<6; i++) {
		if (!node_exists(tree, i)) { // parent control
			dist = MAP_junctionDistArray[(*tree)->nr][i];
			if (dist > 0) {
				if (child == 1) {
					nodeArray[i]->cost = (*tree)->cost + dist;
					nodeArray[i]->parent = *tree;
					(*tree)->left = nodeArray[i]; //connect nodes
					insert(&(*tree)->left);
					child = 2; //går inte med child++ !
				}
				else if (child == 2) {
					nodeArray[i]->cost = (*tree)->cost + dist;
					nodeArray[i]->parent = *tree;
					(*tree)->middle = nodeArray[i]; //connect nodes
					insert(&(*tree)->middle);
					child = 3; //går inte med child++ !
				}
				else if (child == 3) {
					nodeArray[i]->cost = (*tree)->cost + dist;
					nodeArray[i]->parent = *tree;
					(*tree)->right = nodeArray[i]; //connect nodes
					insert(&(*tree)->right);
					child = 0;
				}
			}
		}
	}
}

void main() {
	// tas bort sen:
	MAP_junctionDistArray[0][1] = 3;	MAP_junctionDistArray[1][0] = 3;
	MAP_junctionDistArray[1][2] = 4;	MAP_junctionDistArray[2][1] = 4;
	MAP_junctionDistArray[2][3] = 5;	MAP_junctionDistArray[3][2] = 5;
	MAP_junctionDistArray[3][5] = 7;	MAP_junctionDistArray[5][3] = 7;
	MAP_junctionDistArray[5][0] = 13;	MAP_junctionDistArray[0][5] = 13;
	MAP_junctionDistArray[5][6] = 2;	MAP_junctionDistArray[6][5] = 2;
	MAP_junctionDistArray[6][0] = 7;	MAP_junctionDistArray[0][6] = 7;
	MAP_junctionDistArray[1][6] = 6;	MAP_junctionDistArray[6][1] = 6;
	/////////////////////////////////////////////////////////////////////
	initNodeArray();
	initPaths();

	node * root;
	root = nodeArray[3];//MAP_currentJunction];
	root->nr = 3;//MAP_currentJunction;
	insert(&root); // eventually the path is saved in a list

	// the tree nodes point to nodeArray (left, middle, right, parent)
	// should be enough to flush this array to free memory
	flushNodeArray();
	root->left = root->middle = root->right = root->parent = NULL;
	root = NULL;
	free(root);

	printPath();
}

// TO DO:
// - change max value for index i in all functions to MAP_junctionCount +/-1 (?)
// - MAP_junctionCount should count start/destination squares as junctions (med Robert)
// - MAP_junctionDistArray do for start/destination squares also (med Robert)
// - MAP_startJunction needed or not? maybe can just set to 0 instead (med Robert)
// - free memory where tree nodes are when path is found /done
// - set node numbers /done
// - initialize the nodeArray /done
// - parent control: to avoid infinite loops /done
// - cost /done
// - create a list (or smth else, mb array) to save the shortest path /done
// - memory allocation for every node: /done
// - print result
// - number items in path-lists
// - new log
// - segmentation fault in function nodeExists
