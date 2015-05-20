/*
 * path.c
 *
 * Created: 5/5/2015
 * Last modified: 19/5/2015
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
	printf("shortest path:\n");
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

//för felsökning. tas bort sen
void printPathsTest() {
	item * curr1, * curr2;
	curr1 = head1;
	curr2 = head2;

	printf("path1:\n");
	while(curr1) {
      printf("%d\n", curr1->nr);
      curr1 = curr1->next;
	}
	printf("path2:\n");
	while(curr2) {
      printf("%d\n", curr2->nr);
      curr2 = curr2->next;
	}
	printf("-------------\n");
}

// allocate memory to every node in nodeArray[i]
void initNodeArray() {
	unsigned int i;
	for (i=0; i<7; i++) {
		nodeArray[i] = (node *)malloc(sizeof(node));
		nodeArray[i]->left = nodeArray[i]->middle = nodeArray[i]->right = nodeArray[i]->parent = NULL;
		nodeArray[i]->nr = i;
		nodeArray[i]->cost = 0;
	}
}

// free memory the nodes in nodeArray occupy
void flushNodeArray() {
	unsigned int i;
	for (i=0; i<7; i++) {
		nodeArray[i]->left = nodeArray[i]->middle = nodeArray[i]->right = nodeArray[i]->parent = NULL;
		nodeArray[i] = NULL;
		free(nodeArray[i]);
	}
}

// initiate two path-lists
void initPaths() {
	head1 = (item *)malloc(sizeof(item));
	head2 = (item *)malloc(sizeof(item));
	head1->cost = head2->cost = 0;
	head1 = head2 = NULL;
}

// builds a list using nodes from tree
// items are inserted in front of a list
void buildList(node ** tree, int isFirst) {
	node * curr_node = *tree;
	unsigned int total_cost = curr_node->cost;
	if (isFirst) {
		while (curr_node) {
			item * curr_item;
			curr_item = (item *)malloc(sizeof(item));
			curr_item->next = head1;
			curr_item->nr = curr_node->nr;
			head1 = curr_item;
			curr_node = curr_node->parent;
		}
			head1->cost = total_cost;
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
			head2->cost = total_cost;
	}
}

// deletes the list
void flushList(item ** head) {
	if (!(*head)) { //empty
		return;
	}

	item * curr = *head;
	*head = NULL;

	if (curr->next != NULL)
	{
		flushList(&(curr->next));
	}
	free(curr);
}

// compares the two path-lists and flushes the more expensive one
void choosePath() {
	unsigned int cost1=999, cost2=999; // cost sets to infinity (high value)
	if (head1) {
		cost1=head1->cost;
		//printf("cost1=");
		//printf("%u\n", cost1);
	}
	if (head2) {
		cost2=head2->cost;
		//printf("cost2=");
		//printf("%u\n", cost2);
		}
	if (cost2 >= cost1) {
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
	} while (curr != 0);

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
		//printPathsTest();
		// flushes expensive path-list
		choosePath();
		//printPathsTest();
		return;
	}
	unsigned int i, dist, child=1; // child shows the branch to insert in
	for (i=0; i<7; i++) {
//			printf("i=");
//			printf("%d\n", i);
		if (!node_exists(tree, i)) { // parent control
			dist = MAP_junctionDistArray[(*tree)->nr][i];
			if (dist > 0) {
//				printf("parent=");
//				printf("%d\n", (*tree)->nr);
//				printf("kidNR=");
//				printf("%d\n", child);
//				printf("kid=");
//				printf("%d\n", i);
				if (child == 1) {
					nodeArray[i]->cost = (*tree)->cost + dist;
					nodeArray[i]->parent = *tree;
					(*tree)->left = nodeArray[i]; //connect nodes
					insert(&(*tree)->left);
					child++;// = 2;
				}
				else if (child == 2) {
					nodeArray[i]->cost = (*tree)->cost + dist;
					nodeArray[i]->parent = *tree;
					(*tree)->middle = nodeArray[i]; //connect nodes
					insert(&(*tree)->middle);
					child++;// = 3;
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
	MAP_junctionDistArray[3][4] = 3;	MAP_junctionDistArray[4][3] = 3;
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
// - print result /done
// - number items in path-lists /done
// - segmentation fault in function nodeExists /fixed
