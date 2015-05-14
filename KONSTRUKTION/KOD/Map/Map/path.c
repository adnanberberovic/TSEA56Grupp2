#include<stdlib.h>
#include<stdio.h>
#include "Path.h"
#include "Map.h"

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
	head1->next = head2->next = NULL;
}

// inserts an item in front of a list
void buildList(node ** tree, int isFirst) {
	if (isFirst) {
		while ((*tree)->parent) {
			item * curr;
			curr = (item *)malloc(sizeof(item));
			curr->next = head1;
			head1 = curr;
		}
	}
	else {
		while ((*tree)->parent) {
			item * curr;
			curr = (item *)malloc(sizeof(item));
			curr->next = head2;
			head2 = curr;
		}
	}
}

// deletes the list (except: doesn't free the head node)
void flushList(item ** head) {
	if (!head) { //empty
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
	if (head1->cost >= head2->cost) {
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
	} while (curr->parent != 0);

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
	unsigned int i, dist, child=1; // osäker om child kan överskrivas eller ej i olika nivåer
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
	initNodeArray();
	initPaths();

	node * root;
	root = nodeArray[MAP_currentJunction];
	root->nr = MAP_currentJunction;
	insert(&root); // eventually the path is saved in a list

	// the tree nodes point to nodeArray (left, middle, right, parent)
	// should be enough to flush this array to free memory
	flushNodeArray();
	root->left = root->middle = root->right = root->parent = NULL;
	root = NULL;
	free(root);
}

// TO DO:
// - free memory where tree nodes are when path is found /done
// - set node numbers /done
// - initialize the nodeArray /done
// - parent control: to avoid infinite loops /done
// - cost /done
// - create a list (or smth else, mb array) to save the shortest path /done
// - memory allocation for every node: /done
// - new log
