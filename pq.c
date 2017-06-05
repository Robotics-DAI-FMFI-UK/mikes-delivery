#include <math.h>
#include <string.h>
#include <stdint.h>

#include "pq.h"

typedef struct nodestruct {
	int priority;
	int root_dist;
	int row;
	int col;
	struct nodestruct *next;
} qnode;

static qnode* queue = 0;

void pq_pop(int *r, int *c, int *dist) {
	*r = queue->row;
	*c = queue->col;
	*dist = queue->root_dist;
	qnode* p = queue;
	queue = queue->next;
	free(p);
}

qnode* pq_new_node(int row, int col, int priority, int dist, qnode* next) {
	qnode* p = (qnode*) malloc(sizeof(qnode));
	p->row = row;
	p->col = col;
	p->priority = priority;
	p->root_dist = dist;
	p->next = next;
	return p;
}

int pq_modify_or_insert(int row, int col, int new_priority, int secondary_priority, int dist, int already_visited) {
	
	int was_deleted = 0;
        int combined_priority = new_priority * 10 + secondary_priority;
	
	if (queue == 0) {
		if (already_visited)
			return 0;
		queue = pq_new_node(row, col, combined_priority, dist, 0);
	} else {
		qnode* ptr = queue;
		qnode* previous = 0;
		qnode* where_to_insert = 0;
		int found = 0;
		while (ptr)
		{
			if (ptr->priority > combined_priority && found == 0)
			{
				where_to_insert = previous;
				found = 1;
			}
			if (ptr->row == row && ptr-> col == col)
			{
				if (ptr->priority <= combined_priority)
					return 0;
				if (previous == 0 || previous->priority <= combined_priority)
				{
					ptr->priority = combined_priority;
					return 1;
				}
				was_deleted = 1;
				previous->next = ptr->next;
				break;
			}
			previous = ptr;
			ptr = ptr->next;
		}
		if (found == 0)
		{
			if (already_visited)
				return 0;
			previous->next = pq_new_node(row, col, combined_priority, dist, 0);
		} else {
			if (!was_deleted && already_visited)
				return 0;
			if (where_to_insert == 0)
			{
				queue = pq_new_node(row, col, combined_priority, dist, queue);
			} else {
				where_to_insert->next = pq_new_node(row, col, combined_priority, dist, where_to_insert->next);
			}
		}
	}
	return 1;
}

void pq_clear() {
	while(queue)
	{
		qnode* p = queue;
		queue = queue->next;
		free(p);
	}
}

int pq_is_empty() {
	return (queue == 0);
}
