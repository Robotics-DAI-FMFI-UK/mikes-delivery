#ifndef _PQ_H_
#define _PQ_H_

#include <stdlib.h>

void pq_pop(int *r, int *c, int *dist);
int pq_modify_or_insert(int row, int col, int new_priority, int secondary_priority, int dist, int already_visited);
void pq_clear();
int pq_is_empty();

#endif
