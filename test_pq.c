#include <stdio.h>

#include "pq.h"

int main() {
	
	int r;
	int c;
        int dist;
	
	pq_modify_or_insert(1,1,3,0,0);
	pq_modify_or_insert(2,2,4,0,0);
	pq_modify_or_insert(3,3,1,0,0);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_modify_or_insert(4,4,4,0,0);
	pq_modify_or_insert(5,5,1,0,0);
	pq_modify_or_insert(6,6,5,0,0);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_modify_or_insert(7,7,3,0,0);
	pq_modify_or_insert(8,8,2,0,0);
	pq_modify_or_insert(9,9,3,0,0);
	pq_modify_or_insert(10,10,4,0,0);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	pq_pop(&r, &c, &dist);
	printf("%d %d\n", r, c);
	
	return 0;
}
