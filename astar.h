#ifndef _ASTAR_H_
#define _ASTAR_H_

#define MAX_PATH_LENGHT 500

extern int path[MAX_PATH_LENGHT][2];
extern int path_len;
extern int sqx;
extern int sqy;

void path_lock();
void path_unlock();
void init_astar();

int astar(int *map, int map_w, int map_h, int startcol, int startrow, int goalcol, int goalrow, int (distance)(int, int, int, int));
int rectilinear(int r1, int c1, int r2, int c2);

#endif
