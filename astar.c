#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>

#include "pq.h"
#include "astar.h"
#include "mikes_logs.h"

int path[MAX_PATH_LENGHT][2];
int path_len;
int sqx; // actual square x
int sqy; // actual square y

static pthread_mutex_t path_mutex;

void init_astar()
{
    pthread_mutex_init(&path_mutex, 0);
}

void path_lock()
{
    pthread_mutex_lock(&path_mutex);
}

void path_unlock()
{
    pthread_mutex_unlock(&path_mutex);
}

int astar(int *map, int map_w, int map_h, int startcol, int startrow, int goalcol, int goalrow, int (distance)(int, int, int, int))
{
        printf("astar from: %d\n", map[startrow * map_w + startcol]);
	int arrived_from[map_h][map_w][2];
	for (int i = 0; i < map_h; ++i)
		for (int j = 0; j < map_w; ++j)
			arrived_from[i][j][0] = -1;
	
	int delta[4][2] = {{-1,0}, {0,-1}, {1,0}, {0,1}};
	
        int number_of_walls = map[(startrow - 1) * map_w + startcol] + map[(startrow + 1) * map_w + startcol] + map[startrow * map_w + startcol - 1] + map[startrow * map_w + startcol + 1];
	pq_modify_or_insert(startrow, startcol, distance(startrow, startcol, goalrow, goalcol), number_of_walls, 0, 0);
	while (!pq_is_empty())
	{
		int r;
		int c;
		int root_dist;
		pq_pop(&r, &c, &root_dist);
		if (r == goalrow && c == goalcol)
		{
			int newr = r;
			int newc = c;

                     path_lock();

			path_len = root_dist;
			while (r != startrow || c != startcol)
			{
				r = newr;
				c = newc;
				path[root_dist][0] = r;
				path[root_dist][1] = c;
				root_dist--;
				newr = arrived_from[r][c][0];
				newc = arrived_from[r][c][1];
			}

                    path_unlock();

                        pq_clear();
			return 1;
		} else {
			for (int i = 0; i < 4; ++i)
			{
				if (map[(r+delta[i][0])*map_w + c+delta[i][1]] == 0)
				{
					number_of_walls = map[(r+delta[i][0] - 1) * map_w + c+delta[i][1]] + map[(r+delta[i][0] + 1) * map_w + c+delta[i][1]] + map[(r+delta[i][0]) * map_w + c+delta[i][1] - 1] + map[(r+delta[i][0]) * map_w + c+delta[i][1] + 1];
					if (pq_modify_or_insert(r+delta[i][0], c+delta[i][1], distance(r+delta[i][0], c+delta[i][1], goalrow, goalcol) + root_dist, number_of_walls, root_dist + 1, 
											arrived_from[r+delta[i][0]][c+delta[i][1]][0] + 1))
					{
						arrived_from[r+delta[i][0]][c+delta[i][1]][0] = r;
						arrived_from[r+delta[i][0]][c+delta[i][1]][1] = c;
					}
					
				}
			}
		}
	}
	return 0;
}

// manhatan distance
int rectilinear(int r1, int c1, int r2, int c2)
{
	return abs(r1 - r2) + abs(c1 - c2);
}
