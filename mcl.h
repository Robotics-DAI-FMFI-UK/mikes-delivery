#ifndef _MCL_MIKES_H_
#define _MCL_MIKES_H_

#include "lidar.h"
#include "astar.h"
#include "util.h"

#define HYPO_COUNT 1000

#define NORM_PROB_CONST 0.2

#define LOST_PATH_MCL_TRIGGER_DISTANCE (75 * 75)

typedef struct {
  double x, y, alpha;
  double w;
} hypo_t;


extern point poly_i[];
extern point poly_a[];
extern point poly_h[];
extern line lines[];
extern int world[];


int WorldAt(int x, int y);

void get_mcl_data(hypo_t *buffer);
int mcl_update();
double line_intersection(double x1, double y1, double x2, double y2,
                          double x3, double y3, double x4, double y4,
                          double *X, double *Y);
double get_min_intersection_dist(double x1, double y1, double alpha);
int get_line_intersection(double x1, double y1, double x2, double y2);
int init_world();
int init_mcl();


#endif
