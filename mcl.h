#ifndef _MCL_MIKES_H_
#define _MCL_MIKES_H_

#include "lidar.h"

#define MAP_W 4578
#define MAP_H 4303

typedef struct {
  double x, y, alpha;
  double w;
} hypo_t;

typedef struct {
    double x;
    double y;
} point;

typedef struct {
   double x1;
   double y1;
   double x2;
   double y2;
} line;

extern point poly_i[];
extern point poly_a[];
extern point poly_h[];
extern line lines[];

#define HYPO_COUNT 2500

int init_mcl();
void get_mcl_data(hypo_t *buffer);
int mcl_update(double traveled, int heading, lidar_data_type liddata);
double line_intersection(
                         double x1, double y1,
                         double x2, double y2,
                         double x3, double y3,
                         double x4, double y4,
                         double *X, double *Y);
double get_min_intersection_dist(double x1, double y1, double alpha);



#endif
