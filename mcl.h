#ifndef _MCL_MIKES_H_
#define _MCL_MIKES_H_

#include "lidar.h"

typedef struct {
  double x, y, alpha;
  double w;
} hypo_t;

typedef struct {
    double x;
    double y;
} point;

#define HYPO_COUNT 2500

int init_mcl();
void get_mcl_data(hypo_t *buffer);
int mcl_update(double traveled, int heading, lidar_data_type liddata);
double line_intersection(double Ax, double Ay,
                         double Bx, double By,
                         double Cx, double Cy,
                         double Dx, double Dy);
   
double line_inter_poly_i(double Ax, double Ay,
                         double Bx, double By, int i);

#endif
