#ifndef _UTIL_H_
#define _UTIL_H_

typedef struct {
    double x;
    double y;
} point;


typedef struct {
   double x1;
   double y1;
   double x2;
   double y2;
   int id;
} line;


// square of distance of two points
double distance(double x1, double y1, double x2, double y2);

// return current time in milliseconds
long msec();

// return current time in usec (up to 71 minutes)
long usec();

// say the sentence
void say(char *sentence);

int get_lines_from_file(const char *filename, line *lines);

#define NUMBER_OF_VERTICES 163
#define NUMBER_OF_VERTICES_I 101
#define NUMBER_OF_VERTICES_A 49
#define NUMBER_OF_VERTICES_H 13

int get_polygons(point* p_i, point* p_a, point* p_h, line *lines);

long usec_time();

double normAlpha(double alpha);

double rad_normAlpha(double alpha);

#endif
