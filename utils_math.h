#ifndef __BRACELET_UTILS_MATH__
#define __BRACELET_UTILS_MATH__

#include <stdint.h>
#include "math.h"

typedef struct point{
    int16_t x;
    int16_t y;
}point;

typedef struct circle{
    int16_t x;
    int16_t y;
    int16_t radius;
}circle;


int is_point_in_circle( circle* c,  point p);
void circle_intersection( circle* c1, circle* c2, point* p1, point* p2);
#endif
