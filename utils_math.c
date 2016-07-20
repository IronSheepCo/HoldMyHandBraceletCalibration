#include "utils_math.h"

#include "SEGGER_RTT.h"

int is_point_in_circle( circle* c, point p)
{
 return (c->x-p.x)*(c->x-p.x)+(c->y-p.y)*(c->y-p.y) <= c->radius*c->radius;
}

void circle_intersection( circle* c1, circle* c2, point* p1, point* p2)
{
    int d, a, h;
    int px, py;    
    
    //distance between the circles
    do
    {
        d = (c1->x-c2->x);
        d = d*d;
        a = (c1->y-c2->y);
        a = a * a;
        d = sqrt( d + a );
        
        //if the circles don't intersect let's increase one of the circles radius
        if( d<=c1->radius+c2->radius ){
            
            break;
        }

        SEGGER_RTT_printf(0, "increasing radius %d\n", d - c1->radius-c2->radius+20);
        c1->radius = c1->radius + d - c1->radius-c2->radius+20;
    }while( 1 );//d<c1->radius+c2->radius);

    a = c1->radius;
    a = a*a;
    h = c2->radius;
    h = h*h;
    a = ( a - h + d*d )/(2*d);

    h = sqrt( c1->radius*c1->radius - a*a );

    px = c1->x + a *(c2->x-c1->x)/d;
    py = c1->y + a *(c2->y-c1->y)/d;

    //SEGGER_RTT_printf(0, "d: %d a: %d h: %d px: %d py: %d\n", d, a, h, px, py);

    p1->x = px + h*(c2->y-c1->y)/d;
    p1->y = py - h*(c2->x-c1->x)/d;

    p2->x = px - h*(c2->y-c1->y)/d;
    p2->y = py + h*(c2->x-c1->x)/d;
}
