#ifndef BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__RECTTOLINES2_RECTTOPOINTS
#define BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__RECTTOLINES2_RECTTOPOINTS
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "types/ba_struct_Rectangle.h"
#include "types/ba_struct_Point.h"
using namespace arma;
class ba_system_collisionDetection_rectIntersection_1__rectToLines2_rectToPoints{
public:
ba_struct_Rectangle rect;
ba_struct_Point points[4];
void init()
{
}
void execute()
{
points[1-1] = rect.pointA;
points[2-1] = rect.pointB;
points[3-1] = rect.pointC;
points[4-1] = rect.pointD;
}

};
#endif
