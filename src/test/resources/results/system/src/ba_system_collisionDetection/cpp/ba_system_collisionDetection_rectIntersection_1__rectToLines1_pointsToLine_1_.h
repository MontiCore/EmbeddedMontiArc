#ifndef BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__RECTTOLINES1_POINTSTOLINE_1_
#define BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__RECTTOLINES1_POINTSTOLINE_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "types/ba_struct_Point.h"
using namespace arma;
class ba_system_collisionDetection_rectIntersection_1__rectToLines1_pointsToLine_1_{
public:
ba_struct_Point pointA;
ba_struct_Point pointB;
mat line;
void init()
{
line=mat(4,1);
}
void execute()
{
line(1-1, 1-1) = pointA.posX;
line(2-1, 1-1) = pointA.posY;
line(3-1, 1-1) = pointB.posX;
line(4-1, 1-1) = pointB.posY;
}

};
#endif
