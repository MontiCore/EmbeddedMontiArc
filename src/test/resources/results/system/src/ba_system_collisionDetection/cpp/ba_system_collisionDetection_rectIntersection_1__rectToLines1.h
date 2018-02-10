#ifndef BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__RECTTOLINES1
#define BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__RECTTOLINES1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "types/ba_struct_Rectangle.h"
#include "ba_system_collisionDetection_rectIntersection_1__rectToLines1_rectToPoints.h"
#include "ba_system_collisionDetection_rectIntersection_1__rectToLines1_pointsToLine_1_.h"
using namespace arma;
class ba_system_collisionDetection_rectIntersection_1__rectToLines1{
public:
ba_struct_Rectangle rect;
mat lines[4];
ba_system_collisionDetection_rectIntersection_1__rectToLines1_rectToPoints rectToPoints;
ba_system_collisionDetection_rectIntersection_1__rectToLines1_pointsToLine_1_ pointsToLine[4];
void init()
{
lines[0]=mat(4,1);
lines[1]=mat(4,1);
lines[2]=mat(4,1);
lines[3]=mat(4,1);
rectToPoints.init();
pointsToLine[0].init();
pointsToLine[1].init();
pointsToLine[2].init();
pointsToLine[3].init();
}
void execute()
{
rectToPoints.rect = rect;
rectToPoints.execute();
pointsToLine[0].pointA = rectToPoints.points[0];
pointsToLine[0].pointB = rectToPoints.points[1];
pointsToLine[0].execute();
pointsToLine[1].pointA = rectToPoints.points[1];
pointsToLine[1].pointB = rectToPoints.points[2];
pointsToLine[1].execute();
pointsToLine[2].pointA = rectToPoints.points[2];
pointsToLine[2].pointB = rectToPoints.points[3];
pointsToLine[2].execute();
pointsToLine[3].pointA = rectToPoints.points[3];
pointsToLine[3].pointB = rectToPoints.points[0];
pointsToLine[3].execute();
lines[0] = pointsToLine[0].line;
lines[1] = pointsToLine[1].line;
lines[2] = pointsToLine[2].line;
lines[3] = pointsToLine[3].line;
}

};
#endif
