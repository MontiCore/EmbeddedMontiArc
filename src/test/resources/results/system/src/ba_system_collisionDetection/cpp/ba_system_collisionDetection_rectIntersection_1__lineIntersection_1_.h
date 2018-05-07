#ifndef BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__LINEINTERSECTION_1_
#define BA_SYSTEM_COLLISIONDETECTION_RECTINTERSECTION_1__LINEINTERSECTION_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_collisionDetection_rectIntersection_1__lineIntersection_1_{
public:
mat lineA;
mat lineB;
mat point;
bool intersects;
void init()
{
lineA=mat(4,1);
lineB=mat(4,1);
point=mat(2,1);
}
void execute()
{
double Ax = lineA(3-1, 1-1)-lineA(1-1, 1-1);
double Ay = lineA(4-1, 1-1)-lineA(2-1, 1-1);
double Bx = lineB(1-1, 1-1)-lineB(3-1, 1-1);
double By = lineB(2-1, 1-1)-lineB(4-1, 1-1);
double Cx = lineA(1-1, 1-1)-lineB(1-1, 1-1);
double Cy = lineA(2-1, 1-1)-lineB(2-1, 1-1);
double n1 = Ay*Bx-Ax*By;
double cutoff = 1.0E-8;
bool res = false;
if((((abs(n1)) > cutoff))){
double alpha = (By*Cx-Bx*Cy)/n1;
if(((alpha >= cutoff))&&((alpha <= 1))){
double beta = (Ax*Cy-Ay*Cx)/n1;
if(((beta >= cutoff))&&((beta <= 1))){
res = true;
point(1-1, 1-1) = lineA(1-1, 1-1)+alpha*Ax;
point(2-1, 1-1) = lineA(2-1, 1-1)+alpha*Ay;
}
}
}
intersects = res;
}

};
#endif
