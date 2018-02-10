#ifndef BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__TIMECUTOFFFILTER
#define BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__TIMECUTOFFFILTER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "HelperA.h"
using namespace arma;
class ba_system_intersectionController_trajectoryCollision_1__timeCutoffFilter{
const int m = 10;
public:
double timeCutoff;
bool collisionIn;
int indexInA;
int indexInB;
mat colPosIn;
mat trajectoryInA;
mat trajectoryInB;
bool collisionOut;
bool aIsFasterOut;
void init(double timeCutoff)
{
this->timeCutoff=timeCutoff;

colPosIn=mat(2,1);
trajectoryInA=mat(3,m);
trajectoryInB=mat(3,m);
}
void execute()
{
if(((collisionIn)&&((indexInA >= 1))&&((indexInB >= 1))&&((indexInA <= m-1))&&((indexInB <= m-1)))){
double dx1 = trajectoryInA(1-1, indexInA-1)-trajectoryInA(1-1, indexInA+1-1);
double dy1 = trajectoryInA(2-1, indexInA-1)-trajectoryInA(2-1, indexInA+1-1);
double dist1 = (sqrt(dx1*dx1+dy1*dy1));
double dx2 = trajectoryInB(1-1, indexInB-1)-trajectoryInB(1-1, indexInB+1-1);
double dy2 = trajectoryInB(2-1, indexInB-1)-trajectoryInB(2-1, indexInB+1-1);
double dist2 = (sqrt(dx2*dx2+dy2*dy2));
double dxp1 = trajectoryInA(1-1, indexInA-1)-colPosIn(1-1, 1-1);
double dyp1 = trajectoryInA(2-1, indexInA-1)-colPosIn(2-1, 1-1);
double distp1 = (sqrt(dxp1*dxp1+dyp1*dyp1));
double dxp2 = trajectoryInB(1-1, indexInB-1)-colPosIn(1-1, 1-1);
double dyp2 = trajectoryInB(2-1, indexInB-1)-colPosIn(2-1, 1-1);
double distp2 = (sqrt(dxp2*dxp2+dyp2*dyp2));
double time1 = distp1/dist1*(trajectoryInA(3-1, indexInA+1-1)-trajectoryInA(3-1, indexInA-1));
double time2 = distp2/dist2*(trajectoryInB(3-1, indexInB+1-1)-trajectoryInB(3-1, indexInB-1));
if((time1 <= time2)){
aIsFasterOut = true;
}
else {
aIsFasterOut = false;
}
collisionOut = ((abs(time1-time2)) < timeCutoff);
}
else {
collisionOut = false;
}
}

};
#endif
