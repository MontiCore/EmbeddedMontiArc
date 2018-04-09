#ifndef BA_SYSTEM_INTERSECTIONCONTROLLER
#define BA_SYSTEM_INTERSECTIONCONTROLLER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "ba_system_intersectionController_singleSetCompare.h"
#include "ba_system_intersectionController_trajectoryCollision_1_.h"
#include "ba_system_intersectionController_collisionToStop.h"
using namespace arma;
class ba_system_intersectionController{
const int n = 2;
const int x = 1;
const int m = 10;
const int m1 = 4;
const int m2 = 10;
public:
mat trajectoryIn[2];
double cutoffTime;
bool isActive;
bool stop[2];
ba_system_intersectionController_singleSetCompare singleSetCompare;
ba_system_intersectionController_trajectoryCollision_1_ trajectoryCollision[1];
ba_system_intersectionController_collisionToStop collisionToStop;
void init()
{
trajectoryIn[0]=mat(3,m);
trajectoryIn[1]=mat(3,m);
singleSetCompare.init();
trajectoryCollision[0].init();
collisionToStop.init();
}
void execute()
{
singleSetCompare.setIn[0] = trajectoryIn[0];
singleSetCompare.setIn[1] = trajectoryIn[1];
singleSetCompare.execute();
trajectoryCollision[0].trajectoryA = singleSetCompare.outA[0];
trajectoryCollision[0].trajectoryB = singleSetCompare.outB[0];
trajectoryCollision[0].cutoffTime = cutoffTime;
trajectoryCollision[0].execute();
collisionToStop.collisionIn[0] = trajectoryCollision[0].collision;
collisionToStop.aIsFasterIn[0] = trajectoryCollision[0].aIsFasterOut;
collisionToStop.active = isActive;
collisionToStop.execute();
stop[0] = collisionToStop.stopOut[0];
stop[1] = collisionToStop.stopOut[1];
}

};
#endif
