#ifndef BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__FIRSTLINEINTERSECTION
#define BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__FIRSTLINEINTERSECTION
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_intersectionController_trajectoryCollision_1__firstLineIntersection{
const int m1 = 9;
const int m2 = 45;
public:
bool collisionIn[45];
mat pointsIn[45];
bool collisionOut;
int lIndexA;
int lIndexB;
mat pos;
void init()
{
pointsIn[0]=mat(2,1);
pointsIn[1]=mat(2,1);
pointsIn[2]=mat(2,1);
pointsIn[3]=mat(2,1);
pointsIn[4]=mat(2,1);
pointsIn[5]=mat(2,1);
pointsIn[6]=mat(2,1);
pointsIn[7]=mat(2,1);
pointsIn[8]=mat(2,1);
pointsIn[9]=mat(2,1);
pointsIn[10]=mat(2,1);
pointsIn[11]=mat(2,1);
pointsIn[12]=mat(2,1);
pointsIn[13]=mat(2,1);
pointsIn[14]=mat(2,1);
pointsIn[15]=mat(2,1);
pointsIn[16]=mat(2,1);
pointsIn[17]=mat(2,1);
pointsIn[18]=mat(2,1);
pointsIn[19]=mat(2,1);
pointsIn[20]=mat(2,1);
pointsIn[21]=mat(2,1);
pointsIn[22]=mat(2,1);
pointsIn[23]=mat(2,1);
pointsIn[24]=mat(2,1);
pointsIn[25]=mat(2,1);
pointsIn[26]=mat(2,1);
pointsIn[27]=mat(2,1);
pointsIn[28]=mat(2,1);
pointsIn[29]=mat(2,1);
pointsIn[30]=mat(2,1);
pointsIn[31]=mat(2,1);
pointsIn[32]=mat(2,1);
pointsIn[33]=mat(2,1);
pointsIn[34]=mat(2,1);
pointsIn[35]=mat(2,1);
pointsIn[36]=mat(2,1);
pointsIn[37]=mat(2,1);
pointsIn[38]=mat(2,1);
pointsIn[39]=mat(2,1);
pointsIn[40]=mat(2,1);
pointsIn[41]=mat(2,1);
pointsIn[42]=mat(2,1);
pointsIn[43]=mat(2,1);
pointsIn[44]=mat(2,1);
pos=mat(2,1);
}
void execute()
{
int k = 1;
bool found = false;
for( auto i=1;i<=m1;++i){
for( auto j=i;j<=m1;++j){
if((found == false)){
if(collisionIn[k-1]){
lIndexA = i;
lIndexB = j;
pos = pointsIn[k-1];
found = true;
}
k = k+1;
}
}
}
collisionOut = found;
}

};
#endif
