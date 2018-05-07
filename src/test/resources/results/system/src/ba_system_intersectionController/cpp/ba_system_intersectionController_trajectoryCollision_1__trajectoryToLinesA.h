#ifndef BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__TRAJECTORYTOLINESA
#define BA_SYSTEM_INTERSECTIONCONTROLLER_TRAJECTORYCOLLISION_1__TRAJECTORYTOLINESA
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_intersectionController_trajectoryCollision_1__trajectoryToLinesA{
const int m = 10;
const int m1 = 9;
public:
mat trajectoryIn;
mat lineOut[9];
void init()
{
trajectoryIn=mat(3,m);
lineOut[0]=mat(4,1);
lineOut[1]=mat(4,1);
lineOut[2]=mat(4,1);
lineOut[3]=mat(4,1);
lineOut[4]=mat(4,1);
lineOut[5]=mat(4,1);
lineOut[6]=mat(4,1);
lineOut[7]=mat(4,1);
lineOut[8]=mat(4,1);
}
void execute()
{
for( auto i=1;i<=m1;++i){
colvec tmpLine=colvec(4);
tmpLine(1-1, 1-1) = trajectoryIn(1-1, i-1);
tmpLine(2-1, 1-1) = trajectoryIn(2-1, i-1);
tmpLine(3-1, 1-1) = trajectoryIn(1-1, i+1-1);
tmpLine(4-1, 1-1) = trajectoryIn(2-1, i+1-1);
lineOut[i-1] = tmpLine;
}
}

};
#endif
