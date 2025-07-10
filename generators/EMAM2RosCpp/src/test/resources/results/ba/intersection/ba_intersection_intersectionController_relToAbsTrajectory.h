/* (c) https://github.com/MontiCore/monticore */
#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER_RELTOABSTRAJECTORY
#define BA_INTERSECTION_INTERSECTIONCONTROLLER_RELTOABSTRAJECTORY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_intersection_intersectionController_relToAbsTrajectory{
const int n = 4;
const int m = 5;
public:
mat relTrajectoryIn[4];
colvec absPositionIn[4];
mat absTrajectoryOut[4];

void init()
{
for(int i = 0; i < 4; ++i){
    relTrajectoryIn[i] = mat(3,m);
}
for(int i = 0; i < 4; ++i){
    absPositionIn[i] = colvec(2);
}
for(int i = 0; i < 4; ++i){
    absTrajectoryOut[i] = mat(3,m);
}
}
void execute()
{
    for(sword i = 1; i <= n; ++i){
        mat tmpRelTraj = relTrajectoryIn[i-1];
        colvec tmpAbsPos = absPositionIn[i-1];
        mat tmpAbsTraj = mat(3,m);
        for(sword j = 1; i <= m; ++i){
            tmpAbsTraj(1-1,j-1) = tmpRelTraj(1-1,j-1) + tmpAbsPos(1-1,1-1);
            tmpAbsTraj(2-1,j-1) = tmpRelTraj(2-1,j-1) + tmpAbsPos(2-1,1-1);
            tmpAbsTraj(3-1,j-1) = tmpRelTraj(3-1,j-1);
        }
        absTrajectoryOut[i-1] = tmpAbsTraj;
    }
}

};
#endif
