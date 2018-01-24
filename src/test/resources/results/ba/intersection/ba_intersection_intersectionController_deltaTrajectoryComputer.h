#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER_DELTATRAJECTORYCOMPUTER
#define BA_INTERSECTION_INTERSECTIONCONTROLLER_DELTATRAJECTORYCOMPUTER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class ba_intersection_intersectionController_deltaTrajectoryComputer{
const int n = 3;
const int x = 6;
const int m = 5;
mat trajectoryIn[n];
mat deltaTrajectoryOut[x];
public:
void init()
{
    for(int i = 0; i < n; ++i){
        trajectoryIn[i] = mat(3,m);
    }
    for(int i = 0; i < x; ++i){
        deltaTrajectoryOut[i] = mat(3,m);
    }
}
void execute()
{
    sword k = 1;
    sword maxI = n - 1;
    for(sword i = 1; i <= maxI; ++i){
        sword minJ = i + 1;
        for(sword j = minJ;j <= n; ++j){
            deltaTrajectoryOut[k-1] = trajectoryIn[i-1] - trajectoryIn[j-1];
            k = k + 1;
        }
    }
}

};
#endif
