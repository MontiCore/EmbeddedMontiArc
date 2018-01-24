#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER_CONFLICTCOMPUTER
#define BA_INTERSECTION_INTERSECTIONCONTROLLER_CONFLICTCOMPUTER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class ba_intersection_intersectionController_conflictComputer{
const int x = 6;
const int m = 5;
public:
mat deltaTrajectoryIn[x];
double cutoffPosition;
double cutoffTime;
bool conflictOut[x];
void init()
{
    for(int i = 0; i < x;++i){
        deltaTrajectoryIn[i] = mat(3,m);
    }
}
void execute()
{
    for(sword i = 1; i <= x; ++i){
        bool tmp = false;
        if(tmp == false){
            for(sword j = 1; j <=m; ++j){
                mat tmpTrajectory = deltaTrajectoryIn[i-1];
                sword deltaPosSquared = tmpTrajectory(1-1,j-1)*tmpTrajectory(1-1,j-1) + tmpTrajectory(2-1,j-1)*tmpTrajectory(2-1,j-1);
                sword absDeltaTime = abs(tmpTrajectory(3-1,j-1));

                bool condPos = deltaPosSquared <= (cutoffPosition*cutoffPosition);
                bool condTime = absDeltaTime <= cutoffTime;

                if(condPos && condTime){
                    tmp = True;
                }
            }
        }
        conflictOut[i-1] = tmp;
    }
}

};
#endif
