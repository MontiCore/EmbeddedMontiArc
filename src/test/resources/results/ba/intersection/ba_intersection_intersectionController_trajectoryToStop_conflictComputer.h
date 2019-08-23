/* (c) https://github.com/MontiCore/monticore */
#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP_CONFLICTCOMPUTER
#define BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP_CONFLICTCOMPUTER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_intersection_intersectionController_trajectoryToStop_conflictComputer{
const int n = 4;
const int x = 6;
const int m = 5;
public:
mat trajectoryIn[4];
double cutoffPos;
double cutoffTime;
bool conflictOut[6];
void init()
{
    for(int i = 0; i < n;++i){
        trajectoryIn[i] = mat(3,m);
    }
}
void execute()
{
    sword xi = 1;
    
    for(sword ni = 1; ni <= n; ++ni){
    sword minNj = ni + 1;
    for(sword nj = minNj;nj <= n;++nj){
        bool conflictFlag = false;
        mat tmpTrajA = trajectoryIn[ni-1];
        mat tmpTrajB = trajectoryIn[nj-1];
    
        for(sword mi = 1; mi <= m; ++mi){
        sword minMj = mi + 1;
        for(sword mj = minMj; mj <= m; ++mj){
            double deltaXSquared = (tmpTrajA(1-1,mi-1) - tmpTrajB(1-1,mj-1)) * (tmpTrajA(1-1,mi-1) - tmpTrajB(1-1,mj-1));
            double deltaYSquared = (tmpTrajA(2-1,mi-1) - tmpTrajB(2-1,mj-1)) * (tmpTrajA(2-1,mi-1) - tmpTrajB(2-1,mj-1));
            double deltaTSquared = (tmpTrajA(3-1,mi-1) - tmpTrajB(3-1,mj-1)) * (tmpTrajA(3-1,mi-1) - tmpTrajB(3-1,mj-1));
    
            bool condPos = deltaXSquared + deltaYSquared <= cutoffPos * cutoffPos;
            bool condTime = deltaTSquared <= cutoffTime * cutoffTime;
    
            if(condPos && condTime){
                conflictFlag = true;
            }
        }
        }
    
        conflictOut[xi-1] = conflictFlag;
        xi = xi + 1;
    }
    }


}

};
#endif
