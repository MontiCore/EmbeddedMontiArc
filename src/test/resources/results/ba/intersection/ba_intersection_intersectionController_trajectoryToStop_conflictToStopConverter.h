/* (c) https://github.com/MontiCore/monticore */
#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP_CONFLICTTOSTOPCONVERTER
#define BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP_CONFLICTTOSTOPCONVERTER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_intersection_intersectionController_trajectoryToStop_conflictToStopConverter{
const int n = 3;
const int x = 6;
const int m = 5;
public:
sword indexLookupIn[6];
bool conflictIn[6];
bool stopOut[3];
void init()
{
}
void execute()
{
    for(sword i = 1;i <= n;++i){
        stopOut[i-1] = false;
    }
    
    for(sword i = 1;i <= x;++i){
        if(conflictIn[i-1]){
            sword curIndex = indexLookupIn[i-1];
            stopOut[curIndex-1] = true;
        }
    }
}

};
#endif
