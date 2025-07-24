/* (c) https://github.com/MontiCore/monticore */
#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP
#define BA_INTERSECTION_INTERSECTIONCONTROLLER_TRAJECTORYTOSTOP
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "ba_intersection_intersectionController_trajectoryToStop_conflictToStopLookup.h"
#include "ba_intersection_intersectionController_trajectoryToStop_conflictComputer.h"
#include "ba_intersection_intersectionController_trajectoryToStop_conflictToStopConverter.h"
using namespace arma;
class ba_intersection_intersectionController_trajectoryToStop{
const int n = 4;
const int x = 6;
const int m = 5;
public:
mat trajectoryIn[4];

double cutoffPos;
double cutoffTime;

bool stop[4];

ba_intersection_intersectionController_trajectoryToStop_conflictToStopLookup conflictToStopLookup;
ba_intersection_intersectionController_trajectoryToStop_conflictComputer conflictComputer;
ba_intersection_intersectionController_trajectoryToStop_conflictToStopConverter conflictToStopConverter;

void init()
{
for(int i = 0; i < 4; ++i){
    trajectoryIn[i] = mat(3,m);
}
conflictToStopLookup.init();
conflictComputer.init();
conflictToStopConverter.init();
}
void execute()
{
for(int i = 0; i < n; i++){
    conflictComputer.trajectoryIn[i] = trajectoryIn[i];
}
conflictComputer.cutoffPos = cutoffPos;
conflictComputer.cutoffTime = cutoffTime;
for(int i = 0; i < x; i++){
    conflictToStopConverter.conflictIn[i] = conflictComputer.conflictOut[i];
}
for(int i = 0; i < x; i++){
    conflictToStopConverter.indexLookupIn[i] = conflictToStopLookup.indexLookup[i];
}
for(int i = 0; i < n; i++){
    stop[i] = conflictToStopConverter.stopOut[i];
}
conflictToStopLookup.execute();
conflictComputer.execute();
conflictToStopConverter.execute();
}

};
#endif
