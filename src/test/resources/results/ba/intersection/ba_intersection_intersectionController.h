#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER
#define BA_INTERSECTION_INTERSECTIONCONTROLLER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "ba_intersection_intersectionController_conflictToStopLookup.h"
#include "ba_intersection_intersectionController_deltaTrajectoryComputer.h"
#include "ba_intersection_intersectionController_conflictComputer.h"
#include "ba_intersection_intersectionController_conflictToStopConverter.h"
using namespace arma;
class ba_intersection_intersectionController{
public:
mat trajectoryIn[3];
double cutoffPosition;
double cutoffTime;
bool stop[3];
ba_intersection_intersectionController_conflictToStopLookup conflictToStopLookup;
ba_intersection_intersectionController_deltaTrajectoryComputer deltaTrajectoryComputer;
ba_intersection_intersectionController_conflictComputer conflictComputer;
ba_intersection_intersectionController_conflictToStopConverter conflictToStopConverter;
void init()
{
for(int i = 0; i < 3; ++i){
    trajectoryIn[i] = mat(3,5);
}
conflictToStopLookup.init();
deltaTrajectoryComputer.init();
conflictComputer.init();
conflictToStopConverter.init();
}
void execute()
{
for(int i = 0; i < 3; i++){
    deltaTrajectoryComputer.trajectoryIn[i] = trajectoryIn[i];
}
for(int i = 0; i < 6; i++){
    conflictComputer.deltaTrajectoryIn[i] = deltaTrajectoryComputer.deltaTrajectoryOut[i];
}
conflictComputer.cutoffPosition = cutoffPosition;
conflictComputer.cutoffTime = cutoffTime;
for(int i = 0; i < 6; i++){
    conflictToStopConverter.conflictIn[i] = conflictComputer.conflictOut[i];
}
for(int i = 0; i < 6; i++){
    conflictToStopConverter.indexLookupIn[i] = conflictToStopLookup.indexLookup[i];
}
for(int i = 0; i < 3; i++){
    stop[i] = conflictToStopConverter.stopOut[i];
}
conflictToStopLookup.execute();
deltaTrajectoryComputer.execute();
conflictComputer.execute();
conflictToStopConverter.execute();
}

};
#endif
