/* (c) https://github.com/MontiCore/monticore */
#ifndef BA_INTERSECTION_INTERSECTIONCONTROLLER
#define BA_INTERSECTION_INTERSECTIONCONTROLLER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "ba_intersection_intersectionController_trajectoryToStop.h"
#include "ba_intersection_intersectionController_relToAbsTrajectory.h"
using namespace arma;
class ba_intersection_intersectionController{
public:
sword objectIdIn[4];
mat relTrajectoryIn[4];
colvec absPositionIn[4];

double cutoffPos;
double cutoffTime;

sword objectIdOut[4];
bool stopOut[4];

ba_intersection_intersectionController_trajectoryToStop trajectoryToStop;
ba_intersection_intersectionController_relToAbsTrajectory relToAbsTrajectory;

void init()
{
for(int i = 0; i < 4; ++i){
    relTrajectoryIn[i] = mat(3,5);
}
for(int i = 0; i < 4; ++i){
    absPositionIn[i] = colvec(2);
}
trajectoryToStop.init();
relToAbsTrajectory.init();
}
void execute()
{
for(int i = 0; i < 4; i++){
    objectIdOut[i] = objectIdIn[i];
}
trajectoryToStop.cutoffPos = cutoffPos;
trajectoryToStop.cutoffTime = cutoffTime;
for(int i = 0; i < 4; i++){
    relToAbsTrajectory.relTrajectoryIn[i] = relTrajectoryIn[i];
}
for(int i = 0; i < 4; i++){
    relToAbsTrajectory.absPositionIn[i] = absPositionIn[i];
}
for(int i = 0; i < 4; i++){
    trajectoryToStop.trajectoryIn[i] = relToAbsTrajectory.absTrajectoryOut[i];
}
for(int i = 0; i < 4; i++){
    stopOut[i] = trajectoryToStop.stop[i];
}
trajectoryToStop.execute();
relToAbsTrajectory.execute();
}

};
#endif
