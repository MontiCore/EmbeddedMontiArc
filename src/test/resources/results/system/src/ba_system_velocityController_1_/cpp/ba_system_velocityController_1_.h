#ifndef BA_SYSTEM_VELOCITYCONTROLLER_1_
#define BA_SYSTEM_VELOCITYCONTROLLER_1_
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class ba_system_velocityController_1_{
public:
double holdTimeIn;
double maxVelIn;
double maxAccelIn;
double deltaTimeIn;
bool slowDownIn;
bool resetVelIn;
double curVelOut;
double lastVel;
double holdFor;
void init(double holdTimeIn)
{
this->holdTimeIn=holdTimeIn;

lastVel=0;
holdFor=0;
}
void execute()
{
double resVel = lastVel;
if(resetVelIn){
lastVel = maxVelIn;
resVel = maxVelIn;
holdFor = 0;
}
else {
if(slowDownIn){
resVel = lastVel-maxAccelIn*deltaTimeIn;
holdFor = holdTimeIn;
}
else {
holdFor = holdFor-deltaTimeIn;
if((holdFor <= 0)){
resVel = lastVel+maxAccelIn*(-holdFor);
holdFor = 0;
}
}
}
if((resVel < 0)){
resVel = 0;
}
else if((resVel > maxVelIn)){
resVel = maxVelIn;
}
curVelOut = resVel;
lastVel = curVelOut;
}

};
#endif
