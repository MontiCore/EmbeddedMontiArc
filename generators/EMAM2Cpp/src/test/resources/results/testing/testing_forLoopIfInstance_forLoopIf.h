/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_FORLOOPIFINSTANCE_FORLOOPIF
#define TESTING_FORLOOPIFINSTANCE_FORLOOPIF
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
class testing_forLoopIfInstance_forLoopIf{
public:
double result;
void init()
{
}
void execute()
{
for( auto i=1;i<=8;i+=1){
result = result+counter;
}
if((counter < 0)){
result = 0;
}
else if((counter < 100)){
result = counter;
}
else {
result = 100;
}
}

};
#endif
