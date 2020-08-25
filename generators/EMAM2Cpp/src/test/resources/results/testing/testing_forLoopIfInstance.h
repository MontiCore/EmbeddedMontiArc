/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_FORLOOPIFINSTANCE
#define TESTING_FORLOOPIFINSTANCE
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "octave/oct.h"
#include "testing_forLoopIfInstance_forLoopIf.h"
class testing_forLoopIfInstance{
public:
double result;
testing_forLoopIfInstance_forLoopIf forLoopIf;
void init()
{
forLoopIf.init();
}
void execute()
{
forLoopIf.execute();
result = forLoopIf.result;
}

};
#endif
