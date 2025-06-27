/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_PORTTYPECUBETEST
#define TESTING_PORTTYPECUBETEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class testing_portTypeCubeTest{
public:
cube cubeIn;
cube cubeOut;
void init()
{
cubeIn = cube(3, 224, 224);
cubeOut = cube(3, 224, 224);
}
void execute()
{
cubeOut = cubeIn;
}

};
#endif
