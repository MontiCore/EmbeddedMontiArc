/* (c) https://github.com/MontiCore/monticore */
#ifndef TESTING_WHOLENUMBERMATRIX
#define TESTING_WHOLENUMBERMATRIX
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
using namespace arma;
class testing_wholeNumberMatrix{
public:
imat matrix;
ivec columnVector;
void init()
{
matrix=imat(3,3);
columnVector=ivec(3);
}
void execute()
{
imat varMatrix=imat(3,3);
ivec varColumnVector=ivec(3);
}

};
#endif
