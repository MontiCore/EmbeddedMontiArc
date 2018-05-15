#ifndef TESTING_WHOLENUMBERMATRIX
#define TESTING_WHOLENUMBERMATRIX
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
using namespace arma;
class testing_wholeNumberMatrix{
public:
imat matrix;
icol columnVector;
void init()
{
matrix=imat(3,3);
columnVector=icol(3);
}
void execute()
{
imat varMatrix=imat(3,3);
icol varColumnVector=icol(3);
}

};
#endif
