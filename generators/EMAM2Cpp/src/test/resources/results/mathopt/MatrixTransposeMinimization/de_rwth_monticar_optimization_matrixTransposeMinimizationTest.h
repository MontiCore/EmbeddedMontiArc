#ifndef DE_RWTH_MONTICAR_OPTIMIZATION_MATRIXTRANSPOSEMINIMIZATIONTEST
#define DE_RWTH_MONTICAR_OPTIMIZATION_MATRIXTRANSPOSEMINIMIZATIONTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt121.h"
using namespace arma;
class de_rwth_monticar_optimization_matrixTransposeMinimizationTest{
public:
colvec xOut;
double yOut;
colvec CONSTANTCONSTANTVECTOR0;
void init()
{
xOut=colvec(2);
CONSTANTCONSTANTVECTOR0 = colvec(4);
CONSTANTCONSTANTVECTOR0(0,0) = 1;
CONSTANTCONSTANTVECTOR0(1,0) = 2;
CONSTANTCONSTANTVECTOR0(2,0) = 3;
CONSTANTCONSTANTVECTOR0(3,0) = 4;
}
void execute()
{
colvec x=colvec(2);
double y;
CallIpopt121::solveOptimizationProblemIpOpt(&x, &y, xOut, yOut);
xOut = x;
yOut = y;
}

};
#endif
