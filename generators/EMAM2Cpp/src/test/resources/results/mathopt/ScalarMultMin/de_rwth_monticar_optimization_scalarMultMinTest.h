#ifndef DE_RWTH_MONTICAR_OPTIMIZATION_SCALARMULTMINTEST
#define DE_RWTH_MONTICAR_OPTIMIZATION_SCALARMULTMINTEST
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt121.h"
using namespace arma;
class de_rwth_monticar_optimization_scalarMultMinTest{
public:
double xOut;
double yOut;
rowvec CONSTANTCONSTANTVECTOR0;
colvec CONSTANTCONSTANTVECTOR1;
void init()
{
CONSTANTCONSTANTVECTOR0 = rowvec(2);
CONSTANTCONSTANTVECTOR0(0,0) = 1;
CONSTANTCONSTANTVECTOR0(0,1) = 1;
CONSTANTCONSTANTVECTOR1 = colvec(2);
CONSTANTCONSTANTVECTOR1(0,0) = 2;
CONSTANTCONSTANTVECTOR1(1,0) = 2;
}
void execute()
{
double x;
double y;
CallIpopt121::solveOptimizationProblemIpOpt(&x, &y, xOut, yOut);
xOut = x;
yOut = y;
}

};
#endif
