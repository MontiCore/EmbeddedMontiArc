#ifndef DE_RWTH_MONTICAR_OPTIMIZATION_TRANSPORTATIONPROBLEM
#define DE_RWTH_MONTICAR_OPTIMIZATION_TRANSPORTATIONPROBLEM
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt121.h"
using namespace arma;
class de_rwth_monticar_optimization_transportationProblem{
public:
mat xOut;
double yOut;
colvec CONSTANTCONSTANTVECTOR0;
colvec CONSTANTCONSTANTVECTOR1;
mat CONSTANTCONSTANTVECTOR2;
void init()
{
xOut=mat(3,2);
CONSTANTCONSTANTVECTOR0 = colvec(2);
CONSTANTCONSTANTVECTOR0(0,0) = 350;
CONSTANTCONSTANTVECTOR0(1,0) = 600;
CONSTANTCONSTANTVECTOR1 = colvec(3);
CONSTANTCONSTANTVECTOR1(0,0) = 325;
CONSTANTCONSTANTVECTOR1(1,0) = 300;
CONSTANTCONSTANTVECTOR1(2,0) = 275;
CONSTANTCONSTANTVECTOR2 = mat(2,3);
CONSTANTCONSTANTVECTOR2(0,0) = 2.5;
CONSTANTCONSTANTVECTOR2(0,1) = 1.7;
CONSTANTCONSTANTVECTOR2(0,2) = 1.8;
CONSTANTCONSTANTVECTOR2(1,0) = 2.5;
CONSTANTCONSTANTVECTOR2(1,1) = 1.8;
CONSTANTCONSTANTVECTOR2(1,2) = 1.4;
}
void execute()
{
double m = 2;
double n = 3;
colvec A = CONSTANTCONSTANTVECTOR0;
colvec b = CONSTANTCONSTANTVECTOR1;
mat c = CONSTANTCONSTANTVECTOR2;
mat x=mat(2,3);
double y;
CallIpopt121::solveOptimizationProblemIpOpt(&x, &y, xOut, yOut, m, n, A, b, c, CONSTANTCONSTANTVECTOR0, CONSTANTCONSTANTVECTOR1, CONSTANTCONSTANTVECTOR2);
xOut = x;
yOut = y;
}

};
#endif
