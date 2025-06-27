#ifndef DE_RWTH_MONTICAR_OPTIMIZATION_HS71
#define DE_RWTH_MONTICAR_OPTIMIZATION_HS71
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt121.h"
using namespace arma;
class de_rwth_monticar_optimization_hS71{
public:
colvec xOut;
double yOut;
void init()
{
xOut=colvec(4);
}
void execute()
{
colvec x;
double y;
CallIpopt121::solveOptimizationProblemIpOpt(&x, &y, xOut, yOut);
xOut = x;
yOut = y;
}

};
#endif
