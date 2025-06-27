#ifndef DE_RWTH_MONTICAR_OPTIMIZATION_MATRIXSUMMINIMIZATIONTEST1
#define DE_RWTH_MONTICAR_OPTIMIZATION_MATRIXSUMMINIMIZATIONTEST1
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo"
#include "CallIpopt121.h"
using namespace arma;
class de_rwth_monticar_optimization_matrixSumMinimizationTest1{
public:
mat xOut;
double yOut;
void init()
{
xOut=mat(2,2);
}
void execute()
{
mat x=mat(2,2);
double y;
CallIpopt121::solveOptimizationProblemIpOpt(&x, &y, xOut, yOut);
xOut = x;
yOut = y;
}

};
#endif
