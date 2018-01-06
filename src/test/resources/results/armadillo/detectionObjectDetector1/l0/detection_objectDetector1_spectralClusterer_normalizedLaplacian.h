#ifndef DETECTION_OBJECTDETECTOR1_SPECTRALCLUSTERER_NORMALIZEDLAPLACIAN
#define DETECTION_OBJECTDETECTOR1_SPECTRALCLUSTERER_NORMALIZEDLAPLACIAN
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "HelperA.h"
using namespace arma;
class detection_objectDetector1_spectralClusterer_normalizedLaplacian{
const int n = 2500;
public:
mat degree;
mat W;
mat nLaplacian;
void init()
{
degree=mat(n,n);
W=mat(n,n);
nLaplacian=mat(n,n);
}
void execute()
{
nLaplacian = (HelperA::getSqrtMat((inv(degree))))*W*(HelperA::getSqrtMat((inv(degree))));
}

};
#endif
