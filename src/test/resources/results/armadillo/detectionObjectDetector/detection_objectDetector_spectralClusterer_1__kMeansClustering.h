#ifndef DETECTION_OBJECTDETECTOR_SPECTRALCLUSTERER_1__KMEANSCLUSTERING
#define DETECTION_OBJECTDETECTOR_SPECTRALCLUSTERER_1__KMEANSCLUSTERING
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
using namespace arma;
class detection_objectDetector_spectralClusterer_1__kMeansClustering{
const int n = 2500;
public:
mat vectors;
mat clusters;
void init()
{
vectors=mat(n,amountVectors);
clusters=mat(n,1);
}
void execute()
{
mat UMatrix;
for( auto i=1;i<=(Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(vectors, 1),1)));++i){
rowvec target = Helper::callOctaveFunctionRowVectorFirstResult(Helper::convertToOctaveValueList(vectors.row(i-1),2),"power",1);
double amount = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList((Helper::getDoubleFromOctaveListFirstResult(Fsum(Helper::convertToOctaveValueList(target),1)))),1)));
UMatrix.row(i-1) = Helper::callOctaveFunctionRowVectorFirstResult(Helper::convertToOctaveValueList(vectors.row(i-1),amount),"ldivide",1);
}
clusters = kmeansidx(UMatrix, maximumClusters);
}

};
#endif
