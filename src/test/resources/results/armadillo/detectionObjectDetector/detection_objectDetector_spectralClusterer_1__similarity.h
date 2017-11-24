#ifndef DETECTION_OBJECTDETECTOR_SPECTRALCLUSTERER_1__SIMILARITY
#define DETECTION_OBJECTDETECTOR_SPECTRALCLUSTERER_1__SIMILARITY
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
using namespace arma;
class detection_objectDetector_spectralClusterer_1__similarity{
const int n = 2500;
public:
mat data;
mat similarity;
mat degree;
void init()
{
data=mat(n,3);
similarity=mat(n,n);
degree=mat(n,n);
}
void execute()
{
for( auto i=1;i<=(Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(data, 1),1)));++i){
for( auto j=1;j<=(Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(data, 1),1)));++j){
double dist = (Helper::getDoubleFromOctaveListFirstResult(Fsqrt(Helper::convertToOctaveValueList(Helper::getDoubleFromOctaveListFirstResult(Fmpower(Helper::convertToOctaveValueList(Helper::getDoubleFromOctaveListFirstResult(Fmpower(Helper::convertToOctaveValueList(Helper::getDoubleFromOctaveListFirstResult(Fmpower(Helper::convertToOctaveValueList((data(i-1, 1-1)-data(j-1, 1-1)),2+(data(i-1, 2-1)-data(j-1, 2-1))),1)),2+(data(i-1, 3-1)-data(j-1, 3-1))),1)),2),1))),1)));
similarity(i-1, j-1) = (Helper::getDoubleFromOctaveListFirstResult(Fexp(Helper::convertToOctaveValueList((0-dist)/(2)),1)));
}
}
for( auto k=1;k<=(Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(similarity, 1),1)));++k){
degree(k-1, k-1) = (Helper::getDoubleFromOctaveListFirstResult(Fsum(Helper::convertToOctaveValueList(similarity.row(k-1)),1)));
}
}

};
#endif
