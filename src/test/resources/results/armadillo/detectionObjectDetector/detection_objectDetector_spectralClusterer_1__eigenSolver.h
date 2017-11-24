#ifndef DETECTION_OBJECTDETECTOR_SPECTRALCLUSTERER_1__EIGENSOLVER
#define DETECTION_OBJECTDETECTOR_SPECTRALCLUSTERER_1__EIGENSOLVER
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "armadillo.h"
#include "Helper.h"
#include "octave/builtin-defun-decls.h"
using namespace arma;
class detection_objectDetector_spectralClusterer_1__eigenSolver{
const int n = 2500;
public:
mat matrix;
mat eigenvectors;
void init()
{
matrix=mat(n,n);
eigenvectors=mat(n,targetEigenvectors);
}
void execute()
{
mat eigenVectors = (Helper::getColumnVectorFromOctaveListFirstResult(Feig(Helper::convertToOctaveValueList(matrix),2)));
double counter = 1;
double start = (Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(eigenVectors, 2),1)))-(targetEigenvectors-1);
for( auto i=start;i<=(Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(eigenVectors, 1),1)));++i){
eigenvectors.col(counter-1) = eigenVectors.col(i-1);
counter = counter+1;
}
}

};
#endif
